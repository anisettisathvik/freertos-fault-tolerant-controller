#include "control_logic.h"

/* ---- helpers ------------------------------------------------------------ */

static uint32_t elapsed(uint32_t now, uint32_t then)
{
        /* Unsigned subtraction is wrap-correct. A `now > then` comparison fails
     * once, 49.7 days after boot at 1 kHz, and is unreproducible in the field. */
    return now - then;
}

uint8_t ctrl_output_allowed(ctrl_state_t s)
{
    return (s == ST_NORMAL || s == ST_WARNING) ? 1u : 0u;
}

const char* ctrl_state_name(ctrl_state_t s)
{
    switch (s) {
    case ST_INIT:       return "INIT";
    case ST_NORMAL:     return "NORMAL";
    case ST_WARNING:    return "WARNING";
    case ST_FAULT:      return "FAULT";
    case ST_SAFE:       return "SAFE";
    case ST_RECOVERING: return "RECOVERING";
    case ST_LATCHED:    return "LATCHED";
    default:            return "?";
    }
}

const char* ctrl_fault_name(uint32_t single_flag)
{
    switch (single_flag) {
    case F_SENSOR_TIMEOUT:     return "sensor timeout";
    case F_SENSOR_IMPLAUSIBLE: return "sensor implausible";
    case F_SENSOR_RATE:        return "sensor rate";
    case F_COMMS_TIMEOUT:      return "comms timeout";
    case F_QUEUE_OVERFLOW:     return "queue overflow";
    case F_TASK_UNRESPONSIVE:  return "task unresponsive";
    default:                   return "?";
    }
}

void ctrl_init(ctrl_t* c, uint32_t now_ms)
{
    c->state            = ST_INIT;
    c->last_sensor_ms   = now_ms;
    c->last_command_ms  = now_ms;
    c->last_value       = 0;
    c->have_last_value  = 0u;
    c->active_faults    = 0u;
    c->state_entered_ms = now_ms;
    c->good_streak      = 0u;
    c->recovery_attempt = 0u;
    c->fault_events     = 0u;
    c->recoveries       = 0u;
    c->latch_events     = 0u;
}

static void enter(ctrl_t* c, ctrl_state_t s, uint32_t now)
{
    if (c->state != s) {
        c->state            = s;
        c->state_entered_ms = now;
        c->good_streak      = 0u;
    }
}

/* Fault detection, kept separate from the transition logic so the two can be
 * reasoned about and tested independently. */
static uint32_t detect_faults(const ctrl_t* c, const ctrl_inputs_t* in)
{
    uint32_t f = 0u;

    if (in->sensor_fresh) {
        if (in->sensor_value < SENSOR_MIN_VALUE ||
            in->sensor_value > SENSOR_MAX_VALUE) {
            f |= F_SENSOR_IMPLAUSIBLE;
        }
                /* Rate is checked only against a value already held; comparing against
         * an uninitialised value would fault the controller at power-up. */
        if (c->have_last_value) {
            const int32_t d = in->sensor_value - c->last_value;
            const int32_t ad = (d < 0) ? -d : d;
            if (ad > SENSOR_MAX_DELTA) {
                f |= F_SENSOR_RATE;
            }
        }
    } else {
        if (elapsed(in->now_ms, c->last_sensor_ms) > SENSOR_DEADLINE_MS) {
            f |= F_SENSOR_TIMEOUT;
        }
    }

    if (!in->command_fresh &&
        elapsed(in->now_ms, c->last_command_ms) > COMMS_DEADLINE_MS) {
        f |= F_COMMS_TIMEOUT;
    }

    if (in->queue_overflowed)    f |= F_QUEUE_OVERFLOW;
    if (in->task_missed_checkin) f |= F_TASK_UNRESPONSIVE;

    return f;
}

void ctrl_step(ctrl_t* c, const ctrl_inputs_t* in, ctrl_outputs_t* out)
{
    uint32_t faults;

        /* Freshness is recorded before detection so a reading arriving in the same
     * step it was due does not trip the deadline. */
    if (in->sensor_fresh)  c->last_sensor_ms  = in->now_ms;
    if (in->command_fresh) c->last_command_ms = in->now_ms;

    faults = detect_faults(c, in);
    c->active_faults = faults;

        /* Recorded only after passing plausibility: storing a rejected value would
     * poison the next rate check and mask the fault. */
    if (in->sensor_fresh &&
        !(faults & (F_SENSOR_IMPLAUSIBLE | F_SENSOR_RATE))) {
        c->last_value      = in->sensor_value;
        c->have_last_value = 1u;
    }

    switch (c->state) {

    case ST_INIT:
        if (faults) {
            c->fault_events++;
            enter(c, ST_FAULT, in->now_ms);
        } else if (in->sensor_fresh) {
            enter(c, ST_NORMAL, in->now_ms);
        }
        break;

    case ST_NORMAL:
        if (faults) {
            c->fault_events++;
            enter(c, ST_FAULT, in->now_ms);
        } else if (in->sensor_fresh && in->sensor_value >= SENSOR_WARN_VALUE) {
            enter(c, ST_WARNING, in->now_ms);
        }
        break;

    case ST_WARNING:
        if (faults) {
            c->fault_events++;
            enter(c, ST_FAULT, in->now_ms);
        } else if (in->sensor_fresh && in->sensor_value < SENSOR_WARN_VALUE) {
            enter(c, ST_NORMAL, in->now_ms);
        }
        break;

    case ST_FAULT:
                /* FAULT is a single step: detect, cut the output, then hold in SAFE.
         * Remaining here would prevent the cooldown from starting. */
        enter(c, ST_SAFE, in->now_ms);
        break;

    case ST_SAFE:
        if (elapsed(in->now_ms, c->state_entered_ms) >= SAFE_COOLDOWN_MS) {
            if (c->recovery_attempt >= RECOVERY_MAX_TRIES) {
                c->latch_events++;
                enter(c, ST_LATCHED, in->now_ms);
            } else {
                c->recovery_attempt++;
                enter(c, ST_RECOVERING, in->now_ms);
            }
        }
        break;

    case ST_RECOVERING:
        if (faults) {
                        /* A fault during recovery consumes budget and returns through
             * SAFE, never directly to an operating state. */
            c->fault_events++;
            enter(c, ST_FAULT, in->now_ms);
        } else if (in->sensor_fresh) {
            c->good_streak++;
            if (c->good_streak >= RECOVERY_GOOD_NEEDED) {
                c->recoveries++;
                c->recovery_attempt = 0u;
                enter(c, ST_NORMAL, in->now_ms);
            }
        }
        break;

    case ST_LATCHED:
                /* Absorbing by design: repeated automatic recovery on a failed plant
         * risks equipment damage, so a human must intervene. */
        if (in->manual_reset) {
            c->recovery_attempt = 0u;
            enter(c, ST_INIT, in->now_ms);
        }
        break;

    default:
        enter(c, ST_FAULT, in->now_ms);
        break;
    }

    out->state            = c->state;
    out->output_enabled   = ctrl_output_allowed(c->state);
    out->active_faults    = c->active_faults;
    out->recovery_attempt = c->recovery_attempt;
}
