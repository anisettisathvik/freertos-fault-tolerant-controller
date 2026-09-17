/* Directed tests for the control logic. The exhaustive explorer proves the
 * safety properties; these document the intended behaviour case by case. */

#include <stdio.h>
#include <string.h>
#include "control_logic.h"

static int pass = 0, fail = 0;
static void check(const char *n, int ok)
{
    if (ok) { pass++; printf("  PASS  %s\n", n); }
    else    { fail++; printf("  FAIL  %s\n", n); }
}

static ctrl_t c;
static ctrl_outputs_t o;

static void step(uint32_t t, int value, int fresh, int cmd)
{
    ctrl_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.now_ms = t; in.sensor_value = value;
    in.sensor_fresh = (uint8_t)fresh; in.command_fresh = (uint8_t)cmd;
    ctrl_step(&c, &in, &o);
}

static void step_full(uint32_t t, int value, int fresh, int cmd,
                      int qo, int stall, int reset)
{
    ctrl_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.now_ms = t; in.sensor_value = value;
    in.sensor_fresh = (uint8_t)fresh; in.command_fresh = (uint8_t)cmd;
    in.queue_overflowed = (uint8_t)qo; in.task_missed_checkin = (uint8_t)stall;
    in.manual_reset = (uint8_t)reset;
    ctrl_step(&c, &in, &o);
}

int main(void)
{
    printf("\n=== startup ===\n");
    ctrl_init(&c, 1000);
    step(1000, 400, 0, 1);
    check("stays in INIT before the first reading", o.state == ST_INIT);
    check("output off during INIT", !o.output_enabled);
    step(1100, 400, 1, 1);
    check("first good reading enters NORMAL", o.state == ST_NORMAL);
    check("output on in NORMAL", o.output_enabled);

    printf("\n=== warning band ===\n");
    step(1200, 550, 1, 1);
    step(1300, 700, 1, 1);
    step(1400, 860, 1, 1);
    check("crossing the warn threshold enters WARNING", o.state == ST_WARNING);
    check("output stays on in WARNING", o.output_enabled);
    step(1500, 700, 1, 1);
    check("dropping below the threshold returns to NORMAL", o.state == ST_NORMAL);

    printf("\n=== sensor timeout ===\n");
    ctrl_init(&c, 1000); step(1000, 400, 1, 1);
    step(1000 + SENSOR_DEADLINE_MS + 100, 0, 0, 1);
    check("missing readings raise a timeout fault",
          o.active_faults & F_SENSOR_TIMEOUT);
    check("fault cuts the output immediately", !o.output_enabled);
    check("state is FAULT", o.state == ST_FAULT);

    printf("\n=== implausible value ===\n");
    ctrl_init(&c, 1000); step(1000, 400, 1, 1);
    step(1100, 9999, 1, 1);
    check("out-of-range reading is rejected",
          o.active_faults & F_SENSOR_IMPLAUSIBLE);
    check("output cut", !o.output_enabled);

    printf("\n=== negative reading ===\n");
    {
                /* A broken ADC, a disconnected thermocouple or a signed conversion bug
         * all produce negative readings; a controller checking only the upper
         * bound would act on one. */
        ctrl_init(&c, 1000);
        step(1000, 400, 1, 1);
        check("healthy before the negative reading", o.state == ST_NORMAL);
        step(1100, -50, 1, 1);
        check("negative value raises implausible",
              o.active_faults & F_SENSOR_IMPLAUSIBLE);
        check("output cut on a negative reading", !o.output_enabled);
    }

    printf("\n=== rate limit ===\n");
    ctrl_init(&c, 1000); step(1000, 400, 1, 1);
    step(1100, 400 + SENSOR_MAX_DELTA + 1, 1, 1);
    check("impossible jump raises a rate fault", o.active_faults & F_SENSOR_RATE);

    printf("\n=== recovery path ===\n");
    ctrl_init(&c, 1000); step(1000, 400, 1, 1);
    step(1100, 9999, 1, 1);                       /* fault  */
    check("FAULT entered", o.state == ST_FAULT);
    step(1150, 400, 1, 1);
    check("FAULT moves to SAFE in one step", o.state == ST_SAFE);
    step(1200, 400, 1, 1);
    check("SAFE holds during the cooldown", o.state == ST_SAFE);
    step(1150 + SAFE_COOLDOWN_MS + 50, 400, 1, 1);
    check("cooldown expiry enters RECOVERING", o.state == ST_RECOVERING);
    check("output still off while recovering", !o.output_enabled);
    {
        uint32_t t = 1150 + SAFE_COOLDOWN_MS + 100;
        for (int i = 0; i < RECOVERY_GOOD_NEEDED; i++) { step(t, 400, 1, 1); t += 50; }
    }
    check("consecutive good readings restore NORMAL", o.state == ST_NORMAL);
    check("output restored", o.output_enabled);
    check("recovery counter incremented", c.recoveries == 1);

    printf("\n=== latching after repeated failure ===\n");
    ctrl_init(&c, 1000); step(1000, 400, 1, 1);
    {
        uint32_t t = 1100;
        for (int attempt = 0; attempt < RECOVERY_MAX_TRIES + 1; attempt++) {
            step_full(t, 400, 1, 1, 1, 0, 0);   t += 50;   /* queue overflow */
            step_full(t, 400, 1, 1, 0, 0, 0);   t += 50;   /* FAULT -> SAFE  */
            t += SAFE_COOLDOWN_MS + 50;
            step_full(t, 400, 1, 1, 0, 0, 0);   t += 50;   /* -> RECOVERING  */
        }
        check("recovery budget exhausted latches the controller",
              o.state == ST_LATCHED);
        check("output off while latched", !o.output_enabled);
        check("latch event recorded", c.latch_events == 1);

        step_full(t, 400, 1, 1, 0, 0, 0); t += 50;
        check("latched state ignores good readings", o.state == ST_LATCHED);

        step_full(t, 400, 1, 1, 0, 0, 1);
        check("manual reset is the only exit", o.state == ST_INIT);
    }

    printf("\n=== timer wrap ===\n");
    {
                /* The millisecond tick wraps 49.7 days after boot. A `now > then`
         * comparison fails exactly once, here. */
        ctrl_init(&c, 0xFFFFFF00u);
        step(0xFFFFFF00u, 400, 1, 1);
        check("healthy just before the wrap", o.state == ST_NORMAL);
        step(0x00000050u, 420, 1, 1);   /* wrapped: elapsed is 0x150 = 336 ms */
        check("no spurious fault across the 32-bit wrap",
              !(o.active_faults & F_SENSOR_TIMEOUT));
    }

    printf("\n=== corrupted state must fail safe ===\n");
    {
                /* The default arm of the transition switch guards against a state
         * variable corrupted by a stack overflow or bit flip. It is unreachable
         * through the API, which is why it requires an explicit test. */
        ctrl_init(&c, 1000);
        step(1000, 400, 1, 1);
        check("healthy before corruption", o.state == ST_NORMAL);

        c.state = (ctrl_state_t)99;      /* not a valid enum value */
        step(1100, 400, 1, 1);
        check("corrupted state forced to FAULT", o.state == ST_FAULT);
        check("output cut on corruption", !o.output_enabled);
    }

    printf("\n=== diagnostic name helpers ===\n");
    {
                /* Reached only from the firmware's logging path. */
        int names_ok = 1;
        const ctrl_state_t all[] = { ST_INIT, ST_NORMAL, ST_WARNING, ST_FAULT,
                                     ST_SAFE, ST_RECOVERING, ST_LATCHED };
        for (unsigned i = 0; i < sizeof(all)/sizeof(all[0]); i++) {
            if (ctrl_state_name(all[i])[0] == '?') names_ok = 0;
        }
        check("every state has a name", names_ok);
        check("invalid state name is guarded",
              ctrl_state_name((ctrl_state_t)99)[0] == '?');

        const uint32_t flags[] = { F_SENSOR_TIMEOUT, F_SENSOR_IMPLAUSIBLE,
                                   F_SENSOR_RATE, F_COMMS_TIMEOUT,
                                   F_QUEUE_OVERFLOW, F_TASK_UNRESPONSIVE };
        int flags_ok = 1;
        for (unsigned i = 0; i < sizeof(flags)/sizeof(flags[0]); i++) {
            if (ctrl_fault_name(flags[i])[0] == '?') flags_ok = 0;
        }
        check("every fault flag has a name", flags_ok);
        check("unknown fault flag is guarded", ctrl_fault_name(0x800u)[0] == '?');
    }

    printf("\n---------------------------------------\n");
    printf("  %d passed, %d failed\n", pass, fail);
    printf("---------------------------------------\n\n");
    return fail ? 1 : 0;
}
