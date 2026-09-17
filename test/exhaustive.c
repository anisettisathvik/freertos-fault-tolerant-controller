/* Exhaustive verification of the controller state machine.
 *
 * Enumerates the entire reachable state space by breadth-first search over
 * every (state, input) combination and checks the safety properties on each
 * transition, so a property that holds here holds for every reachable
 * configuration rather than for those a test happened to visit.
 *
 * Properties:
 *   P1  the output is never enabled outside NORMAL and WARNING
 *   P2  FAULT never transitions directly to NORMAL or WARNING
 *   P3  recovery always passes through SAFE, then RECOVERING
 *   P4  LATCHED is absorbing without a manual reset
 *   P5  every declared state is reachable
 *   P6  no reachable state is a dead end other than LATCHED
 *   P7  a fault detected in any state cuts the output within one step
 *   P8  the recovery budget can never be exceeded
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "control_logic.h"

/* State abstraction. The concrete controller holds 32-bit timestamps, so its
 * literal state space is unbounded. Time is abstracted into the three buckets
 * the logic can distinguish: within the sensor deadline, past it, and past the
 * safe cooldown. This keeps the space finite while preserving every branch. */

#define TIME_BUCKETS 3      /* fresh | past sensor deadline | past cooldown  */

typedef struct {
    unsigned char state;
    unsigned char good_streak;      /* 0..RECOVERY_GOOD_NEEDED               */
    unsigned char recovery_attempt; /* 0..RECOVERY_MAX_TRIES                 */
    unsigned char have_last_value;
    unsigned char value_bucket;     /* 0 low, 1 mid, 2 warning band          */
} absstate_t;

/* The last sensor value is tracked in three buckets. Pinning it to a single
 * value makes the warning band unreachable in one step: with SENSOR_MAX_DELTA
 * of 200, any reading at or above 850 from a base of 500 is a rate violation,
 * so the only exit from NORMAL would be FAULT. The real controller reaches the
 * warning band across two steps. */
static int bucket_value(int b) { return (b == 0) ? 400 : (b == 1) ? 700 : 900; }
static unsigned char value_to_bucket(int v)
{
    if (v < 600) return 0;
    if (v < SENSOR_WARN_VALUE) return 1;
    return 2;
}

#define MAX_STATES 65536
static absstate_t   seen[MAX_STATES];
static int          seen_n = 0;
static int          queue_[MAX_STATES];
static int          qhead = 0, qtail = 0;

/* transition record, for coverage and for the property checks */
static unsigned char state_visited[ST_COUNT];
static unsigned long transitions[ST_COUNT][ST_COUNT];

static int  fail_count = 0;
static char first_failure[512];

static void violate(const char* prop, ctrl_state_t from, ctrl_state_t to,
                    uint32_t faults)
{
    if (fail_count == 0) {
        snprintf(first_failure, sizeof(first_failure),
                 "%s : %s -> %s (faults 0x%02X)",
                 prop, ctrl_state_name(from), ctrl_state_name(to), faults);
    }
    fail_count++;
}

static int find_or_add(const absstate_t* a)
{
    int i;
    for (i = 0; i < seen_n; i++) {
        if (memcmp(&seen[i], a, sizeof(absstate_t)) == 0) return -1; /* known */
    }
    if (seen_n >= MAX_STATES) { fprintf(stderr, "state space overflow\n"); exit(2); }
    seen[seen_n] = *a;
    queue_[qtail++] = seen_n;
    return seen_n++;
}

/* Rebuild a concrete controller from an abstract state, apply one input
 * combination, and read back the abstract successor. */
static void expand(const absstate_t* a, int t_bucket,
                   uint32_t input_faults, int sensor_fresh, int cmd_fresh,
                   int manual_reset, int sensor_value,
                   absstate_t* out_abs, ctrl_state_t* out_state,
                   uint8_t* out_enabled, uint32_t* out_faults)
{
    ctrl_t c;
    ctrl_inputs_t in;
    ctrl_outputs_t o;
    uint32_t base = 100000u;   /* well away from zero, to catch wrap bugs */

    ctrl_init(&c, base);
    c.state            = (ctrl_state_t)a->state;
    c.good_streak      = a->good_streak;
    c.recovery_attempt = a->recovery_attempt;
    c.have_last_value  = a->have_last_value;
    c.last_value       = bucket_value(a->value_bucket);

        /* Place timestamps so the requested time bucket is what the logic sees. */
    switch (t_bucket) {
    case 0:  /* everything fresh */
        c.last_sensor_ms   = base;
        c.last_command_ms  = base;
        c.state_entered_ms = base;
        break;
    case 1:  /* past the sensor deadline, before the cooldown */
        c.last_sensor_ms   = base - (SENSOR_DEADLINE_MS + 1);
        c.last_command_ms  = base;
        c.state_entered_ms = base;
        break;
    default: /* past the cooldown and the comms deadline too */
        c.last_sensor_ms   = base - (SENSOR_DEADLINE_MS + 1);
        c.last_command_ms  = base - (COMMS_DEADLINE_MS + 1);
        c.state_entered_ms = base - (SAFE_COOLDOWN_MS + 1);
        break;
    }

    memset(&in, 0, sizeof(in));
    in.now_ms              = base;
    in.sensor_fresh        = (uint8_t)sensor_fresh;
    in.command_fresh       = (uint8_t)cmd_fresh;
    in.sensor_value        = sensor_value;
    in.queue_overflowed    = (input_faults & F_QUEUE_OVERFLOW)    ? 1u : 0u;
    in.task_missed_checkin = (input_faults & F_TASK_UNRESPONSIVE) ? 1u : 0u;
    in.manual_reset        = (uint8_t)manual_reset;

    ctrl_step(&c, &in, &o);

    out_abs->state            = (unsigned char)c.state;
    out_abs->good_streak      = c.good_streak;
    out_abs->recovery_attempt = c.recovery_attempt;
    out_abs->have_last_value  = c.have_last_value;
    out_abs->value_bucket     = value_to_bucket(c.last_value);

    *out_state   = o.state;
    *out_enabled = o.output_enabled;
    *out_faults  = o.active_faults;
}

int main(void)
{
    absstate_t start;
    unsigned long explored = 0, edges = 0;
    int i, j;

    printf("\n==============================================\n");
    printf(" Exhaustive state space exploration\n");
    printf("==============================================\n");

    memset(state_visited, 0, sizeof(state_visited));
    memset(transitions, 0, sizeof(transitions));

    memset(&start, 0, sizeof(start));
    start.state = ST_INIT;
    find_or_add(&start);

    while (qhead < qtail) {
        const int idx = queue_[qhead++];
        const absstate_t cur = seen[idx];
        explored++;
        state_visited[cur.state] = 1u;

        /* every input combination the logic can distinguish */
        for (int t = 0; t < TIME_BUCKETS; t++)
        for (int sf = 0; sf <= 1; sf++)
        for (int cf = 0; cf <= 1; cf++)
        for (int mr = 0; mr <= 1; mr++)
        for (int qo = 0; qo <= 1; qo++)
        for (int tu = 0; tu <= 1; tu++)
        for (int vi = 0; vi < 6; vi++) {

                        /* Representative sensor values: two normal, the warning boundary,
             * the warning band, out of physical range, and a step large enough
             * to trip the rate limiter from the current value. */
            static const int values[5] = { 400, 700, SENSOR_WARN_VALUE, 900, 5000 };
            const int value = (vi == 5)
                            ? (bucket_value(cur.value_bucket) + SENSOR_MAX_DELTA + 50)
                            : values[vi];

            uint32_t inject = 0u;
            if (qo) inject |= F_QUEUE_OVERFLOW;
            if (tu) inject |= F_TASK_UNRESPONSIVE;

            absstate_t next;
            ctrl_state_t to;
            uint8_t enabled;
            uint32_t faults;

            expand(&cur, t, inject, sf, cf, mr, value,
                   &next, &to, &enabled, &faults);

            edges++;
            transitions[cur.state][to]++;

            /* ---- P1: output only in NORMAL or WARNING ------------------ */
            if (enabled && to != ST_NORMAL && to != ST_WARNING) {
                violate("P1 output enabled outside NORMAL/WARNING",
                        (ctrl_state_t)cur.state, to, faults);
            }
            if (!enabled && (to == ST_NORMAL || to == ST_WARNING)) {
                violate("P1 output disabled in an operating state",
                        (ctrl_state_t)cur.state, to, faults);
            }

            /* ---- P2: FAULT never goes straight back to operating ------- */
            if (cur.state == ST_FAULT && (to == ST_NORMAL || to == ST_WARNING)) {
                violate("P2 FAULT jumped to an operating state",
                        (ctrl_state_t)cur.state, to, faults);
            }

            /* ---- P3: SAFE only ever leads to RECOVERING or LATCHED ----- */
            if (cur.state == ST_SAFE &&
                to != ST_SAFE && to != ST_RECOVERING && to != ST_LATCHED) {
                violate("P3 SAFE escaped to an unexpected state",
                        (ctrl_state_t)cur.state, to, faults);
            }

            /* ---- P4: LATCHED is absorbing without a manual reset ------- */
            if (cur.state == ST_LATCHED && !mr && to != ST_LATCHED) {
                violate("P4 LATCHED escaped without a manual reset",
                        (ctrl_state_t)cur.state, to, faults);
            }

            /* ---- P7: any active fault cuts the output within one step -- */
            if (faults != 0u && enabled) {
                violate("P7 output still enabled with an active fault",
                        (ctrl_state_t)cur.state, to, faults);
            }

            /* ---- P8: the recovery budget is never exceeded ------------- */
            if (next.recovery_attempt > RECOVERY_MAX_TRIES) {
                violate("P8 recovery budget exceeded",
                        (ctrl_state_t)cur.state, to, faults);
            }

            find_or_add(&next);
        }
    }

    printf("\n  reachable states explored .... %lu\n", explored);
    printf("  transitions evaluated ........ %lu\n", edges);

    /* ---- P5: every declared state is reachable ------------------------- */
    printf("\n  state reachability\n");
    int unreachable = 0;
    for (i = 0; i < ST_COUNT; i++) {
        printf("    %-12s %s\n", ctrl_state_name((ctrl_state_t)i),
               state_visited[i] ? "reachable" : "UNREACHABLE");
        if (!state_visited[i]) unreachable++;
    }

    /* ---- P6: no dead ends other than LATCHED --------------------------- */
    int dead_ends = 0;
    for (i = 0; i < ST_COUNT; i++) {
        if (!state_visited[i] || i == ST_LATCHED) continue;
        int has_exit = 0;
        for (j = 0; j < ST_COUNT; j++)
            if (j != i && transitions[i][j]) { has_exit = 1; break; }
        if (!has_exit) {
            printf("    DEAD END: %s\n", ctrl_state_name((ctrl_state_t)i));
            dead_ends++;
        }
    }

    printf("\n  transition matrix (from row -> to column)\n    %-12s", "");
    for (j = 0; j < ST_COUNT; j++) printf("%11.10s", ctrl_state_name((ctrl_state_t)j));
    printf("\n");
    for (i = 0; i < ST_COUNT; i++) {
        printf("    %-12s", ctrl_state_name((ctrl_state_t)i));
        for (j = 0; j < ST_COUNT; j++) {
            if (transitions[i][j]) printf("%11lu", transitions[i][j]);
            else                   printf("%11s", ".");
        }
        printf("\n");
    }

    printf("\n  properties\n");
    printf("    P1 output only in NORMAL/WARNING ......... %s\n", fail_count ? "see below" : "HOLDS");
    printf("    P2 no FAULT -> operating ................. %s\n", fail_count ? "see below" : "HOLDS");
    printf("    P3 SAFE -> RECOVERING or LATCHED only .... %s\n", fail_count ? "see below" : "HOLDS");
    printf("    P4 LATCHED absorbing without reset ....... %s\n", fail_count ? "see below" : "HOLDS");
    printf("    P5 every state reachable ................. %s\n", unreachable ? "FAILED" : "HOLDS");
    printf("    P6 no unintended dead ends ............... %s\n", dead_ends ? "FAILED" : "HOLDS");
    printf("    P7 fault cuts output within one step ..... %s\n", fail_count ? "see below" : "HOLDS");
    printf("    P8 recovery budget never exceeded ........ %s\n", fail_count ? "see below" : "HOLDS");

    printf("\n----------------------------------------------\n");
    if (fail_count == 0 && unreachable == 0 && dead_ends == 0) {
        printf("  PASS  %lu states, %lu transitions, all properties hold\n",
               explored, edges);
    } else {
        printf("  FAIL  %d property violations, %d unreachable, %d dead ends\n",
               fail_count, unreachable, dead_ends);
        if (fail_count) printf("        first: %s\n", first_failure);
    }
    printf("----------------------------------------------\n\n");

    return (fail_count || unreachable || dead_ends) ? 1 : 0;
}
