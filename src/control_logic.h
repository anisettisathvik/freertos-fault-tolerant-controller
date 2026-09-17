#ifndef CONTROL_LOGIC_H
#define CONTROL_LOGIC_H

#include <stdint.h>

/* Fault-tolerant industrial controller: decision logic.
 *
 * A pure function of (current state, inputs) -> (next state, outputs), with no
 * RTOS, register or timer dependency. The reachable state space is therefore
 * finite and small enough to enumerate completely, which is what allows the
 * safety properties in test/exhaustive.c to be proved rather than sampled.
 */

/* ---- states ------------------------------------------------------------- */
typedef enum {
    ST_INIT = 0,     /* Powered up, no valid reading yet                     */
    ST_NORMAL,       /* healthy, output enabled                              */
    ST_WARNING,      /* degraded but operating, output still enabled         */
    ST_FAULT,        /* fault detected this step, output cut immediately     */
    ST_SAFE,         /* holding safe state, waiting out the cooldown         */
    ST_RECOVERING,   /* probing: needs consecutive good readings to return   */
    ST_LATCHED,      /* recovery budget exhausted; only a manual reset exits */
    ST_COUNT
} ctrl_state_t;

/* ---- fault flags -------------------------------------------------------- */
#define F_SENSOR_TIMEOUT     (1u << 0)  /* no reading within the deadline    */
#define F_SENSOR_IMPLAUSIBLE (1u << 1)  /* outside the physical range        */
#define F_SENSOR_RATE        (1u << 2)  /* changed faster than physics allows*/
#define F_COMMS_TIMEOUT      (1u << 3)  /* no master command within deadline */
#define F_QUEUE_OVERFLOW     (1u << 4)  /* producer outran the consumer      */
#define F_TASK_UNRESPONSIVE  (1u << 5)  /* a task missed its health check-in */
#define F_ALL                0x3Fu
#define F_COUNT              6

/* ---- tuning ------------------------------------------------------------- */
#define SENSOR_MIN_VALUE     0
#define SENSOR_MAX_VALUE     1000
#define SENSOR_WARN_VALUE    850     /* degraded, not yet a fault            */
#define SENSOR_MAX_DELTA     200     /* per step; larger implies a bad read  */
#define SENSOR_DEADLINE_MS   500
#define COMMS_DEADLINE_MS    2000
#define SAFE_COOLDOWN_MS     1000
#define RECOVERY_GOOD_NEEDED 3       /* consecutive clean steps to return    */
#define RECOVERY_MAX_TRIES   3       /* then latch, permanently              */

typedef struct {
    uint32_t now_ms;
    int32_t  sensor_value;
    uint8_t  sensor_fresh;      /* a new reading arrived this step           */
    uint8_t  command_fresh;     /* a master command arrived this step        */
    uint8_t  queue_overflowed;
    uint8_t  task_missed_checkin;
    uint8_t  manual_reset;      /* The only exit from ST_LATCHED             */
} ctrl_inputs_t;

typedef struct {
    ctrl_state_t state;
    uint8_t      output_enabled;
    uint32_t     active_faults;
    uint8_t      recovery_attempt;
} ctrl_outputs_t;

typedef struct {
    ctrl_state_t state;
    uint32_t     last_sensor_ms;
    uint32_t     last_command_ms;
    int32_t      last_value;
    uint8_t      have_last_value;
    uint32_t     active_faults;
    uint32_t     state_entered_ms;
    uint8_t      good_streak;
    uint8_t      recovery_attempt;
    /* diagnostics */
    uint32_t     fault_events;
    uint32_t     recoveries;
    uint32_t     latch_events;
} ctrl_t;

void ctrl_init(ctrl_t* c, uint32_t now_ms);

/* One control step. Deterministic: identical state and inputs always produce
 * the same successor, which the exhaustive explorer depends on. */
void ctrl_step(ctrl_t* c, const ctrl_inputs_t* in, ctrl_outputs_t* out);

/* Output is permitted only in the operating states. Asserted after every step
 * in verification and used by the firmware to drive the output pin. */
uint8_t ctrl_output_allowed(ctrl_state_t s);

const char* ctrl_state_name(ctrl_state_t s);
const char* ctrl_fault_name(uint32_t single_flag);

#endif /* CONTROL_LOGIC_H */
