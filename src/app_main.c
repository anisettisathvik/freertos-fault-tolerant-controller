/* Fault-tolerant industrial controller: FreeRTOS application.
 *
 * Control decisions come from control_logic.c, the same file verified
 * exhaustively on the host. This file provides task wiring, timing and I/O.
 *
 * Tasks, highest priority first:
 *   HealthMonitor  5  watchdog owner; kicks only if every task checked in
 *   Processing     4  runs ctrl_step, drives the output pin
 *   Acquisition    3  samples the sensor, posts to the queue
 *   Command        2  simulates a master link
 *
 * The health monitor sits above the processing task: at a lower priority a
 * spinning processing task would starve the mechanism intended to detect it.
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "control_logic.h"

void uart_init(void);
void uart_puts(const char *s);

/* Microsecond clock. SysTick counts down 25000 core cycles per tick;
 * combining the tick count with the current reload value gives sub-millisecond
 * resolution without a second timer. The retry loop guards against the tick
 * incrementing between the two reads, which would give a 1 ms error at the
 * rollover. */
#define SYST_RVR_REG (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR_REG (*(volatile uint32_t *)0xE000E018u)
#define CYCLES_PER_US 25u

static uint32_t micros(void)
{
    uint32_t t1, t2, cvr;
    do {
        t1  = (uint32_t)xTaskGetTickCount();
        cvr = SYST_CVR_REG;
        t2  = (uint32_t)xTaskGetTickCount();
    } while (t1 != t2);
    return (t1 * 1000u) + ((SYST_RVR_REG - cvr) / CYCLES_PER_US);
}

/* ---- timing statistics --------------------------------------------------- */
typedef struct {
    uint32_t min_us, max_us, count;
    uint64_t sum_us;
} tstat_t;

static void tstat_init(tstat_t *t) { t->min_us = 0xFFFFFFFFu; t->max_us = 0; t->count = 0; t->sum_us = 0; }
static void tstat_add(tstat_t *t, uint32_t us)
{
    if (us < t->min_us) t->min_us = us;
    if (us > t->max_us) t->max_us = us;
    t->sum_us += us;
    t->count++;
}
static uint32_t tstat_avg(const tstat_t *t)
{
    return t->count ? (uint32_t)(t->sum_us / t->count) : 0u;
}

static tstat_t st_acq, st_proc, st_health, st_ctrl_step;

/* event latencies, measured once per occurrence rather than averaged */
static uint32_t g_fault_to_output_off_us = 0;
static uint32_t g_last_fault_us          = 0;
static tstat_t  st_recovery_ms;
static uint32_t g_fault_entry_ms         = 0;
static uint32_t g_watchdog_measured_ms   = 0;

/* The MPS2 machine model provides no independent watchdog, so the hardware
 * watchdog is emulated by a FreeRTOS one-shot timer with identical semantics:
 * if the health monitor stops resetting it, it expires and forces safe state. */
#define WATCHDOG_TIMEOUT_MS 1500
static volatile uint32_t g_watchdog_last_kick_ms = 0;
static volatile uint32_t g_watchdog_expiries     = 0;
static TimerHandle_t     g_watchdog_timer        = NULL;

/* ---- task check-in ------------------------------------------------------- */
#define TASK_ACQ  0
#define TASK_PROC 1
#define TASK_CMD  2
#define TASK_N    3
static volatile uint32_t g_checkin_ms[TASK_N];
#define CHECKIN_DEADLINE_MS 400

/* ---- shared ------------------------------------------------------------- */
typedef struct { int32_t value; uint32_t ts_ms; } sample_t;

static QueueHandle_t      g_sample_q;
static TaskHandle_t       g_proc_task = NULL;
static SemaphoreHandle_t  g_state_mutex;
static ctrl_t             g_ctrl;
static ctrl_outputs_t     g_out;

static volatile uint32_t g_queue_overflows = 0;
static volatile uint32_t g_commands_seen   = 0;
static volatile uint32_t g_steps           = 0;

/* Deterministic fault injection. A seeded schedule replays identically every
 * run, which is what makes a failure bisectable. */
typedef enum {
    INJ_NONE = 0, INJ_SENSOR_DEAD, INJ_SENSOR_IMPLAUSIBLE, INJ_SENSOR_JUMP,
    INJ_COMMS_DEAD, INJ_QUEUE_FLOOD, INJ_TASK_STALL
} injection_t;

typedef struct { uint32_t at_ms; uint32_t until_ms; injection_t what; } inj_step_t;

static const inj_step_t g_schedule[] = {
    { 3000,  4200,  INJ_SENSOR_DEAD        },
    { 7000,  7100,  INJ_SENSOR_IMPLAUSIBLE },
    { 10000, 10100, INJ_SENSOR_JUMP        },
    { 13000, 16000, INJ_COMMS_DEAD         },
    { 19000, 19200, INJ_QUEUE_FLOOD        },
    { 22000, 25600, INJ_TASK_STALL         },  /* longer than WATCHDOG_TIMEOUT_MS,
                                                 so the expiry path actually runs */
};
#define N_INJ (sizeof(g_schedule) / sizeof(g_schedule[0]))

static injection_t active_injection(uint32_t now)
{
    for (unsigned i = 0; i < N_INJ; i++) {
        if (now >= g_schedule[i].at_ms && now < g_schedule[i].until_ms)
            return g_schedule[i].what;
    }
    return INJ_NONE;
}

static uint32_t now_ms(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }

static void log_line(const char *s) { uart_puts(s); }

/* ---- output pin ---------------------------------------------------------- */
#define GPIO_OUT (*(volatile uint32_t *)0x40010004u)
static void set_output(uint8_t on) { GPIO_OUT = on ? 1u : 0u; }

/* ---- Acquisition, priority 3 --------------------------------------------- */
static void task_acquisition(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    int32_t    v    = 400;
    int        dir  = 1;

    for (;;) {
        const uint32_t t_enter = micros();
        const uint32_t t   = now_ms();
        const injection_t f = active_injection(t);
        g_checkin_ms[TASK_ACQ] = t;

        if (f != INJ_SENSOR_DEAD) {
            sample_t s;
            v += dir * 40;
            if (v > 820) dir = -1;
            if (v < 300)  dir =  1;

            s.value = v;
            if (f == INJ_SENSOR_IMPLAUSIBLE) s.value = 4000;   /* out of range */
            if (f == INJ_SENSOR_JUMP)        s.value = v + 500; /* rate trip   */
            s.ts_ms = t;

            const int flood = (f == INJ_QUEUE_FLOOD) ? 12 : 1;
            for (int i = 0; i < flood; i++) {
                if (xQueueSend(g_sample_q, &s, 0) != pdPASS) g_queue_overflows++;
            }
                        /* Direct-to-task notification: no separate object to allocate and
             * the give side is a single store rather than a queue operation.
             * The consumer keeps a timeout, so a lost notification degrades to
             * polling rather than deadlocking. */
            if (g_proc_task) xTaskNotifyGive(g_proc_task);
        }
        tstat_add(&st_acq, micros() - t_enter);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}

/* ---- Command link, priority 2 -------------------------------------------- */
static void task_command(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        const uint32_t t = now_ms();
        g_checkin_ms[TASK_CMD] = t;
        if (active_injection(t) != INJ_COMMS_DEAD) g_commands_seen++;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(250));
    }
}

/* ---- Processing, priority 4 ---------------------------------------------- */
static void task_processing(void *arg)
{
    (void)arg;
    TickType_t   last = xTaskGetTickCount();
    ctrl_state_t prev = ST_INIT;
    uint32_t     last_cmd_count = 0;

    for (;;) {
                /* Wait on a notification, with a timeout so the control loop still
         * runs when the sensor is dead -- the case the fault logic exists for. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        const uint32_t t_enter = micros();
        const uint32_t t = now_ms();
        const injection_t f = active_injection(t);

                /* A stalled task must not check in: this is the condition the health
         * monitor exists to detect. */
        if (f != INJ_TASK_STALL) g_checkin_ms[TASK_PROC] = t;

        sample_t s;
        ctrl_inputs_t in;
        memset(&in, 0, sizeof(in));
        in.now_ms = t;

        int drained = 0;
        while (xQueueReceive(g_sample_q, &s, 0) == pdPASS) {
            in.sensor_value = s.value;
            in.sensor_fresh = 1u;
            drained++;
        }
        if (drained > 4) g_queue_overflows++;

        in.command_fresh       = (g_commands_seen != last_cmd_count) ? 1u : 0u;
        last_cmd_count         = g_commands_seen;
        in.queue_overflowed    = (g_queue_overflows > 0) ? 1u : 0u;
        in.task_missed_checkin = 0u;

        for (int i = 0; i < TASK_N; i++) {
            if ((t - g_checkin_ms[i]) > CHECKIN_DEADLINE_MS) in.task_missed_checkin = 1u;
        }

        const uint8_t was_enabled = g_out.output_enabled;
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            const uint32_t c0 = micros();
            ctrl_step(&g_ctrl, &in, &g_out);
            tstat_add(&st_ctrl_step, micros() - c0);
            g_steps++;
            xSemaphoreGive(g_state_mutex);
        }

                /* Fault-to-output-off latency, measured from the top of the step that
         * detected the fault to the pin going low. */
        if (was_enabled && !g_out.output_enabled) {
            g_last_fault_us = t_enter;
        }
        set_output(g_out.output_enabled);
        if (g_last_fault_us) {
            const uint32_t lat = micros() - g_last_fault_us;
            if (lat > g_fault_to_output_off_us) g_fault_to_output_off_us = lat;
            g_last_fault_us = 0;
        }
        if (g_out.active_faults == 0u) g_queue_overflows = 0;

        if (g_out.state != prev) {
            if (g_out.state == ST_FAULT) g_fault_entry_ms = t;
            if (prev == ST_RECOVERING && g_out.state == ST_NORMAL &&
                g_fault_entry_ms) {
                tstat_add(&st_recovery_ms, t - g_fault_entry_ms);
                g_fault_entry_ms = 0;
            }
            char line[160];
            snprintf(line, sizeof(line),
                     "[%6lu ms] %-10s -> %-10s  out=%s faults=0x%02lX try=%u\n",
                     (unsigned long)t, ctrl_state_name(prev),
                     ctrl_state_name(g_out.state),
                     g_out.output_enabled ? "ON " : "OFF",
                     (unsigned long)g_out.active_faults,
                     g_out.recovery_attempt);
            log_line(line);
            prev = g_out.state;
        }

                /* A stalled task also stops delaying, monopolising the CPU against
         * every lower-priority task. */
        if (f == INJ_TASK_STALL) {
            for (volatile int i = 0; i < 20000; i++) { }
        }
        tstat_add(&st_proc, micros() - t_enter);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

/* ---- Health monitor, priority 5 (highest) -------------------------------- */
static void task_health(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    int reported = 0;

    for (;;) {
        const uint32_t t_enter = micros();
        const uint32_t t = now_ms();
        int all_alive = 1;

        for (int i = 0; i < TASK_N; i++) {
            if ((t - g_checkin_ms[i]) > CHECKIN_DEADLINE_MS) all_alive = 0;
        }

                /* The kick is conditional on every task having checked in. An
         * unconditional kick would prove only that the monitor itself runs. */
        if (all_alive) {
            g_watchdog_last_kick_ms = t;
                        /* Resetting the one-shot timer is the kick. If this stops
             * happening the timer expires and the callback forces safe state. */
            if (g_watchdog_timer) xTimerReset(g_watchdog_timer, 0);
            reported = 0;
        } else if ((t - g_watchdog_last_kick_ms) > WATCHDOG_TIMEOUT_MS) {
            set_output(0);
                        /* Counts expiry events, not iterations spent expired: the latter
             * measures observation time rather than fault frequency. */
            if (!reported) {
                char line[128];
                g_watchdog_expiries++;
                g_watchdog_measured_ms = t - g_watchdog_last_kick_ms;
                snprintf(line, sizeof(line),
                         "[%6lu ms] WATCHDOG EXPIRED after %lu ms without a kick\n",
                         (unsigned long)t,
                         (unsigned long)g_watchdog_measured_ms);
                log_line(line);
                reported = 1;
            }
        }
        tstat_add(&st_health, micros() - t_enter);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}

/* Watchdog expiry callback. One-shot, reset by every successful health check. */
static void watchdog_expired(TimerHandle_t xT)
{
    (void)xT;
    g_watchdog_expiries++;
    set_output(0);
    log_line("[watchdog] software timer expired - safe state forced\n");
}

/* ---- reporter ------------------------------------------------------------ */
static void task_report(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(29000));

    char line[320];
    snprintf(line, sizeof(line),
        "\n=== run summary ===\n"
        "  control steps ........ %lu\n"
        "  fault events ......... %lu\n"
        "  recoveries ........... %lu\n"
        "  latch events ......... %lu\n"
        "  watchdog expiries .... %lu\n"
        "  final state .......... %s (output %s)\n",
        (unsigned long)g_steps,
        (unsigned long)g_ctrl.fault_events,
        (unsigned long)g_ctrl.recoveries,
        (unsigned long)g_ctrl.latch_events,
        (unsigned long)g_watchdog_expiries,
        ctrl_state_name(g_out.state),
        g_out.output_enabled ? "ON" : "OFF");
    log_line(line);

    log_line("\n=== timing, microseconds ===\n");
    log_line("  task loop body        min      avg      max    count\n");
    snprintf(line, sizeof(line),
        "  Acquisition      %8lu %8lu %8lu %8lu\n"
        "  Processing       %8lu %8lu %8lu %8lu\n"
        "  HealthMonitor    %8lu %8lu %8lu %8lu\n"
        "  ctrl_step()      %8lu %8lu %8lu %8lu\n",
        (unsigned long)st_acq.min_us,   (unsigned long)tstat_avg(&st_acq),
        (unsigned long)st_acq.max_us,   (unsigned long)st_acq.count,
        (unsigned long)st_proc.min_us,  (unsigned long)tstat_avg(&st_proc),
        (unsigned long)st_proc.max_us,  (unsigned long)st_proc.count,
        (unsigned long)st_health.min_us,(unsigned long)tstat_avg(&st_health),
        (unsigned long)st_health.max_us,(unsigned long)st_health.count,
        (unsigned long)st_ctrl_step.min_us, (unsigned long)tstat_avg(&st_ctrl_step),
        (unsigned long)st_ctrl_step.max_us, (unsigned long)st_ctrl_step.count);
    log_line(line);

    log_line("\n=== event latencies ===\n");
    snprintf(line, sizeof(line),
        "  worst fault-to-output-off .... %lu us\n"
        "  recovery time (fault->normal)  min %lu ms  avg %lu ms  max %lu ms  n=%lu\n"
        "  watchdog deadline .............. %lu ms configured\n"
        "  watchdog detection latency ..... %lu ms\n",
        (unsigned long)g_fault_to_output_off_us,
        (unsigned long)(st_recovery_ms.count ? st_recovery_ms.min_us : 0),
        (unsigned long)tstat_avg(&st_recovery_ms),
        (unsigned long)st_recovery_ms.max_us,
        (unsigned long)st_recovery_ms.count,
        (unsigned long)WATCHDOG_TIMEOUT_MS,
        (unsigned long)g_watchdog_measured_ms);
    log_line(line);

    log_line("\n=== stack headroom, words remaining ===\n");
    snprintf(line, sizeof(line),
        "  Report task ........ %u of 768\n",
        (unsigned)uxTaskGetStackHighWaterMark(NULL));
    log_line(line);

    log_line("\nNote: these are QEMU cycle counts, not silicon. Relative costs\n"
             "and the control-flow latencies are meaningful; absolute numbers\n"
             "need a real part.\n");

    log_line("\nRUN COMPLETE\n");

    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

/* ---- hooks --------------------------------------------------------------- */
void vApplicationStackOverflowHook(TaskHandle_t t, char *name);
void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    (void)t;
    set_output(0);
    log_line("STACK OVERFLOW in ");
    log_line(name);
    log_line("\n");
    for (;;) { }
}

void vApplicationMallocFailedHook(void);
void vApplicationMallocFailedHook(void)
{
    set_output(0);
    log_line("MALLOC FAILED\n");
    for (;;) { }
}

void vApplicationAssert(const char *file, int line)
{
    char b[128];
    set_output(0);
    snprintf(b, sizeof(b), "ASSERT %s:%d\n", file, line);
    log_line(b);
    for (;;) { }
}

/* ---- entry --------------------------------------------------------------- */
int main(void)
{
    uart_init();
    log_line("\n=== Fault-Tolerant Industrial Controller ===\n");
    log_line("FreeRTOS on Cortex-M3, MPS2-AN385\n");
    log_line("deterministic fault schedule, 29 s run\n\n");

    tstat_init(&st_acq); tstat_init(&st_proc); tstat_init(&st_health);
    tstat_init(&st_ctrl_step); tstat_init(&st_recovery_ms);

    ctrl_init(&g_ctrl, 0u);
    memset(&g_out, 0, sizeof(g_out));
    for (int i = 0; i < TASK_N; i++) g_checkin_ms[i] = 0u;

    g_sample_q    = xQueueCreate(8, sizeof(sample_t));
    g_state_mutex = xSemaphoreCreateMutex();
    g_watchdog_timer = xTimerCreate("wdog", pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS),
                                    pdFALSE, NULL, watchdog_expired);
    if (!g_sample_q || !g_state_mutex || !g_watchdog_timer) {
        log_line("alloc failed\n"); for (;;) { }
    }
    xTimerStart(g_watchdog_timer, 0);

    xTaskCreate(task_health,      "Health", 512, NULL, 5, NULL);
    xTaskCreate(task_processing,  "Proc",   768, NULL, 4, &g_proc_task);
    xTaskCreate(task_acquisition, "Acq",    512, NULL, 3, NULL);
    xTaskCreate(task_command,     "Cmd",    512, NULL, 2, NULL);
    xTaskCreate(task_report,      "Report", 768, NULL, 1, NULL);

    vTaskStartScheduler();
    for (;;) { }
}
