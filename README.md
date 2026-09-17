# Fault-Tolerant Real-Time Industrial Controller — FreeRTOS on Cortex-M3

A four-task FreeRTOS controller for an industrial sensor loop, with a
seven-state fault machine covering sensor timeout, implausible and out-of-rate
readings, communication loss, queue overflow and unresponsive tasks, plus
cut-to-safe-state, a bounded recovery budget and permanent latching. The control
logic is separated from the RTOS so that its reachable state space is finite:
eight safety properties are therefore proved by exhaustive exploration rather
than sampled by tests. The firmware is real ARMv7-M built with
`arm-none-eabi-gcc`, with its own vector table and linker script, running on
QEMU's MPS2-AN385 Cortex-M3 model.

---

## Repository layout

```
freertos-fault-tolerant-controller/
├── src/
│   ├── control_logic.h       states, fault flags, tuning constants
│   ├── control_logic.c       fault detection and transition logic (no RTOS)
│   ├── app_main.c            FreeRTOS tasks, watchdog, fault injection
│   └── FreeRTOSConfig.h      kernel configuration for Cortex-M3
├── port/
│   ├── startup_mps2.c        vector table, reset handler, C runtime init
│   └── uart.c                CMSDK UART driver and newlib stubs
├── linker/
│   └── mps2_an385.ld         memory map for the MPS2-AN385
├── test/
│   ├── test_logic.c          directed tests
│   └── exhaustive.c          state-space exploration and property checking
├── tools/
│   ├── gdb_session.sh        GDB against the target over QEMU's gdbserver
│   └── timing_runs.sh        repeated runs, reports timing spread
├── .github/workflows/ci.yml
├── SETUP.md
└── Makefile
```

`freertos/` is not committed — see SETUP.md.

---

## Requirements

```bash
sudo apt install build-essential gcc-arm-none-eabi qemu-system-arm \
                 gdb-multiarch cppcheck
```

The FreeRTOS kernel is third-party and fetched separately:

```bash
git clone --depth 1 --branch V11.1.0 \
    https://github.com/FreeRTOS/FreeRTOS-Kernel.git freertos
```

## Build and run

```bash
make logic       # directed tests, ASan + UBSan
make exhaustive  # state-space exploration, property checking
make coverage    # gcov line and branch coverage
make analyze     # cppcheck static analysis
make firmware    # Cortex-M3 image
make run         # firmware under QEMU (Ctrl-A then X to quit)
make timing      # 5 runs, timing spread
make demo        # all of the above, transcript to logs/demo.log
```

```bash
./tools/gdb_session.sh   # attach GDB to the running target
```

---

## Verification results

Measured on Ubuntu 26.04 under WSL2 — GCC 13.3, arm-none-eabi-gcc 14.2,
QEMU 10.2.

| Check | Result |
|---|---|
| Directed tests | 38 / 38 pass |
| State-space exploration | 74 reachable states, 42,624 transitions |
| Safety properties | 8 / 8 hold |
| State reachability | 7 / 7 reachable, 0 unintended dead ends |
| Line coverage (gcov) | 100.00% of 128 lines |
| Branch coverage | 100.00% executed, 98.82% taken at least once |
| Sanitizers | ASan + UBSan clean |
| Static analysis | cppcheck `--check-level=exhaustive`, clean |
| Firmware image | 19,860 B text, 84 B data, 66 KB bss |
| QEMU run | 581 control steps, 9 faults, 4 recoveries, 1 latch, 2 watchdog expiries |

### The eight properties

| | Property |
|---|---|
| P1 | The output is never enabled outside NORMAL and WARNING |
| P2 | FAULT never transitions directly to an operating state |
| P3 | Recovery always passes through SAFE, then RECOVERING |
| P4 | LATCHED is absorbing without a manual reset |
| P5 | Every declared state is reachable |
| P6 | No reachable state is a dead end other than LATCHED |
| P7 | An active fault cuts the output within one step |
| P8 | The recovery budget can never be exceeded |

Because the explorer visits every reachable configuration, a property that holds
here holds for all of them rather than for those a test happened to reach.

### Transition matrix

```
                   INIT   NORMAL  WARNING    FAULT     SAFE  RECOVER  LATCHED
INIT                 96      120        .     2088        .        .        .
NORMAL                .      102       40     1586        .        .        .
WARNING               .       10       44      522        .        .        .
FAULT                 .        .        .        .     9216        .        .
SAFE                  .        .        .        .     6144     2304      768
RECOVERING            .      210        .    15780        .     1290        .
LATCHED            1152        .        .        .        .        .     1152
```

`FAULT -> NORMAL` is structurally impossible; the empty cell is the evidence.

### Live fault escalation

The fault schedule is seeded and replays identically, which is what makes a
failure bisectable.

```
[     1 ms] INIT       -> NORMAL      out=ON  faults=0x00
[  3450 ms] NORMAL     -> FAULT       out=OFF faults=0x01   sensor disconnected
[  4500 ms] SAFE       -> RECOVERING  out=OFF try=1
[  4700 ms] RECOVERING -> NORMAL      out=ON               recovered
[  7000 ms] NORMAL     -> FAULT       out=OFF faults=0x06   impossible value
[ 10000 ms] NORMAL     -> FAULT       out=OFF faults=0x06   rate violation
[ 14900 ms] NORMAL     -> FAULT       out=OFF faults=0x08   comms lost
[ 17200 ms] RECOVERING -> NORMAL      out=ON               recovered on try 2
[ 19000 ms] NORMAL     -> FAULT       out=OFF faults=0x10   queue overflow
[ 23400 ms] SAFE       -> LATCHED     out=OFF              budget exhausted
[ 23900 ms] WATCHDOG EXPIRED after 1600 ms without a kick
```

### Measured timing

From `make timing`, five runs. A microsecond clock is built from SysTick's
reload value plus the tick count.

```
  metric                         min       max    spread
  control steps                  581       581      same
  fault events                     9         9      same
  recoveries                       4         4      same
  watchdog expiries                2         2      same
  fault-to-output-off          88 us    214 us    126 us
  watchdog detection         1600 ms   1600 ms      same
  ctrl_step avg                  3 us      3 us      same
```

Every logic outcome is identical across runs because the fault schedule is
seeded and the control logic is deterministic. Only the microsecond figures
move, and they move with host scheduling under emulation rather than with
anything in the firmware — which is why `fault-to-output-off` is reported as a
range rather than a single worst case.

Watchdog detection at 1600 ms against a 1500 ms deadline is one health-monitor
period (100 ms) of granularity.

---

## Design notes

**The health monitor runs at priority 5, above the processing task at 4.** At a
lower priority, a spinning processing task would starve the mechanism intended
to detect it, and the watchdog would be kicked by a system already dead. The
`INJ_TASK_STALL` injection exercises this, and the watchdog fires.

**The watchdog kick is conditional** on every task having checked in. An
unconditional kick would prove only that the monitor itself runs.

**FAULT is a single-step state.** Detect, cut the output, move to SAFE.
Remaining in FAULT would prevent the cooldown from starting.

**LATCHED is absorbing by design.** Repeated automatic recovery on a failed
plant risks equipment damage, so a human must intervene.

**Elapsed time uses unsigned subtraction.** The 32-bit millisecond tick wraps
after 49.7 days; a `now > then` comparison fails exactly once, then, and is
unreproducible in the field. There is a directed test for it.

**A rejected reading is never stored**, since it would poison the next rate
check and mask the fault.

**The HardFault handler cuts the output before halting**, writing the pin
directly rather than through any layer that may itself have faulted.

---

## Limitations and known gaps

**No physical hardware.** The firmware is real ARMv7-M running on QEMU's
MPS2-AN385 model, not on silicon.

- **The watchdog is a FreeRTOS one-shot software timer**, not a hardware IWDG.
  The machine model provides no independent watchdog. The semantics are
  identical — expiry forces safe state — but the failure modes of real IWDG
  hardware are not covered.
- **Timing figures are QEMU cycle counts.** They are correct relative to one
  another and the control-flow latencies are real, but absolute microseconds on
  silicon will differ.
- Not covered: real interrupt latency, ADC noise, electrical sensor faults, EMI.

**One branch is not covered.** `enter()`'s same-state guard prevents re-entry
from resetting a state's entry timestamp and good-streak. No current call site
passes the state it is already in, so the branch is defensive against future
edits. This is why branch coverage reads 98.82% rather than 100%.

**cppcheck reports one false positive.** `ctrl_output_allowed` is flagged as
having no external use; it is called from `app_main.c`, which cppcheck does not
analyse in the same pass. Making it static would break the firmware build.

**CI runs on every push.** The workflow in `.github/workflows/ci.yml` runs the
full suite on a clean Ubuntu runner: directed tests under sanitizers, the
exhaustive state-space exploration with all eight properties, a coverage gate
that fails the build below 100% lines, static analysis, the Cortex-M3 build,
the QEMU fault escalation, and a GDB session against the target. It passes on a
runner that has never seen the development machine, so the results above are
independently reproducible rather than self-reported.
