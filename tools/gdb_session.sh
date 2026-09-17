#!/usr/bin/env bash
# Attach GDB to the firmware running under QEMU and capture a real debug
# session: break on the fault transition, inspect controller state, read the
# fault flags, walk the stack.
#
# QEMU starts halted (-S) with a gdbserver on :3333, so the debugger is
# attached before the reset handler runs — the same setup as an ST-Link with
# OpenOCD, minus the wire.
set -u
cd "$(dirname "$0")/.."
cleanup() { kill ${QEMU:-} 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT
mkdir -p logs

qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -m 16M -nographic \
    -serial null -kernel build/firmware.elf -S -gdb tcp::3333 \
    > /dev/null 2>&1 &
QEMU=$!
sleep 2

timeout 60 gdb-multiarch -q -batch \
  -ex "set confirm off" \
  -ex "set pagination off" \
  -ex "target remote :3333" \
  -ex "break ctrl_step if in->queue_overflowed == 1" \
  -ex "continue" \
  -ex "echo \n=== stopped on an injected queue-overflow fault ===\n" \
  -ex "bt" \
  -ex "echo \n--- controller state on entry ---\n" \
  -ex "print c->state" \
  -ex "print/x c->active_faults" \
  -ex "print c->recovery_attempt" \
  -ex "echo \n--- the inputs that caused it ---\n" \
  -ex "print in->queue_overflowed" \
  -ex "print in->sensor_fresh" \
  -ex "print in->now_ms" \
  -ex "echo \n--- run the transition and re-read the state ---\n" \
  -ex "delete 1" \
  -ex "finish" \
  -ex "print g_out.state" \
  -ex "print g_out.output_enabled" \
  -ex "echo \n--- CPU registers ---\n" \
  -ex "info registers sp lr pc" \
  -ex "echo \n--- currently scheduled FreeRTOS task ---\n" \
  -ex "print pxCurrentTCB->pcTaskName" \
  -ex "detach" \
  build/firmware.elf 2>&1 | tee logs/gdb_session.log
