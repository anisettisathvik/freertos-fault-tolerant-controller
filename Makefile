# ---------------------------------------------------------------------------
# Two build targets from one source tree:
#   host     - the control logic, verified exhaustively with gcc + sanitizers
#   firmware - the same logic inside FreeRTOS on Cortex-M3, via arm-none-eabi
# ---------------------------------------------------------------------------
CC        := gcc
ARMCC     := arm-none-eabi-gcc
OBJCOPY   := arm-none-eabi-objcopy
SIZE      := arm-none-eabi-size
QEMU      := qemu-system-arm

RTOS      := freertos
PORTDIR   := $(RTOS)/portable/GCC/ARM_CM3
MEMMANG   := $(RTOS)/portable/MemMang/heap_4.c

HOSTFLAGS := -std=c99 -Wall -Wextra -Wpedantic -O1 -g -Isrc
SAN       := -fsanitize=address,undefined -fno-omit-frame-pointer

ARMFLAGS  := -mcpu=cortex-m3 -mthumb -std=gnu99 -O2 -g \
             -Wall -Wextra -ffunction-sections -fdata-sections \
             -Isrc -Iport -I$(RTOS)/include -I$(PORTDIR)
LDFLAGS   := -T linker/mps2_an385.ld -nostartfiles \
             -Wl,--gc-sections -Wl,-Map=build/firmware.map \
             --specs=nano.specs --specs=nosys.specs -lc -lnosys

RTOS_SRC  := $(RTOS)/tasks.c $(RTOS)/queue.c $(RTOS)/list.c \
             $(RTOS)/timers.c $(RTOS)/event_groups.c $(PORTDIR)/port.c $(MEMMANG)
FW_SRC    := src/app_main.c src/control_logic.c port/startup_mps2.c port/uart.c

.PHONY: all verify exhaustive logic firmware run coverage analyze demo clean help

help:
	@echo "make verify     - host verification (exhaustive + directed)"
	@echo "make firmware   - build the Cortex-M3 image"
	@echo "make run        - run the firmware under QEMU"
	@echo "make coverage  - line and branch coverage of the control logic"
	@echo "make analyze   - cppcheck static analysis"
	@echo "make demo      - full transcript to logs/demo.log"
	@echo "make timing    - firmware timing spread across 5 runs"
	@echo "make clean"

all: verify firmware

# ---- host verification ---------------------------------------------------
build/exhaustive: src/control_logic.c test/exhaustive.c
	@mkdir -p build
	$(CC) $(HOSTFLAGS) $(SAN) $^ -o $@

build/test_logic: src/control_logic.c test/test_logic.c
	@mkdir -p build
	$(CC) $(HOSTFLAGS) $(SAN) $^ -o $@

exhaustive: build/exhaustive
	./build/exhaustive

logic: build/test_logic
	./build/test_logic

verify: logic exhaustive

# ---- firmware -------------------------------------------------------------
build/firmware.elf: $(FW_SRC) $(RTOS_SRC) linker/mps2_an385.ld
	@mkdir -p build
	$(ARMCC) $(ARMFLAGS) $(FW_SRC) $(RTOS_SRC) $(LDFLAGS) -o $@
	$(SIZE) $@

firmware: build/firmware.elf

run: build/firmware.elf
	$(QEMU) -machine mps2-an385 -cpu cortex-m3 -m 16M \
	        -nographic -serial mon:stdio -kernel $< -semihosting-config enable=on

clean:
	rm -rf build

# ---- structural coverage --------------------------------------------------
# The control logic is the safety-critical part, so it gets the same treatment
# as the Modbus core: line and branch coverage, not just "the tests pass".
# control_logic.c is compiled ONCE and that single object is linked into both
# test binaries, so both runs accumulate into one .gcda and gcov reports their
# union. Compiling it separately per binary gives two disjoint reports and
# understates coverage — which is how this target read 89% while the tests
# between them actually covered everything.
coverage:
	@mkdir -p build/cov
	@rm -f build/cov/*.gcda build/cov/*.gcno
	$(CC) -std=c99 -O0 -g --coverage -Isrc -c src/control_logic.c \
	      -o build/cov/control_logic.o
	$(CC) -std=c99 -O0 -g --coverage -Isrc \
	      build/cov/control_logic.o test/test_logic.c -o build/cov/cov_logic
	$(CC) -std=c99 -O0 -g --coverage -Isrc \
	      build/cov/control_logic.o test/exhaustive.c -o build/cov/cov_exh
	cd build/cov && ./cov_logic > /dev/null && ./cov_exh > /dev/null
	@echo ""
	@echo "coverage of src/control_logic.c (directed + exhaustive):"
	@gcov -b -s . build/cov/control_logic.gcda 2>/dev/null | \
	   grep -E "^File|^Lines|^Branches|^Taken" || true

# ---- static analysis ------------------------------------------------------
analyze:
	cppcheck --enable=all --inconclusive --check-level=exhaustive --std=c99 \
	         --error-exitcode=1 --suppress=missingIncludeSystem \
	         --suppress=unusedFunction --suppress=checkersReport \
	         -Isrc src/control_logic.c 2>&1 | tail -15
	@echo "cppcheck: clean"

# ---- one-command demo -----------------------------------------------------
# Replaces a screen recording: a committed transcript anyone can read without
# running anything, and that CI regenerates on every push.
demo:
	@mkdir -p logs
	@{ \
	  echo "########################################################"; \
	  echo "#  Fault-Tolerant Industrial Controller - full run      #"; \
	  echo "########################################################"; \
	  echo ""; echo "### 1. directed logic tests ###"; $(MAKE) -s logic; \
	  echo ""; echo "### 2. exhaustive state space exploration ###"; $(MAKE) -s exhaustive; \
	  echo ""; echo "### 3. structural coverage ###"; $(MAKE) -s coverage; \
	  echo ""; echo "### 4. static analysis ###"; $(MAKE) -s analyze; \
	  echo ""; echo "### 5. Cortex-M3 firmware build ###"; $(MAKE) -s firmware; \
	  echo ""; echo "### 6. live fault escalation under QEMU ###"; \
	  timeout 45 qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -m 16M \
	    -nographic -serial file:logs/qemu_demo.log -monitor none \
	    -kernel build/firmware.elf < /dev/null > /dev/null 2>&1 || true; \
	  tr -d '\r' < logs/qemu_demo.log; \
	  echo ""; echo "### 7. timing spread across 5 runs ###"; ./tools/timing_runs.sh 5; \
	  echo ""; echo "### 8. GDB session on target ###"; ./tools/gdb_session.sh; \
	} 2>&1 | tee logs/demo.log
	@echo ""
	@echo "transcript written to logs/demo.log"

timing: build/firmware.elf
	./tools/timing_runs.sh 5

.PHONY: timing
