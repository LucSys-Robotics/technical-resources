# Four-wheel steering kinematics differential-test harness
#
# Directly compile the real source files in the firmware without copying — ensuring the code tested is the one running on the car.
#
#   make compile
#   make run: compile and run the default 200,000 cases
#   make quick: run 10,000 cases (quick self-test after code changes)
#   make full: run 2,000,000 cases (full differential comparison before committing)
#   make clean
#
# Switch compiler: make CC=clang (command line assignment has the highest priority)

# Note to use = instead of ?=: GNU Make has CC=cc built-in, so ?= will be considered defined and not overridden,
# on MinGW, it will look for non-existent cc.exe.
CC      = gcc
BSP     := ../../LUCSYS_BSP

# Windows products have .exe, cannot add the ./ prefix at runtime, and removing it also does not have rm available.
# (The executable file for make on Windows is usually called mingw32-make, not make)
ifeq ($(OS),Windows_NT)
  EXE   := .exe
  RUN   :=
  RMCMD := if exist 4ws_sim.exe del /Q 4ws_sim.exe
else
  EXE   :=
  RUN   := ./
  RMCMD := rm -f 4ws_sim
endif
TARGET  := 4ws_sim$(EXE)

# The source files are GBK encoded (consistent with firmware engineering).
# Do not add -finput-charset: MinGW 8.1 lacks iconv support and would report "failure to convert GBK to UTF-8".
# gcc reads by default as UTF-8, and illegal byte sequences in GBK Chinese comments are directly released without affecting compilation.
# (Scan confirmed that there is no continuation trap in the project where the GBK character end byte is 0x5C and is at the end of // line)
CFLAGS  := -std=c99 -O2 -Wall -Wextra -I$(BSP)/Inc
LDFLAGS := -lm

SRCS := main.c $(BSP)/4ws_v2.c

.PHONY: all run quick full clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDFLAGS)

run: $(TARGET)
	$(RUN)$(TARGET)

quick: $(TARGET)
	$(RUN)$(TARGET) 10000

full: $(TARGET)
	$(RUN)$(TARGET) 2000000

clean:
	-$(RMCMD)
