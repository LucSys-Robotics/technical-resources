# 四轮四转运动学对拍工程
#
# 直接编译固件里的真实源文件,不做拷贝 —— 保证测的就是车上跑的那份代码.
#
#   make          编译
#   make run      编译并跑默认 20 万组
#   make quick    跑 1 万组(改代码后快速自检)
#   make full     跑 200 万组(提交前完整对拍)
#   make clean
#
# 换编译器: make CC=clang   (命令行赋值优先级最高)

# 注意用 = 而不是 ?= : GNU Make 内置了 CC=cc, ?= 会认为它已定义而不覆盖,
# 在 MinGW 上就会去找不存在的 cc.exe.
CC      = gcc
BSP     := ../../LUCSYS_BSP

# Windows 下产物带 .exe, 运行时不能加 ./ 前缀, 删除也没有 rm 可用.
# (Windows 上 make 的可执行文件通常叫 mingw32-make, 不是 make)
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

# 源文件是 GBK 编码(与固件工程一致).
# 不加 -finput-charset:MinGW 8.1 缺 iconv 支持,加了会报 "failure to convert GBK to UTF-8".
# gcc 默认按 UTF-8 读,GBK 中文注释里的非法字节序列它会直接放行,不影响编译.
# (已扫描确认工程内不存在"GBK字符尾字节为0x5C且位于//行尾"的续行陷阱)
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
