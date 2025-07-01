TESTNAME	:= x2000_lcd_test

export CROSS_COMPILE := $(PWD)/../../sdkk/prebuilts/toolchains/mips-gcc720-glibc229/bin/mips-linux-gnu-

export CC:=$(CROSS_COMPILE)gcc

CFLAGS := -Wall -O2 -lpthread

TARGET = $(PWD)/x2000_lcd_test

SOURCES = $(wildcard *.c)
HEADERS = $(wildcard *.h)

DBGOBJFILES = $(SOURCES:%.c=debug/%.o)
RELOBJFILES = $(SOURCES:%.c=%.o)

.PHONY:clean debug release install

$(TARGET): $(RELOBJFILES)
	$(LINK.c) -o $@ $^

$(RELOBJFILES): %.o: %.c
	$(COMPILE.c) -o $@ $^
install:$(TARGET)
	install -d $(EXEC_DIR)
	install $(TARGET) $(EXEC_DIR)
clean:
	$(RM) *.o $(TARGET)