TARGET = main

# Default target chip.
MCU ?= STM32F103x8
#MCU ?= STM32F103xB
#MCU ?= GD32VF103xB

# Define target chip information.
ifeq ($(MCU), STM32F103x8)
	MCU_FILES = STM32F103x8
	ST_MCU_DEF = STM32F103xB
	MCU_TYPE = STM32
else ifeq ($(MCU), STM32F103xB)
	MCU_FILES = STM32F103xB
	ST_MCU_DEF = STM32F103xB
	MCU_TYPE = STM32
else ifeq ($(MCU), GD32VF103xB)
	MCU_FILES = GD32VF103xB
	MCU_TYPE = GD32V
endif

ifeq ($(MCU_TYPE), STM32)
	# Assembly and C flags.
	ASFLAGS += -mcpu=cortex-m3
	ASFLAGS += -mthumb
	CFLAGS  += -mcpu=cortex-m3
	CFLAGS  += -mthumb
	CFLAGS  += -msoft-float
	CFLAGS  += -mfloat-abi=soft
	CFLAGS += -D$(ST_MCU_DEF)
	LFLAGS += -mcpu=cortex-m3
	LFLAGS += -mthumb
	LFLAGS += -msoft-float
	LFLAGS += -mfloat-abi=soft
	# Toolchain definitions (ARM Cortex-M bare metal defaults)
	CC = arm-none-eabi-gcc
	OC = arm-none-eabi-objcopy
	OS = arm-none-eabi-size
else ifeq ($(MCU_TYPE), GD32V)
	# Assembly, C, and linker flags.
	ASFLAGS += -march=rv32imac
	ASFLAGS += -mabi=ilp32
	ASFLAGS += -mcmodel=medlow
	CFLAGS += -march=rv32imac
	CFLAGS += -mabi=ilp32
	CFLAGS += -mcmodel=medlow
	LFLAGS += -Wl,--no-relax
	LFLAGS += -march=rv32imac
	LFLAGS += -mabi=ilp32
	LFLAGS += -mcmodel=medlow
	# Toolchain definitions (RISC-V bare metal defaults)
	CC = riscv32-unknown-elf-gcc
	OC = riscv32-unknown-elf-objcopy
	OS = riscv32-unknown-elf-size
endif

# Assembly directives.
ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -Wall
# (Set error messages to appear on a single line.)
ASFLAGS += -fmessage-length=0
ASFLAGS += -DVVC_$(MCU_TYPE)

# C compilation directives
CFLAGS += -Wall
CFLAGS += -g
CFLAGS += -fmessage-length=0
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += --specs=nosys.specs
CFLAGS += -D$(MCU_FILES)
CFLAGS += -DVVC_$(MCU_TYPE)

# Linker directives.
LSCRIPT = ./ld/$(MCU_FILES).ld
LFLAGS += -Wall
LFLAGS += --specs=nosys.specs
LFLAGS += -lgcc
LFLAGS += -Wl,--gc-sections
LFLAGS += -Wl,-L./ld
LFLAGS += -T$(LSCRIPT)

AS_SRC   =  ./src/$(MCU_FILES).S
C_SRC    =  ./src/main.c
C_SRC    += ./src/patterns.c
ifeq ($(MCU_TYPE), GD32V)
	C_SRC += ./device_headers/n200_func.c
endif

INCLUDE  =  -I./
INCLUDE  += -I./device_headers

OBJS  = $(AS_SRC:.S=.o)
OBJS += $(C_SRC:.c=.o)

.PHONY: all
all: $(TARGET).bin

%.o: %.S
	$(CC) -x assembler-with-cpp $(ASFLAGS) $(INCLUDE) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

.PHONY: clean
clean:
	rm -f $(OBJS)
	rm -f $(ST_MCU_DEF)_vt.S
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
