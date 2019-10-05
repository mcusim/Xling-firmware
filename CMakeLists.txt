#
# This file is part of MCUSim, an XSPICE library with microcontrollers.
#
# Copyright (C) 2017-2019 MCUSim Developers, see AUTHORS.txt for contributors.
#
# MCUSim is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# MCUSim is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#
# A build configuration file of the Xling firmware.
#
# NOTE: Verbose output of the build process can be activated
# using 'make VERBOSE=1'.
#
cmake_minimum_required(VERSION 3.2)
project(Xling-firmware C)

# ------------------------------------------------------------------------------
# MCU type and clock frequency
# ------------------------------------------------------------------------------
set(AVR_MCU "atmega328p")
set(AVR_FREQ 12000000UL)

set(TARGET_OUTPUT_BASENAME "xling-0.3.1-firmware")
set(TARGET_OUTPUT_FILE "${TARGET_OUTPUT_BASENAME}.elf")
set(TARGET_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
#
# Remove '-rdynamic', '-Wl,-search_paths_first' (avr-gcc doesn't support this
# one and treats it as '-Wl,-s' which strips linked ELF
#
set(CMAKE_C_LINK_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

# ------------------------------------------------------------------------------
# Set flags
# ------------------------------------------------------------------------------
if (CMAKE_BUILD_TYPE MATCHES Debug)
	message(STATUS "Building DEBUG version of Xling firmware")
	add_definitions("-g -DDEBUG -mmcu=${AVR_MCU} -DF_CPU=${AVR_FREQ}")
else()
	message(STATUS "Building RELEASE version of Xling firmware")
	add_definitions("-Os -mmcu=${AVR_MCU} -DF_CPU=${AVR_FREQ}")
endif()

# Minimal flags
add_definitions("-Wall")
add_definitions("-pedantic")
add_definitions("-std=iso9899:1999")
# Recommended flags
add_definitions("-Wshadow")
add_definitions("-Wpointer-arith")
add_definitions("-Wcast-qual")
add_definitions("-Wcast-align")
add_definitions("-Wstrict-prototypes")
add_definitions("-Wmissing-prototypes")
add_definitions("-Wconversion")
add_definitions("-Wsign-compare")
add_definitions("-Wmissing-declarations")
add_definitions("-Wnested-externs")
add_definitions("-Wbad-function-cast")
add_definitions("-Wold-style-definition")
add_definitions("-Wunused")
add_definitions("-Wuninitialized")
add_definitions("-Wmissing-noreturn")
add_definitions("-Wmissing-format-attribute")
add_definitions("-Wredundant-decls")
add_definitions("-Werror=implicit")
add_definitions("-Werror=nonnull")
add_definitions("-Werror=init-self")
add_definitions("-Werror=main")
add_definitions("-Werror=missing-braces")
add_definitions("-Werror=sequence-point")
add_definitions("-Werror=return-type")
add_definitions("-Werror=trigraphs")
add_definitions("-Werror=array-bounds")
add_definitions("-Werror=write-strings")
add_definitions("-Werror=address")

# Set linker flags
if (CMAKE_BUILD_TYPE MATCHES Debug)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mmcu=${AVR_MCU}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.map,--cref,--section-start=.text=0")
else()
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mmcu=${AVR_MCU}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.map,--cref,--section-start=.text=0")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s")
endif()
message(STATUS "Linker flags: ${CMAKE_EXE_LINKER_FLAGS}")

# ------------------------------------------------------------------------------
# MCUSim drivers configuration
# ------------------------------------------------------------------------------
add_definitions("-DconfigMSIM_DRV_DISPLAY_SH1106_TWI")
add_definitions("-DconfigMSIM_DRV_DISPLAY_SH1106_DNUM=1")
add_definitions("-DconfigMSIM_DRV_DISPLAY_SH1106_BUFSZ=144")

# ------------------------------------------------------------------------------
# Find executables
# ------------------------------------------------------------------------------
find_program(AVR_CC avr-gcc)
find_program(AVR_SIZE_TOOL avr-size)
find_program(AVR_OBJCOPY avr-objcopy)
find_program(AVR_OBJDUMP avr-objdump)
find_program(AVR_DUDE avrdude)
find_program(SREC_CAT srec_cat)

# ------------------------------------------------------------------------------
# Define mandatory variables
# ------------------------------------------------------------------------------
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)
set(CMAKE_C_COMPILER ${AVR_CC})
set(CMAKE_CXX_COMPILER ${AVR_CXX})

# ------------------------------------------------------------------------------
# Define includes
# ------------------------------------------------------------------------------
include_directories("include/")

# ------------------------------------------------------------------------------
# Set sources here
# ------------------------------------------------------------------------------
set(XLING_SRC
	src/fuse.c
	src/main.c
	src/mcusim/avr-gcc/avr/drivers/display/sh1106.c
	src/mcusim/avr-gcc/avr/drivers/display/sh1106_graphics.c
)

add_executable(${TARGET_OUTPUT_FILE} ${XLING_SRC})
add_custom_target("upload")

# ------------------------------------------------------------------------------
# Prepare files for MCU
# ------------------------------------------------------------------------------
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_SIZE_TOOL} -C --mcu=${AVR_MCU} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_FILE})
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_OBJDUMP} -h -S ${TARGET_OUTPUT_FILE} > ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.lss)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_OBJCOPY} -R .eeprom -R .fuse -R .lock -R .signature -O ihex ${TARGET_OUTPUT_FILE} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.hex)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_OBJCOPY} --no-change-warnings -j .fuse --change-section-lma .fuse=0 -O ihex ${TARGET_OUTPUT_FILE} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.fuse)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_OBJCOPY} --no-change-warnings -j .eeprom --change-section-lma .eeprom=0 -O ihex ${TARGET_OUTPUT_FILE} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.eep)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_OBJCOPY} --no-change-warnings -j .lock --change-section-lma .lock=0 -O ihex ${TARGET_OUTPUT_FILE} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.lock)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_OBJCOPY} --no-change-warnings -j .signature --change-section-lma .signature=0 -O ihex ${TARGET_OUTPUT_FILE} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.sig)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${SREC_CAT} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.fuse -Intel -crop 0x00 0x01 -offset  0x00 -O ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.lfs -Intel)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${SREC_CAT} ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.fuse -Intel -crop 0x01 0x02 -offset -0x01 -O ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.hfs -Intel)
add_custom_command(
	TARGET ${TARGET_OUTPUT_FILE} POST_BUILD
	COMMAND ${AVR_OBJDUMP} -m avr -D ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.hex > ${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.hex.txt)

# ------------------------------------------------------------------------------
# Upload command configurtion
# ------------------------------------------------------------------------------
add_custom_command(
	TARGET "upload" POST_BUILD
	COMMAND ${AVR_DUDE} -p m328p -b 115200 -P /dev/ttyACM0 -c stk500v2 -Uflash:w:${TARGET_OUTPUT_DIR}/${TARGET_OUTPUT_BASENAME}.hex)
