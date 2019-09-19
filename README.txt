How to install Xling-firmware
-----------------------------

 This file contains an instruction about compiling and uploading
 firmware to the Xling board.

Prerequisites
-------------

 Firmware for the Xling is written in C99 using avr-gcc compiler. CMake build
 system is required to generate build files on the various operating systems.

 If you want to compile firmware yourself, be sure that these programs are in
 your PATH and reachable then:

	- avr-gcc
	- avr-size
        - avr-objcopy
        - avr-objdump
        - avrdude
        - srec_cat

Compile and upload the firmware
-------------------------------

 The default build configuration assumes that there is an STK500v2 programmer
 (or compatible AVR programmer) is connected and reachable via /dev/ttyACM0
 in your operating system.

 If that's not the case, please, take a look at the CMakeLists.txt and
 parameters of its "upload" target.

 Perform these steps in a terminal:

    $ git clone https://github.com/mcusim/Xling-firmware.git
    $ cd Xling-firmware
    $ mkdir build && cd build
    $ cmake .. && make
    $ sudo make upload

 An expected output may look like:

        dsl@ds-laptop:~/git/Xling-firmware/build$ cmake ..
        -- The C compiler identification is GNU 8.3.0
        -- Check for working C compiler: /usr/bin/cc
        -- Check for working C compiler: /usr/bin/cc -- works
        -- Detecting C compiler ABI info
        -- Detecting C compiler ABI info - done
        -- Detecting C compile features
        -- Detecting C compile features - done
        -- Building RELEASE version of Xling firmware
        -- Linker flags:  -mmcu=atmega328p -Wl,-Map=/home/dsl/git/Xling-firmware/build/xling-0.3-firmware.map,--cref,--section-start=.text=0 -s
        -- Configuring done
        -- Generating done
        -- Build files have been written to: /home/dsl/git/Xling-firmware/build
        dsl@ds-laptop:~/git/Xling-firmware/build$ make
        Scanning dependencies of target xling-0.3-firmware.elf
        [ 20%] Building C object CMakeFiles/xling-0.3-firmware.elf.dir/src/fuse.c.o
        [ 40%] Building C object CMakeFiles/xling-0.3-firmware.elf.dir/src/main.c.o
        ...
        [ 80%] Building C object CMakeFiles/xling-0.3-firmware.elf.dir/src/mcusim/avr-gcc/avr/drivers/display/sh1106_graphics.c.o
        [100%] Linking C executable xling-0.3-firmware.elf
        AVR Memory Usage
        ----------------
        Device: atmega328p

        Program:    8908 bytes (27.2% Full)
        (.text + .data + .bootloader)

        Data:        189 bytes (9.2% Full)
        (.data + .bss + .noinit)


        [100%] Built target xling-0.3-firmware.elf
        dsl@ds-laptop:~/git/Xling-firmware/build$ sudo make upload
        [sudo] password for dsl:
        Scanning dependencies of target upload

        avrdude: AVR device initialized and ready to accept instructions

        Reading | ################################################## | 100% 0.00s

        avrdude: Device signature = 0x1e950f (probably m328p)
        avrdude: NOTE: "flash" memory has been specified, an erase cycle will be performed
                 To disable this feature, specify the -D option.
        avrdude: erasing chip
        avrdude: reading input file "/home/dsl/git/Xling-firmware/build/xling-0.3-firmware.hex"
        avrdude: input file /home/dsl/git/Xling-firmware/build/xling-0.3-firmware.hex auto detected as Intel Hex
        avrdude: writing flash (8908 bytes):

        Writing | ################################################## | 100% 3.26s

        avrdude: 8908 bytes of flash written
        avrdude: verifying flash memory against /home/dsl/git/Xling-firmware/build/xling-0.3-firmware.hex:
        avrdude: load data flash data from input file /home/dsl/git/Xling-firmware/build/xling-0.3-firmware.hex:
        avrdude: input file /home/dsl/git/Xling-firmware/build/xling-0.3-firmware.hex auto detected as Intel Hex
        avrdude: input file /home/dsl/git/Xling-firmware/build/xling-0.3-firmware.hex contains 8908 bytes
        avrdude: reading on-chip flash data:

        Reading | ################################################## | 100% 2.92s

        avrdude: verifying ...
        avrdude: 8908 bytes of flash verified

        avrdude: safemode: Fuses OK (E:FF, H:D9, L:EF)

        avrdude done.  Thank you.

        Built target upload
