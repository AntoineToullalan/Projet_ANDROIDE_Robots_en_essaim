1) install an updated avr toolchain in the system:
   sudo apt-get install avrdude binutils-avr gcc-avr avr-libc gdb-avr

2) substitute the Arduino IDE toolchain with the one just installed in the system:
   - copy /usr/avr-* to  arduino/hardware/tools/avr/bin
   - copy /usr/lib/avr to arduino/hardware/tools/avr/lib
   - copy /usr/lib/gcc/avr to arduino/hardware/tools/avr/lib/gcc

3) copy the Elisa3 library in the /arduino/libraries folder

4) make

5) make upload
