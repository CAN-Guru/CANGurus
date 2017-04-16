D:
CD D:\DE038\OneDrive - OOO\01 Eisenbahn\00 Decoder develop
000Avrdude\avrdude.exe -c usbtiny -B 1 -patmega328p -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0x05:m
000Avrdude\avrdude.exe -c usbtiny -B 1 -patmega328p -e -U flash:w:00_BTLDR.hex:a
