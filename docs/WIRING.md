# Wiring

This is the reference classic ESP32 pin map, not a verification of an assembled unit.

| function | GPIO |
| --- | --- |
| pan STEP / DIR | 25 / 26 |
| tilt STEP / DIR | 27 / 14 |
| shared DRV8825 nENBL | 13 |
| pan min / max NC loop | 16 / 17 |
| tilt min / max NC loop | 18 / 19 |
| E-stop NC sense loop | 21 |
| TRACK switch | 22 |
| STOP button | 23 |
| HOME button | 32 |
| joystick pan / tilt ADC1 | 34 / 35 |

GPIO33 is no longer configured or used. Physically disconnect the old trigger electronics. No servo, valve, pump or firing output is part of this design.

NC inputs close to ground when healthy; internal pull-ups read an opened loop as active/broken. TRACK/HOME/STOP close to ground when selected/pressed. Keep all ESP32 signals at 3.3 V. GPIO34/35 are analog inputs without internal pull-ups.

The ESP32 uses USB serial to the PC. The camera uses a separate PC USB port. Wire driver logic grounds to the ESP32 ground. Use an appropriately protected motor supply, genuine driver current settings and local VMOT capacitors. Do not route motor power through a breadboard. Avoid connecting competing USB/external 5 V sources.

The E-stop's physical contactor circuit should remove motor power independently of firmware while the other NC contact supplies the sense input. Commissioning must verify both contacts and their DC ratings. Software E-stop handling alone is not an independent safety circuit.
