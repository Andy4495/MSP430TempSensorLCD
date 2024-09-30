# MSP430 Temp Sensor with Built-in LCD Display

[![Arduino Compile Sketches](https://github.com/Andy4495/MSP430TempSensorLCD/actions/workflows/arduino-compile-sketches.yml/badge.svg)](https://github.com/Andy4495/MSP430TempSensorLCD/actions/workflows/arduino-compile-sketches.yml)
[![Check Markdown Links](https://github.com/Andy4495/MSP430TempSensorLCD/actions/workflows/check-links.yml/badge.svg)](https://github.com/Andy4495/MSP430TempSensorLCD/actions/workflows/check-links.yml)

This is another iteration of my Temperature Sensor sketches (see [References](#references) below). This sketch takes advantage of the built-in LCD on the [FR4133][1] and [FR6989][2] LaunchPads, but will also work with any of the other MSP430 LaunchPads.

It is specifically written to use the [Fuel Tank I][3] BoosterPack as a power source, but will also work without it by commenting out the line:

```cpp
#define FUEL_TANK_ENABLED
```

## External Libraries

- [SWI2C][4]
- [MspTandV][5]

## References

- [MSP430 Low Power Temp Sensor][6]
- [MSP430 Temp Sensor with Sharp Display][7]
- CC110L [BoosterPack][8]

## License

The software and other files in this repository are released under what is commonly called the [MIT License][100]. See the file [`LICENSE.txt`][101] in this repository.

[1]: https://www.ti.com/tool/MSP-EXP430FR4133
[2]: https://www.ti.com/tool/MSP-EXP430FR6989
[3]: https://www.ti.com/lit/ug/slvua32/slvua32.pdf
[4]: https://github.com/Andy4495/SWI2C
[5]: https://github.com/Andy4495/MspTandV
[6]: https://github.com/Andy4495/MSP430LowPowerTempSensor
[7]: https://github.com/Andy4495/MSP430TempSensorWithDisplay
[8]: http://www.ti.com/lit/ml/swru312b/swru312b.pdf
[100]: https://choosealicense.com/licenses/mit/
[101]: ./LICENSE.txt
[//]: # ([200]: https://github.com/Andy4495/MSP430TempSensorLCD)

[//]: # ( Former reference [3]: https://www.ti.com/tool/BOOSTXL-BATTPACK )
[//]: # (This is a way to hack a comment in Markdown. This will not be displayed when rendered.)
