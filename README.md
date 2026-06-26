# GPS receiver app for Tanmatsu

This app reads NMEA sentences from an ATGM336H GPS module connected to the Tanmatsu's QWIIC connector, parses them using [minmea](https://github.com/kosma/minmea), and displays the results on the 800×480 screen using the [PAX graphics](https://github.com/robotman2412/pax-graphics/tree/release/1.1.1/docs) library.

The left side of the screen shows fix status, UTC time and date, latitude, longitude, altitude, satellite count, speed, course, fix type, and HDOP. The right side shows a live sky plot with each visible satellite positioned by azimuth and elevation, colour-coded by signal strength.

## Hardware connection

Connect the GPS module to the QWIIC connector on the Tanmatsu. The module's serial TX and RX lines are wired to the QWIIC SCL and SDA pins, which map to GPIO 32 and GPIO 33 on the ESP32-P4:

| ATGM336H pin | QWIIC pin | GPIO  | Direction         |
|--------------|-----------|-------|-------------------|
| RX           | SCL       | 32    | Tanmatsu → GPS    |
| TX           | SDA       | 33    | GPS → Tanmatsu    |
| VCC          | 3V3       | —     | Power             |
| GND          | GND       | —     | Ground            |

The module communicates at 9600 baud, 8N1. No flow control is required.

## License

The contents of this repository, excluding the minmea library, may be considered in the public domain or [CC0-1.0](https://creativecommons.org/publicdomain/zero/1.0) licensed at your disposal.

The minmea library may be used under terms of the `WTFPL`, `MIT` or `LGPL` license as described in the information files inside minmea folder.

At Nicolai Electronics we love open source so we recommend licensing your work based on this template under terms of the [MIT license](https://opensource.org/license/mit). The MIT license allows others to build upon your work without restrictions while also making sure you retain your attribution.
