# F9P_RAWX_Logger


## !WORK IN PROGRESS! (Not yet fully tested!)

A guide on how to assemble a GNSS PPK RAWX Logger from an [Adafruit Feather M0 Adalogger](https://www.adafruit.com/product/2796)
and a [SparkFun GPS-RTK2 Board](https://www.sparkfun.com/products/15136) which incorporates the (u-blox ZED-F9P)[https://www.u-blox.com/en/product/zed-f9p-module]
dual band (L1 + L2) GNSS receiver

![Connections](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Connections.JPG)

The RAWX files logged by this project can be processed using [rtklibexplorer's](https://rtklibexplorer.wordpress.com/)
version of [RTKLIB](http://rtkexplorer.com/downloads/rtklib-code/)

[SOFTWARE.md](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/SOFTWARE.md) describes how to install the Arduino IDE and
all the libraries you will need for this project

[HARDWARE.md](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/HARDWARE.md) describes how to connect the Adalogger to the
SparkFun GPS-RTK2 board

[UBX.md](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/UBX.md) describes how the Arduino code communicates with
the F9P using the u-blox UBX binary protocol to enable and log the RAWX messages

The [Arduino](https://github.com/PaulZC/F9P_RAWX_Logger/tree/master/Arduino) directory contains the Arduino code.

## Licence

This project is distributed under a Creative Commons Attribution + Share-alike (BY-SA) licence.
Please refer to section 5 of the licence for the "Disclaimer of Warranties and Limitation of Liability".

Enjoy!

**_Paul_**



