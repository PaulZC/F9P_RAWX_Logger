# HARDWARE

This guide describes how to connect the Adafruit Feather M0 Adalogger to the SparkFun GPS-RTK2 board to create the F9P_RAWX_Logger

## Adafruit Feather M0 Adalogger

The [Adafruit Feather M0 Adalogger](https://www.adafruit.com/product/2796) is a versatile board equipped with a SAMD21G18A ARM Cortex-M0+
microcontroller (as used on the Arduino Zero), a micro-SD card socket and a LiPo battery charger.

If your Adalogger is still connected to your computer, disconnect it before proceeding. Make sure the LiPo battery is disconnected too.

The Adalogger uses USB power to charge the LiPo battery at 100mA. Charging a larger battery can take quite a long time. Adafruit offer
separate miniature [LiPo chargers](https://www.adafruit.com/product/1904) which can charge larger batteries at 500mA.

Choose a good quality micro-SD card. A 4GB card should provide more than enough storage for your RAWX files. Make sure the card is formatted
as FAT32. If you have problems formatting the card, you might need to download and use the official SD card formatter from the
[SD Association](https://www.sdcard.org/downloads/formatter/index.html). Insert the card into the Adalogger before connecting USB
or LiPo battery power. **Don't insert or remove the card while power is connected!**

There are many ways to hook up the Adalogger. The simplest is to use header pins and jumper wires. There are full instructions
available on the [Adafruit website](https://learn.adafruit.com/adafruit-feather-m0-adalogger/assembly).

## SparkFun GPS-RTK2 Board

The [SparkFun GPS-RTK2 Board](https://www.sparkfun.com/products/15136) is equipped with the 
[u-blox ZED-F9P](https://www.u-blox.com/en/product/zed-f9p-module) GNSS module.

The u-blox ZED-F9P is a sophisticated dual band (L1 + L2) GNSS receiver which can act as a Real Time Kinematic base or rover. It has a variety of interfaces:
UART, SPI, I2C and USB. SparkFun have included their Qwiic I2C connectors on the board, making it easy to interface it to their other Qwiic boards.
For the RAWX_Logger we will only be using the UART interface (the Arduino code disables the I2C and USB interfaces - edit the code if you want
these to remain enabled).

This project only logs RAWX messages which can be post-processed (PPK) using [rtklibexplorer's](https://rtklibexplorer.wordpress.com/) version of
[RTKLIB](http://rtkexplorer.com/downloads/rtklib-code/); it does not currently make use of the F9P's RTK features. However, you will find UART2, SURVEY_IN and RTCM
configuration messages defined in the code which will be useful if you do want to try RTK.

Like the Adalogger, there are many ways to hook up the F9P board. Again, the simplest is to use header pins and jumper wires.

## Connections

Connect the following pins:

- Adalogger GND to SparkFun GND
- Adalogger USB (VBUS) to SparkFun 5V
- Adalogger RX (D0) to SparkFun TX/MISO
- Adalogger TX (D1) to SparkFun RX/MOSI
- Adalogger RST to SparkFun RST

![Connections](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Connections.JPG)

The GPS-RTK2 board has its own 3.3V regulator on board. We will use this and power it from the Adalogger VBUS pin. That way it can be powered via the Adalogger's
USB socket or the Adalogger LiPo battery.

These connections will also work if you want to power the Adalogger using the USB-C socket on the SparkFun board.

**Be careful that you do not connect power via the Adalogger USB and SparkFun USB-C sockets at the same time. BAD THINGS WILL HAPPEN IF YOU DO!**

![Power](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Power.JPG)

The RST connection is only necessary if you want the Adalogger reset switch to be able to reset the ZED-F9P too.

Connect the SparkFun board to a suitable **active** L1/L2 GNSS antenna using the uFL socket. You may need to use a [uFL to SMA adapter](https://www.sparkfun.com/products/9145).

## Rover and Base Mode

Connect the Adalogger A0 pin to GND to put the logger into base mode. Leave A0 floating for rover mode. The only differences between base and rover mode are:
- The RAWX log filenames will start with "r_" for the rover and "b_" for the base
- The F9P navigation engine dynamic model is set to "airborne <1g" for the rover and "stationary" for the base
- (The dynamic models can be changed by editing the Arduino code)

![Extras](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Extras.JPG)

## Stopping the Logger

If you are powering the logger from a LiPo battery, the logger monitors the LiPo battery voltage and will automatically close the log file when the battery voltage starts to fall.

You can connect a "stop logging" push switch between the Adalogger A1 pin and GND. This switch is optional but pushing it will safely close the RAWX log file so you can
unplug the power. If you unplug both USB and LiPo power while the logger is still logging, the RAWX log file will not get closed and you will lose your data!

## Waypoint / Timestamp

You can connect a push switch between GND and the SparkFun INT pin. Pushing it will generate a TIM_TM2 message which will get logged with the RAWX data.
[RTKLIB](https://rtklibexplorer.wordpress.com/2018/10/26/event-logging-with-rtklib-and-the-u-blox-m8t-receiver/) can be used to export these.

Instead of a switch, you can connect the INT pin to a 3.3V logic signal from (e.g.) your UAV camera trigger. (The signal must be between 0V and 3.3V.
Higher voltages will cause permanent damage to the ZED-F9P!)

## UAV Power

If you want to power the logger from your UAV battery, use a suitable 5V battery eliminator (DC-DC converter) to drop your UAV battery voltage down to 5V. Connect the 5V to the Adalogger
using the micro-USB socket and connect a small (~100mAh) LiPo battery to the Adalogger too. When you disconnect the UAV battery at the end of a flight, the Adalogger will switch over
to the LiPo battery and will keep logging until the battery voltage starts to fall. The log file will be closed automatically when the battery voltage is low (or if you press the stop
switch).

When you provide power from a UAV battery, the Adalogger will draw an extra 100mA from the battery eliminator to recharge the LiPo battery. This will cut down your flight time. If you do not want the Adalogger
to do this, you can remove the LiPo charger chip by carefully cutting through its legs with a scalpel blade. If you do this, you won't then be able recharge the LiPo battery through the Adalogger.
You will need to use a [separate charger](https://www.adafruit.com/product/1904) instead. Your will of course void the Adalogger's warranty too!

![Adalogger_LiPo](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Adalogger_LiPo.JPG)

You could connect 5V from the battery eliminator to the USB/VBUS pin but you _must_ make sure you do not connect either the Adalogger USB socket or the SparkFun USB-C socket to a
computer or a power supply. **BAD THINGS WILL HAPPEN IF YOU DO!**

## Testing

Leave the LiPo battery disconnected.

Hook up the Adalogger and SparkFun boards as described above.

Connect the Adalogger to your computer using a micro-USB cable. The logger will draw its power from the USB port.

Open the Arduino IDE and open the RAWX_Logger_F9P.ino sketch. Check the Tools\Board and Tools\Port settings. Upload the code to the Adalogger
using the arrow icon below the Edit menu.

As soon as the upload is finished, click on the Tools menu and then "Serial Monitor". Change the baud rate to 115200 using the pull-down menu
at the bottom of the serial monitor window. All being well, after 10 seconds you should see messages saying:

![Serial_Monitor](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Serial_Monitor.JPG)

Now connect the LiPo battery. You can then disconnect the USB cable if you want to and the logger will keep logging, drawing power from the LiPo battery.

The green LED on the Adalogger will light up when a GNSS fix is established and the logger is about to start logging RAWX data. If the LED doesn't illuminate
after ~1 minute, check the antenna has a clear view of the sky. There are DEBUG messages that you can enable to help diagnose problems. The messages will
appear in the serial monitor. Uncomment the line which says:

```
#define DEBUG // Comment this line out to disable debug messages
```

and upload the code again.

The red LED on the Adalogger will flash quickly each time data is written to the SD card. Continuous red indicates: a problem with the SD card; or that the
stop switch has been pressed; or that the LiPo battery is low.

The logger will keep logging until the stop switch is pressed or the battery voltage starts to fall. A new file is opened every 15 minutes to minimise data
loss if the power is accidentially disconnected. The INTERVAL can be changed by editing the Arduino code. If you disconnect the power while the log file is
still open, you will lose your data (the file will appear zero size).

Disconnect the power before removing the SD card. Plug the SD card into your computer using a suitable adapter and look for the RAWX log file(s). You will find them
in a directory called "YYYYMMDD" where YYYY is the UTC year, MM is the month and DD is the date. A rover file will be called "r_HHMMSS.ubx" where HH is the UTC hour,
MM the minute and SS the second when the file was _created_. Base files start with "b_".

You can concatenate the separate log files into one contiguous file without losing any data:

In Windows:
- Copy the RAWX files from the SD card into a normal Documents folder
- Open a cmd prompt (type cmd into the search box on the toolbar and hit enter)
- cd (change directory) into the Documents folder
- Use the following command to concatenate all the rover UBX RAWX files into a single file
- copy /b r_*.ubx rover.ubx
- Likewise, this command will concatenate the base files into a single file
- copy /b b_*.ubx base.ubx

In Linux:
- cat r_*.ubx > rover.ubx
- cat b_*.ubx > base.ubx

The HHMMSS filename format ensures that the files are concatenated in the correct order. If your files straddle UTC midnight, you will have to combine the files
from day1 and day2 separately first and then combine the two day files together.

You can then process a pair of base and rover files using [RTKLIB](http://rtkexplorer.com/downloads/rtklib-code/).


## Next > [UBX.md](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/UBX.md)



