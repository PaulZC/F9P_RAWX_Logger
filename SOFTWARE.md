# SOFTWARE

This guide describes how to install the Arduino IDE and all the libraries you will need for the F9P_RAWX_Logger.

The guide assumes you will be using Windows; the process would be slightly different under Linux or OSX.

## Arduino IDE

To begin, download and install the latest version of the [Arduino IDE](https://www.arduino.cc/en/Main/Software).
There are versions for Windows, Mac OS X and 32/64 bit Linux. The installation should be straight forward but there
are [step-by-step instructions](https://www.arduino.cc/en/Guide/HomePage) available if you need them.

The download page will ask you if you would like to make a donation to support Arduino. Please do make a donation if you can.

I recommend using the default options and selecting "Always trust software from Arduino AG" and "Always trust software from Adafruit" when asked.

Once installation is complete, the IDE should open and display a blank sketch.

## Install the Arduino and Adafruit SAMD Boards

A full guide on how to install the Arduino and Adafruit Boards is available on [the Adafruit website](https://learn.adafruit.com/adafruit-feather-m0-adalogger/setup)

In summary:
- Click on the File menu and then Preferences
- Add https://adafruit.github.io/arduino-board-index/package_adafruit_index.json to the "Additional Boards Manager URLs" box
- Click OK
- Click on the Tools menu, hover over the line which says "Board" and then click on "Boards Manager"
- Let the Boards Manager window update the list of available libraries
- Scroll down the list until you see "Arduino SAMD Boards (32-bits ARM Cortex-M0+) by Arduino"
- Hover over that line and click "Install"
- Wait for the installation to complete
- Scroll further down the list and install the "Adafruit SAMD Boards"
- Close the Boards Manager

![URLs](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/URLs.JPG)

![Boards](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Boards.JPG)

You may see warning messages about "skipping script execution". You can ignore these.

Now quit the IDE.

## Blink

You can now connect your Adalogger board to your computer using a standard micro-USB cable. All being well,
you should hear Windows acknowledge the Adalogger. It should appear as a new COM port.

Restart the IDE. It will open with the same blank sketch as before.

Click on the Tools menu and hover over the line which says "Port". All being well you should see a line which
says "COM4 (Adafruit Feather M0)". The COM number may be different depending on how many devices you have attached.
Click on that line.

![Port](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Port.JPG)

Click on the Tools menu again and hover over the line which says "Board". Use the down arrow to scroll down to the
Adafruit SAMD boards. Click on the line which says "Adafruit Feather M0".

After a short delay, you should see the text in the bottom right corner of the IDE window
change to "Adafruit Feather M0 on COM4".

![Board](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Board.JPG)

Click on the File menu and hover over the line which says "Examples". Hover over the line which says "01.Basics"
and click on the entry which says "Blink".

A new IDE window should appear containing the Blink code. You can now close the IDE window that contains the blank
sketch, leaving only the Blink window open.

Click on the arrow icon below the Edit menu. This will compile the blink code and upload it onto the Adalogger.

All being well, the red LED on the Adalogger should start to blink once per second.

You may notice that the Adalogger COM port number has changed. This is normal. If you have problems please refer
to the [Adafruit tutorial](https://learn.adafruit.com/adafruit-feather-m0-adalogger/using-with-arduino-ide).

Now would be a good time to connect a small (~100mAh) LiPo battery to the Adalogger so it can start charging.

## Libraries

We now need to install some libraries that are required by the RAWX_Logger_F9P code.

### Adafruit GPS Library

RAWX_Logger_F9P uses the Adafruit GPS library to parse the standard NMEA messages output by the F9P to set the M0's
Real Time Clock (RTC).

Click on the Tools menu and then click "Manage Libraries". The Library Manager window will open.

Use the filter text box to search for "Adafruit GPS". Hover over the line which says "Adafruit GPS Library"
and click "Install".

### RTCZero

RAWX_Logger_F9P uses the Arduino RTCZero library to set the SAMD M0 RTC.

Use the library manager filter text box to search for "RTCZero". Hover over the line which says "RTCZero by Arduino"
and click "Install".

### Bill Greiman's SdFat

RAWX_Logger_F9P uses Bill Greiman's SdFat to access the Adalogger micro-SD card, write data to the RAWX log file rapidly
and set the create, write and access timestamps.

Use the library manager filter text box to search for "sdfat". Hover over the line which says "SdFat by Bill Greiman"
and click "Install".

You can now close the library manager.

## Download RAWX_Logger_F9P

You can find the logger code in the [Arduino folder](https://github.com/PaulZC/F9P_RAWX_Logger/tree/master/Arduino)

The best way to download the code is to clone the repository using GitHub Desktop. That way you can keep up to date with new versions.

If you don't want to do that, you can use the use the green "Clone or download" button on the main repo page to "Download ZIP" instead.
It depends on which browser you are using, but you should end up with a file called RAWX_Logger_F9P-master.zip in your
Downloads directory.

- Open File Explorer (right-click on the Windows icon in the bottom left corner of your screen then click on "File Explorer")
- Click on your "Downloads" quick access link
- Double-click on the RAWX_Logger_F9P-master.zip file
- Double-click on the RAWX_Logger_F9P-master folder
- Double-click on the Arduino folder
- Right-click on the RAWX_Logger_F9P folder and click "Copy"
- Click on your "Documents" quick access link
- Double-click on the "Arduino" folder
- Right-click and select "Paste"

All being well, you should now have a Documents\Arduino\RAWX_Logger_F9P folder containing (only) RAWX_Logger_F9P.ino

In the Ardino IDE, click on the File menu and then "Open...". When the file window opens double-click on the RAWX_Logger_F9P folder
and then double-click on the RAWX_Logger_F9P.ino file. The logger code will open in a new window. You can now close the Blink window.

## Serial RX Buffer

One final change you need to make is to increase the size of the serial receive buffer in the Arduino board definition. We need the buffer to be large enough
so it can hold the incoming RAWX serial data while we close the SD card log file and open a new one.

You will need to use File Explorer (in Windows) to navigate to:
- C:\Users\<your user name>\AppData\Local\Arduino15\packages\adafruit\hardware\samd\1.4.1\cores\arduino

Some of these directories are hidden. You will need to click on the View menu and ensure that both the "Hidden items" and "File name extensions"
tick boxes are ticked.

Find the file called RingBuffer.h. Right-click on it and choose "Open with" and "WordPad".

Scroll down and find the line which says:
- #define SERIAL_BUFFER_SIZE 256

Change the 256 to 6144. Save the file and quit WordPad. The next time the Arduino code is compiled, the serial buffer will be increased.

![RingBuffer](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/RingBuffer.JPG)

You are now ready to upload the logger code to the Adalogger using the arrow icon below the Edit menu in the IDE just like you did with Blink.

Sadly, you will need to edit RingBuffer.h each time you update the Adafruit SAMD boards. If your RAWX_Logger stops working, or the log files
appear corrupt, the serial buffer size will proably have been reset back to 256.

This is a very inefficient way to increase the buffer size as both the transmit and receive buffers are increased in size for both Serial1 and Serial5. I.e.
increasing the buffer size uses _four_ times as much RAM as you might expect. You can reduce this to two times if you edit variant.h and variant.cpp for the
feather_m0 and comment out the definitions for Serial5. If anyone knows an 'easy' way to increase the size of the Serial1 receive buffer only, without creating
a custom variant, please get in touch!

### variant.h (in ...\1.4.1\variants\feather_m0):
![VariantH](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/VariantH.JPG)

### variant.cpp (in ...\1.4.1\variants\feather_m0):
![VariantCPP](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/VariantCPP.JPG)

## Next > [HARDWARE.md](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/HARDWARE.md)



