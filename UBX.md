# UBX

This guide describes how the RAWX_Logger_F9P Arduino code works: enabling the ZED-F9P RAWX messages using the binary UBX protocol and logging them to SD card with SdFat

## Resources

You can find the latest ZED-F9P documentation on the u-blox website:
- https://www.u-blox.com/en/product/zed-f9p-module
- Product summary: https://www.u-blox.com/sites/default/files/ZED-F9P_ProductSummary_%28UBX-17005151%29.pdf
- Data sheet: https://www.u-blox.com/sites/default/files/ZED-F9P_DataSheet_%28UBX-17051259%29.pdf
- Integration manual: https://www.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_%28UBX-18010802%29.pdf
- Interface manual: https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

## UBX Protocol

The UBX binary protocol is defined in section 5 of the interface manual.

UBX message frames comprise:
- Two sync characters (0xB5 and 0x62)
- The message class (1 byte)
- The message ID (1 byte)
- The message payload length (2 bytes, little endian)
- The payload
- A two byte checksum

The Arduino code contains a function called "sendUBX" which will automatically calculate and append the checksum for you.
The messages are defined in the code as byte arrays (uint8_t).

## Initialisation: Baud Rate

When power is applied to the ZED-F9P, it starts to output standard NMEA navigation messages on its UART1 port at 38400 Baud
(8 data bits, no parity bit, 1 stop bit). The following messages are activated at startup: GGA, GLL, GSA, GSV, RMC, VTG, TXT(INF).
See section 8 of the data sheet for further details.

The first thing we need to do is to increase the baud rate to 230400 baud so the interface can cope with the RAWX message rates.
The relevant parts of the Arduino code are:

```
#include <Adafruit_GPS.h> // Include the Adafruit GPS library
Adafruit_GPS GPS(&Serial1); // Create a GPS instance using Serial1
#define GPSECHO false // Disable message echoing
GPS.begin(38400); // Open Serial1 at 38400 baud
sendUBX(setUART1BAUD, len_setUART1BAUD); // Change the ZED-F9P UART1 Baud Rate
delay(1100); // Wait
GPS.begin(230400); // Restart Serial1 at 230400 baud
```

The important part is the definition of setUART1BAUD, which uses a UBX-CFG-VALSET message with a key ID of 0x40520001 (CFG-UART1-BAUDRATE)
to set the UART1 baud rate in RAM (only) to 230400 baud.
See section 5.9.27 of the interface manual for UBX-CFG-VALSET and section 6.7.24 for configuration of the UART1 interface.
230400 in hexadecimal is 0x00038400, which becomes 0x00, 0x84, 0x03, 0x00 in little endian format.

```
static const uint8_t setUART1BAUD[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0c, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x40, 0x52, 0x00, 0x01,  0x00, 0x84, 0x03, 0x00 };
static const int len_setUART1BAUD = 18;
```

## Initialisation: UBX messages

In case the logger was reset while already logging UBX messages, we need to make sure the UBX RAWX, SFRBX and TIM_TM2 messages are disabled as they will confuse the
GPS library. We do this by sending a UBX-CFG-VALSET message with key IDs of:
- 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
- 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
- 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)

and values (rates) of zero:

```
setRAWXoff[] = { 0xb5, 0x62,  0x06, 0x8a,  0x13, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x20, 0x91, 0x02, 0xa5,  0x00,  0x20, 0x91, 0x02, 0x32,  0x00,  0x20, 0x91, 0x01, 0x79,  0x00 };
len_setRAWXoff = 25;
```

## Initialisation: NMEA messages

To keep things simple for the Adafruit GPS library, we need to disable the GLL, GSA, GSV, VTG, and TXT(INF) messages
and make sure that the GGA and RMC messages are enabled.
We do this by sending a UBX-CFG-VALSET message with key IDs of:
- 0x209100ca (CFG-MSGOUT-NMEA_ID_GLL_UART1)
- 0x209100c0 (CFG-MSGOUT-NMEA_ID_GSA_UART1)
- 0x209100c5 (CFG-MSGOUT-NMEA_ID_GSV_UART1)
- 0x209100b1 (CFG-MSGOUT-NMEA_ID_VTG_UART1)
- 0x20920007 (CFG-INFMSG-NMEA_UART1)
- 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
- 0x209100ac (CFG-MSGOUT-NMEA_ID_RMC_UART1)

```
setNMEAon[] = { 0xb5, 0x62,  0x06, 0x8a,  0x27, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x20, 0x91, 0x00, 0xca,  0x00,
  0x20, 0x91, 0x00, 0xc0,  0x00,  0x20, 0x91, 0x00, 0xc5,  0x00,
  0x20, 0x91, 0x00, 0xb1,  0x00,  0x20, 0x92, 0x00, 0x07,  0x00,
  0x20, 0x91, 0x00, 0xbb,  0x01,  0x20, 0x91, 0x00, 0xac,  0x01 };
len_setNMEAon = 45;
```

## Initialisation: Talker ID

As the ZED-F9P can track all four major GNSS constellations (GPS, Galileo, GLONASS and BeiDou) concurrently, it will normally output NMEA
messages which begin "GN" instead of "GP". This could confuse the Adafruit GPS library (and TinyGPS) so we need to change the "talker ID" to "GP".
We do this by sending a UBX-CFG-VALSET message with a key ID of:
- 0x20930031 (CFG-NMEA-MAINTALKERID)

and a value of 1:

```
setTALKERID[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x20, 0x93, 0x00, 0x31,  0x01 };
len_setTALKERID = 15;
```

## Initialisation: Set Measurement Rate

During RAWX logging, the measurement rate will be increased to (e.g.) 4 Hz. During initialisation, we need to make sure it is set back to 1 Hz.
We do this by sending a UBX-CFG-VALSET message with a key ID of:
- 0x30210001 (CFG-RATE-MEAS)

and a value of 1000 milliseconds. 1000 in hexadecimal is 0x03e8, which becomes 0xe8, 0x03 in U2 little endian format:

```
setRATE_1Hz[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x30, 0x21, 0x00, 0x01,  0xe8, 0x03 };
len_setRATE = 16;
```

Later we will use a value of 250 milliseconds (0x00fa) to set the rate to 4 Hz for RAWX logging:

```
setRATE_4Hz[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x30, 0x21, 0x00, 0x01,  0xfa, 0x00 };
```

## Initialisation: Set Navigation Dynamic Model

For the base logger, we need to set the navigation dynamic model to STATionary. We do this by sending a UBX-CFG-VALSET message with a key ID of:
- 0x20110021 (CFG-NAVSPG-DYNMODEL)

and a value of 2:

```
setNAVstationary[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x20, 0x11, 0x00, 0x21,  0x02 };
len_setNAV = 15;
```

For the rover logger, we set the dynamic model to "AIR1" (airborne with <1g acceleration) using a value of 6:

```
setNAVair1g[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x20, 0x11, 0x00, 0x21,  0x06 };
```

The values for the other dynamic models are defined at the end of section 6.7.12 in the interface manual.

## NMEA Parsing

We can now be confident that only the NMEA GPGGA and GPRMC messages are being produced. So we can use the Adafruit GPS library to parse them:

```
char c = GPS.read(); // read data from the GNSS
if (GPSECHO) // if you want to debug, this is a good time to do it!
  if (c) Serial.print(c);
if (GPS.newNMEAreceived()) { // if a sentence is received, we can check the checksum, parse it...
  // we can fail to parse a sentence in which case we should just wait for another
  if (!GPS.parse(GPS.lastNMEA())) break;
```

## Set the RTC

The GPS.fix flag will be set true once the ZED-F9P has established a fix. We can then use the GNSS (UTC) time to set the SAMD Real Time Clock:

```
// Set and start the RTC
alarmFlag = false; // Make sure alarm flag is clear
rtc.begin(); // Start the RTC
rtc.setTime(GPS.hour, GPS.minute, GPS.seconds); // Set the time
rtc.setDate(GPS.day, GPS.month, GPS.year); // Set the date
```

We can then use RTC alarm interrupts to close the RAWX log file and open a new one every INTERVAL minutes:

```
rtc.setAlarmSeconds(0); // Set RTC Alarm Seconds to zero
uint8_t nextAlarmMin = ((GPS.minute+INTERVAL)/INTERVAL)*INTERVAL; // Calculate next alarm minutes
nextAlarmMin = nextAlarmMin % 60; // Correct hour rollover
rtc.setAlarmMinutes(nextAlarmMin); // Set RTC Alarm Minutes
rtc.enableAlarm(rtc.MATCH_MMSS); // Alarm Match on minutes and seconds
rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
```

We can also use the RTC to set the create, write and access timestamps of the log file using SdFat.

## RAWX messages
          
Now that the ZED-F9P has established a fix and we have set the RTC, we can disable the GGA and RMC NMEA messages.
We do this by sending a UBX-CFG-VALSET message with key IDs of:
- 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
- 0x209100ac (CFG-MSGOUT-NMEA_ID_RMC_UART1)

and values of zero:

```
setNMEAoff[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0e, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x20, 0x91, 0x00, 0xbb,  0x00,  0x20, 0x91, 0x00, 0xac,  0x00 };
len_setNMEAoff = 20;
```

We now need to speed up the measurement rate to 4 Hz using the setRATE_4Hz message we defined earlier.

Finally, we can enable the RAWX, SFRBX and TIM_TM2 messages. We do this by sending a UBX-CFG-VALSET message with key IDs of:
- 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
- 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
- 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)

and values (rates) of 1:

```
setRAWXon[] = { 0xb5, 0x62,  0x06, 0x8a,  0x13, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x20, 0x91, 0x02, 0xa5,  0x01,
  0x20, 0x91, 0x02, 0x32,  0x01,
  0x20, 0x91, 0x01, 0x79,  0x01 };
len_setRAWXon = 25;
```

## Opening the log file

We can now be confident that only UBX RXM_RAWX, RXM_SFRBX and TIM_TM2 messages are being produced. So _all_ we need to do is open a log file on the SD
card and throw everything we receive on Serial1 into it. You will find the code that opens the log file starting with the line:

```
case open_file:
```

## Write the RAWX messages to the log file

You will find the code that writes the RAWX messages to the log file starting with the line:

```
case write_file:
```

To make the writing as efficient and as fast as possible, data is written to the SD card by SdFat in packets of 512 bytes (SDpacket).

## Restart Logging, Stop Button and Low Battery

The code checks the UBX serial data continuously, counting the number of bytes and calculating the expected checksum for each message.
If the checksum does not match, due to an error or dropped byte in the serial data, the log file is automatically closed and a new one opened.
You can find the code that does this startine with the line:

```
case restart_file:
```

The code also checks to see if the stop button has been pressed or if the battery is low. If either condition is true, the code will close the log file.

If the stop button was pressed, the code will do nothing more until the Adalogger is reset.

If the battery voltage became too low, the code will wait until the voltage recovers (e.g. when a new UAV battery is inserted) and then it will
start logging again.

## Open a new log file

After an RTC alarm interrupt, the code will close the current log file and open a new one. You will find the code that does that starting
with the line:

```
case new_file:
```

You can change how often a new file is opened by changing the value of INTERVAL in the code before you upload it to the Adalogger.

The code will wait until it gets to the end of the current UBX frame before opening the new file, to ensure the data in the individual log files
is as clean and contiguous as possible.

UBX messages sent from the Adalogger to the ZED-F9P are all acknowledged (or nacknowledged) by a short ack/nack message. The code expects these
and discards them without writing them into the log file. RTKLIB can probably cope with these acks/nacks being in the log files, but the code does
try to make life as easy as possible for RTKLIB by discarding them.

## Checking everyting is OK

If you have access to an oscilloscope, you can check how much data is being received on the Adalogger RX pin. You should see bursts of data
every 250 milliseconds. The RX line goes high (3.3V) when idle. Check that the RX line is not continuously busy. There must be gaps at the
end of each 250 millisecond burst. If the gaps are small, you may need to increase the UART baud rate higher than 230400 baud or decrease the RAWX
measurement rate to 2 Hz or lower.

Likewise, use your oscilloscope to monitor the red LED (digital pin 13). The red LED is on during SD card writes. Again the code must not be
writing to the card continuously. There must be gaps between writes every 250 milliseconds. If the SD card is continously busy: replace your SD
card with a faster one; decrease the RAWX measurement rate; or consider changing the SdFat clock speed by editing the line which says:

```
if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
```

