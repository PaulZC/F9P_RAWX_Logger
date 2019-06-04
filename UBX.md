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
The messages are defined in the code as byte arrays (uint8_t). Make sure the message payload length is defined correctly inside each message (bytes 4 and 5)
in little endian format.

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
sendUBX(setUART1BAUD); // Change the ZED-F9P UART1 Baud Rate
delay(1100); // Wait
GPS.begin(230400); // Restart Serial1 at 230400 baud
```

The important part is the definition of setUART1BAUD, which uses a UBX-CFG-VALSET message with a key ID of 0x40520001 (CFG-UART1-BAUDRATE)
to set the UART1 baud rate in RAM (only) to 230400 baud.
See section 5.9.27 of the interface manual for UBX-CFG-VALSET and section 6.7.24 for configuration of the UART1 interface.
230400 in hexadecimal is 0x00038400, which becomes 0x00, 0x84, 0x03, 0x00 in little endian format.
The key ID needs to be sent in little endian format too, so it becomes 0x01, 0x00, 0x52, 0x40.

```
static const uint8_t setUART1BAUD[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0c, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x52, 0x40,  0x00, 0x84, 0x03, 0x00 };
```

## Initialisation: UBX messages

In case the logger was reset while the ZED-F9P was sending RAWX messages, we need to make sure the UBX RAWX, SFRBX and TIM_TM2 messages
are disabled as they will confuse the GPS library. We do this by sending a UBX-CFG-VALSET message with key IDs of:
- 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
- 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
- 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)

and values (rates) of zero:

```
setRAWXoff[] = { 0xb5, 0x62,  0x06, 0x8a,  0x13, 0x00,  0x00, 0x01, 0x00, 0x00,
  0xa5, 0x02, 0x91, 0x20,  0x00,  0x32, 0x02, 0x91, 0x20,  0x00,  0x79, 0x01, 0x91, 0x20,  0x00 };
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
  0xca, 0x00, 0x91, 0x20,  0x00,
  0xc0, 0x00, 0x91, 0x20,  0x00,  0xc5, 0x00, 0x91, 0x20,  0x00,
  0xb1, 0x00, 0x91, 0x20,  0x00,  0x07, 0x00, 0x92, 0x20,  0x00,
  0xbb, 0x00, 0x91, 0x20,  0x01,  0xac, 0x00, 0x91, 0x20,  0x01 };
```

## Initialisation: Talker ID

As the ZED-F9P can track all four major GNSS constellations (GPS, Galileo, GLONASS and BeiDou) concurrently, it will normally output NMEA
messages which begin "GN" instead of "GP". This could confuse the Adafruit GPS library (and TinyGPS) so we need to change the "talker ID" to "GP".
We do this by sending a UBX-CFG-VALSET message with a key ID of:
- 0x20930031 (CFG-NMEA-MAINTALKERID)

and a value of 1:

```
setTALKERID[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x31, 0x00, 0x93, 0x20,  0x01 };
```

## Initialisation: Set Measurement Rate

During RAWX logging, the measurement rate will be increased to (e.g.) 4 Hz. During initialisation, we need to make sure it is set back to 1 Hz.
We do this by sending a UBX-CFG-VALSET message with a key ID of:
- 0x30210001 (CFG-RATE-MEAS)

and a value of 1000 milliseconds. 1000 in hexadecimal is 0x03e8, which becomes 0xe8, 0x03 in U2 little endian format:

```
setRATE_1Hz[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x21, 0x30,  0xe8, 0x03 };
```

Later we will use a value of 250 milliseconds (0x00fa) to set the rate to 4 Hz for RAWX logging:

```
setRATE_4Hz[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x21, 0x30,  0xfa, 0x00 };
```

## Initialisation: Set Navigation Dynamic Model

For the base logger, we need to set the navigation dynamic model to STATionary. We do this by sending a UBX-CFG-VALSET message with a key ID of:
- 0x20110021 (CFG-NAVSPG-DYNMODEL)

and a value of 2:

```
setNAVstationary[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x21, 0x00, 0x11, 0x20,  0x02 };
```

For the rover logger, we set the dynamic model to "AIR1" (airborne with <1g acceleration) using a value of 6:

```
setNAVair1g[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,
  0x21, 0x00, 0x11, 0x20,  0x06 };
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
  0xbb, 0x00, 0x91, 0x20,  0x00,  0xac, 0x00, 0x91, 0x20,  0x00 };
```

We now need to speed up the measurement rate to 4 Hz using the setRATE_4Hz message we defined earlier.

Finally, we can enable the RAWX, SFRBX and TIM_TM2 messages. We do this by sending a UBX-CFG-VALSET message with key IDs of:
- 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
- 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
- 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)

and values (rates) of 1:

```
setRAWXon[] = { 0xb5, 0x62,  0x06, 0x8a,  0x13, 0x00,  0x00, 0x01, 0x00, 0x00,
  0xa5, 0x02, 0x91, 0x20,  0x01,
  0x32, 0x02, 0x91, 0x20,  0x01,
  0x79, 0x01, 0x91, 0x20,  0x01 };
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
You can find the code that does this starting with the line:

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

The length of the RAWX messages increases depending on how many satellites are being tracked. We need to check that we are not trying to log
more data than the serial interface and SD card can cope with.

If you have access to an oscilloscope, you can check how much data is being received on the Adalogger RX pin. You should see bursts of data
every 250 milliseconds. The RX line goes high (3.3V) when idle. Check that the RX line is not continuously busy. There must be gaps at the
end of each 250 millisecond interval. If the gaps are small, you may need to increase the UART baud rate higher than 230400 baud or decrease the RAWX
measurement rate to 2 Hz or lower.

![Serial.JPG](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/Serial.JPG)

Likewise, use your oscilloscope to monitor the red LED (digital pin 13). The red LED is on during SD card writes. Again the code must not be
writing to the card continuously. There must be gaps between writes every 250 milliseconds.

![SDwrite.JPG](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/SDwrite.JPG)

If the SD card is continously busy: replace your SD card with a faster one; decrease the RAWX measurement rate; or consider changing the SdFat clock speed
by editing the line which says:

```
if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
```

## Serial RX Buffer

The RAWX data rates can be high when the logger is tracking multiple satellites. This could be a problem when closing one SD log file and opening the next
as we need to rely on the serial receive buffer being large enough to buffer the data until the new file is open. Unfortunately, by default, the
SERIAL_BUFFER_SIZE is only 256 bytes, which isn't large enough and causes data to be dropped.

In previous projects, I have recommended increasing the size of the serial buffer by editing the file RingBuffer.h and changing the value of SERIAL_BUFFER_SIZE.
This isn't an efficient way to increase the buffer size as:
- both receive and transmit buffers for both Serial1 and Serial5 are increased in size, so you end up using four times as much RAM as necessary
- the buffer size will be reset each time the Adafruit boards is updated

In this project, we work around this by creating a separate large SerialBuffer using the same class as a normal serial RingBuffer. A timer interrupt is used to
check for the arrival of Serial1 data and move it into SerialBuffer. The main loop then reads the data from SerialBuffer using the inherited
.available and .read_char functions.

Here is the line that defines the large SerialBuffer:

```
// Define SerialBuffer as a large RingBuffer which we will use to store the Serial1 receive data
// That way, we do not need to increase the size of the Serial1 receive buffer (by editing RingBuffer.h)
RingBufferN<8192> SerialBuffer; // Define SerialBuffer as a RingBuffer of size 8192 bytes
```

Here is the code that sets up the TC3 timer interrupt. It is based on the code provided by
[Sheng Chen jdneo](https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989)

```
// TimerCounter3 functions to copy Serial1 receive data into SerialBuffer
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 16

// Set TC3 Interval (sec)
void setTimerInterval(float intervalS) {
  int compareValue = intervalS * CPU_HZ / TIMER_PRESCALER_DIV;
  if (compareValue > 65535) compareValue = 65535;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

// Start TC3 with a specified interval
void startTimerInterval(float intervalS) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 16
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerInterval(intervalS);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_SetPriority(TC3_IRQn, 3); // Set the TC3 interrupt priority to 3 (lowest)
  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}
```

And here is the interrupt handler that copies the serial data from the (small) serial receive buffer into our (large) RingBuffer:

```
// TC3 Interrupt Handler
void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // copy any available Serial1 data into SerialBuffer
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    int available1 = Serial1.available(); // Check if there is any data waiting in the Serial1 RX buffer
    while (available1 > 0) { 
        SerialBuffer.store_char(Serial1.read()); // If there is, copy it into our RingBuffer
        available1--;
    }
  }
}
```

In the main loop, we enable the timer interrupt _after_ processing the NMEA messages using the Adafruit GPS library:

```
// Now that Serial1 should be idle and the buffer empty, start TC3 interrupts to copy all new data into SerialBuffer
// Set the timer interval to 10 * 10 / 230400 = 0.000434 secs (10 bytes * 10 bits (1 start, 8 data, 1 stop) at 230400 baud)
startTimerInterval(0.000434); 
```

Then to read data from SerialBuffer, we can use the inherited .available and .read_char methods:

```
if (SerialBuffer.available()) {
  uint8_t c = SerialBuffer.read_char();
```

The interrupt service routine takes between 3 and 25 usec to execute depending on how many characters are available (0 to 10). This is a
significant overhead given that the ISR runs every 434 usec, but it is a price worth paying to avoid having to edit RingBuffer.h.

![TC3_ISR_1.JPG](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/TC3_ISR_1.JPG)

![TC3_ISR_2.JPG](https://github.com/PaulZC/F9P_RAWX_Logger/blob/master/img/TC3_ISR_2.JPG)


Enjoy!

**_Paul_**








