// RAWX_Logger_F9P

// Logs RXM-RAWX, RXM-SFRBX and TIM-TM2 data from u-blox ZED_F9P GNSS to SD card
// Also logs NAV-STATUS messages for Survey_In mode
// Optionally logs NAV_POSLLH and NAV_PVT too (if enabled - see lines 220 and 221)

// Changes to a new log file every INTERVAL minutes

// Define how long we should log in minutes before changing to a new file
// Sensible values are: 5, 10, 15, 20, 30, 60
// Must be <= 60 (or RTC alarm code needs to be updated to match on HHMMSS)
const int INTERVAL = 15;

// Define how long we should wait in msec (approx.) for residual RAWX data before closing the last log file
// For a measurement rate of 4Hz (250msec), 300msec is a sensible value. i.e. slightly more than one measurement interval
const int dwell = 300;

// This code is written for the Adalogger M0 Feather
// https://www.adafruit.com/products/2796
// https://learn.adafruit.com/adafruit-feather-m0-adalogger

// GNSS data is provided by the SparkFun GPS-RTK2 Board
// https://www.sparkfun.com/products/15136

// Choose a good quality SD card. Some cheap cards can't handle the write rate.
// Ensure the card is formatted as FAT32.

// Send serial debug messages
//#define DEBUG // Comment this line out to disable debug messages

// Connect modePin to GND to select base mode. Leave open for rover mode.
#define modePin 14 // A0 / Digital Pin 14

// Connect a normally-open push-to-close switch between swPin and GND.
// Press it to stop logging and close the log file.
#define swPin 15 // A1 / Digital Pin 15 (0.2" away from the GND pin on the Adalogger)

// Pin A2 (Digital Pin 16) is reserved for the ZED-F9P EXTINT signal
// The code uses this an an interrupt to set the NeoPixel to white
#define ExtIntPin 16 // A2 / Digital Pin 16

// Connect A3 (Digital Pin 17) to GND to select SURVEY_IN mode when in BASE mode
#define SurveyInPin 17 // A3 / Digital Pin 17

// LEDs

//#define NoLED // Uncomment this line to completely disable the LEDs
//#define NoLogLED // Uncomment this line to disable the LEDs during logging only

// NeoPixel Settings
//#define NeoPixel // Uncomment this line to enable a NeoPixel on the same pin as RedLED

// The red LED flashes during SD card writes
#define RedLED 13 // The red LED on the Adalogger is connected to Digital Pin 13
// The green LED indicates that the GNSS has established a fix 
#define GreenLED 8 // The green LED on the Adalogger is connected to Digital Pin 8

// NeoPixel
#ifdef NeoPixel
#include <Adafruit_NeoPixel.h> // Support for the WB2812B
#define swap_red_green // Uncomment this line if your WB2812B has red and green reversed
#ifdef swap_red_green
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RedLED, NEO_GRB + NEO_KHZ800); // GRB WB2812B
#else
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RedLED, NEO_RGB + NEO_KHZ800); // RGB WB2812B
#endif
#define LED_Brightness 32 // 0 - 255 for WB2812B
#endif

// Include the Adafruit GPS Library
// https://github.com/adafruit/Adafruit_GPS
// This is used at the start of the code to establish a fix and
// provide the date and time for the RAWX log file filename
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial1); // M0 hardware serial
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

// Fast SD card logging using Bill Greiman's SdFat
// https://github.com/greiman/SdFat
// From FatFile.h:
//   * \note Data is moved to the cache but may not be written to the
//   * storage device until sync() is called.

#include <SPI.h>
#include <SdFat.h>
const uint8_t cardSelect = 4; // Adalogger uses D4 as the SD SPI select
SdFat sd;
SdFile rawx_dataFile;
// The log filename starts with "r_" for the rover and "b_" for the static base
bool base_mode = true; // Flag to indicate if the code is in base or rover mode
char rawx_filename[] = "20000000/b_000000.ubx"; // the b will be replaced by an r if required
char dirname[] = "20000000";
long bytes_written = 0;

bool survey_in_mode = false; // Flag to indicate if the code is in survey_in mode

// Timer to indicate if an ExtInt has been received
volatile unsigned long ExtIntTimer; // Load this with millis plus 1000 to show when the ExtInt LED should be switched off

// Define packet size, buffer and buffer pointer for SD card writes
const size_t SDpacket = 512;
uint8_t serBuffer[SDpacket];
size_t bufferPointer = 0;
int numBytes;

// Battery voltage
float vbat;
#define LOWBAT 3.55 // Low battery voltage

// Include Real Time Clock support for the M0
// https://github.com/arduino-libraries/RTCZero
#include <RTCZero.h> // M0 Real Time Clock
RTCZero rtc; // Create an rtc object
volatile bool alarmFlag = false; // RTC alarm (interrupt) flag

// Count number of valid fixes before starting to log
#define maxvalfix 10 // Collect at least this many valid fixes before logging starts
int valfix = 0;

bool stop_pressed = false; // Flag to indicate if stop switch was pressed to stop logging

// Define SerialBuffer as a large RingBuffer which we will use to store the Serial1 receive data
// Actual Serial1 receive data will be copied into SerialBuffer by a timer interrupt
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
// That way, we do not need to increase the size of the Serial1 receive buffer (by editing RingBuffer.h)
RingBufferN<8192> SerialBuffer; // Define SerialBuffer as a RingBuffer of size 8192 bytes

// Loop Steps
#define init          0
#define start_rawx    1
#define open_file     2
#define write_file    3
#define new_file      4
#define close_file    5
#define restart_file  6
int loop_step = init;

// UBX State
#define looking_for_B5          0
#define looking_for_62          1
#define looking_for_class       2
#define looking_for_ID          3
#define looking_for_length_LSB  4
#define looking_for_length_MSB  5
#define processing_payload      6
#define looking_for_checksum_A  7
#define looking_for_checksum_B  8
#define sync_lost               9
int ubx_state = looking_for_B5;
int ubx_length = 0;
int ubx_class = 0;
int ubx_ID = 0;
int ubx_checksum_A = 0;
int ubx_checksum_B = 0;
int ubx_expected_checksum_A = 0;
int ubx_expected_checksum_B = 0;

// Definitions for u-blox F9P UBX-format (binary) messages
// Each message begins with: <Sync_Char_1 0xb5>, <Sync_Char_2 0x62>, <Class>, <ID>, <Length_LSB>, <Length_MSB>
// The message definitions need to include the 0xb5 and 0x62 sync chars 
// Each message needs to have the payload length defined (Length_LSB + (256 * Length_MSB))
// The message definitions don't contain the checksum bytes - these are calculated and appended by sendUBX
// The UBX-CFG-VALSET messages are only applied to RAM (not battery-backed memory or flash)

// Set UART1 to 230400 Baud
// UBX-CFG-VALSET message with a key ID of 0x40520001 (CFG-UART1-BAUDRATE) and a value of 0x00038400 (230400 decimal)
static const uint8_t setUART1BAUD[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0c, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x52, 0x40,  0x00, 0x84, 0x03, 0x00 };

// Disable I2C Interface
// UBX-CFG-VALSET message with a key ID of 0x10510003 (CFG-I2C-ENABLED) and a value of 0
static const uint8_t setI2Coff[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x03, 0x00, 0x51, 0x10,  0x00 };

// Disable UART2 Interface
// UBX-CFG-VALSET message with a key ID of 0x10530005 (CFG-UART2-ENABLED) and a value of 0
static const uint8_t setUART2off[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x05, 0x00, 0x53, 0x10,  0x00 };

// Disable USB Interface
// UBX-CFG-VALSET message with a key ID of 0x10650001 (CFG-USB-ENABLED) and a value of 0
static const uint8_t setUSBoff[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x65, 0x10,  0x00 };

// Disable the RXM_RAWX, RXM_SFRBX, TIM_TM2, NAV_POSLLH, NAV_PVT and NAV_STATUS binary messages in RAM
// UBX-CFG-VALSET message with key IDs of:
// 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
// 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
// 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)
// 0x2091002a (CFG-MSGOUT-UBX_NAV_POSLLH_UART1)
// 0x20910007 (CFG-MSGOUT-UBX_NAV_PVT_UART1)
// 0x2091001b (CFG-MSGOUT-UBX_NAV_STATUS_UART1)
// and values (rates) of zero:
static const uint8_t setRAWXoff[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x22, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0xa5, 0x02, 0x91, 0x20,  0x00,
  0x32, 0x02, 0x91, 0x20,  0x00,
  0x79, 0x01, 0x91, 0x20,  0x00,
  0x2a, 0x00, 0x91, 0x20,  0x00,
  0x07, 0x00, 0x91, 0x20,  0x00,
  0x1b, 0x00, 0x91, 0x20,  0x00 };

// Enable the RXM_RAWX, RXM_SFRBX, TIM_TM2, NAV_POSLLH, NAV_PVT and NAV_STATUS binary messages in RAM
// UBX-CFG-VALSET message with key IDs of:
// 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
// 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
// 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)
// 0x2091002a (CFG-MSGOUT-UBX_NAV_POSLLH_UART1)
// 0x20910007 (CFG-MSGOUT-UBX_NAV_PVT_UART1)
// 0x2091001b (CFG-MSGOUT-UBX_NAV_STATUS_UART1)
// and values (rates) of 1:
static const uint8_t setRAWXon[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x22, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0xa5, 0x02, 0x91, 0x20,  0x01,
  0x32, 0x02, 0x91, 0x20,  0x01,
  0x79, 0x01, 0x91, 0x20,  0x01,
  0x2a, 0x00, 0x91, 0x20,  0x00,   // Change the last byte from 0x01 to 0x00 to leave NAV_POSLLH disabled
  0x07, 0x00, 0x91, 0x20,  0x01,   // Change the last byte from 0x01 to 0x00 to leave NAV_PVT disabled
  0x1b, 0x00, 0x91, 0x20,  0x01 };

// Enable the NMEA GGA and RMC messages and disable the GLL, GSA, GSV, VTG, and TXT(INF) messages
// UBX-CFG-VALSET message with key IDs of:
// 0x209100ca (CFG-MSGOUT-NMEA_ID_GLL_UART1)
// 0x209100c0 (CFG-MSGOUT-NMEA_ID_GSA_UART1)
// 0x209100c5 (CFG-MSGOUT-NMEA_ID_GSV_UART1)
// 0x209100b1 (CFG-MSGOUT-NMEA_ID_VTG_UART1)
// 0x20920007 (CFG-INFMSG-NMEA_UART1)
// 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
// 0x209100ac (CFG-MSGOUT-NMEA_ID_RMC_UART1)
static const uint8_t setNMEAon[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x27, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0xca, 0x00, 0x91, 0x20,  0x00,
  0xc0, 0x00, 0x91, 0x20,  0x00,
  0xc5, 0x00, 0x91, 0x20,  0x00,
  0xb1, 0x00, 0x91, 0x20,  0x00,
  0x07, 0x00, 0x92, 0x20,  0x00,
  0xbb, 0x00, 0x91, 0x20,  0x01,
  0xac, 0x00, 0x91, 0x20,  0x01 };

// Disable the NMEA GGA and RMC messages
// UBX-CFG-VALSET message with key IDs of:
// 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
// 0x209100ac (CFG-MSGOUT-NMEA_ID_RMC_UART1)
static const uint8_t setNMEAoff[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x0e, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0xbb, 0x00, 0x91, 0x20,  0x00,
  0xac, 0x00, 0x91, 0x20,  0x00 };

// Set the Main NMEA Talker ID to "GP"
// UBX-CFG-VALSET message with a key ID of 0x20930031 (CFG-NMEA-MAINTALKERID) and a value of 1 (GP):
static const uint8_t setTALKERID[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x31, 0x00, 0x93, 0x20,  0x01 };

// Set the measurement rate
// UBX-CFG-VALSET message with a key ID of 0x30210001 (CFG-RATE-MEAS)
static const uint8_t setRATE_20Hz[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0x32, 0x00 };
static const uint8_t setRATE_10Hz[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0x64, 0x00 };
static const uint8_t setRATE_5Hz[]  = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xc8, 0x00 };
static const uint8_t setRATE_4Hz[]  = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xfa, 0x00 };
static const uint8_t setRATE_2Hz[]  = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xf4, 0x01 };
static const uint8_t setRATE_1Hz[]  = { 0xb5, 0x62,  0x06, 0x8a,  0x0a, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xe8, 0x03 };

// Set the navigation dynamic model
// UBX-CFG-VALSET message with a key ID of 0x20110021 (CFG-NAVSPG-DYNMODEL)
static const uint8_t setNAVportable[]   = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x00 };
static const uint8_t setNAVstationary[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x02 };
static const uint8_t setNAVpedestrian[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x03 };
static const uint8_t setNAVautomotive[] = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x04 };
static const uint8_t setNAVsea[]        = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x05 };
static const uint8_t setNAVair1g[]      = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x06 };
static const uint8_t setNAVair2g[]      = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x07 };
static const uint8_t setNAVair4g[]      = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x08 };
static const uint8_t setNAVwrist[]      = { 0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,  0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x09 };

// The following messages may be useful for RTK

// Set UART2 to 230400 Baud
// UBX-CFG-VALSET message with a key ID of 0x40530001 (CFG-UART2-BAUDRATE) and a value of 0x00038400 (230400 decimal)
static const uint8_t setUART2BAUD_230400[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0c, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x53, 0x40,  0x00, 0x84, 0x03, 0x00 };

// Set UART2 to 115200 Baud
// UBX-CFG-VALSET message with a key ID of 0x40530001 (CFG-UART2-BAUDRATE) and a value of 0x0001c200 (115200 decimal)
static const uint8_t setUART2BAUD_115200[] = { 0xb5, 0x62,  0x06, 0x8a,  0x0c, 0x00,  0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x53, 0x40,  0x00, 0xc2, 0x01, 0x00 };

// Set Survey_In mode
// UBX-CFG-VALSET message with a key IDs and values of:
// 0x20030001 (CFG-TMODE-MODE) and a value of 1
// 0x40030011 (CFG-TMODE-SVIN_ACC_LIMIT) and a value of 0x0000c350 (50000 decimal = 5 m)
// 0x40030010 (CFG-TMODE-SVIN_MIN_DUR) and a value of 0x0000003c (60 decimal = 1 min)
static const uint8_t setSurveyIn[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x19, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x03, 0x20,  0x01,
  0x11, 0x00, 0x03, 0x40,  0x50, 0xc3, 0x00, 0x00,
  0x10, 0x00, 0x03, 0x40,  0x3c, 0x00, 0x00, 0x00 };

// Enable RTCM message output on UART2
// UBX-CFG-VALSET message with the following key IDs
// Set the value byte to 0x01 to send an RTCM message at RATE_MEAS; set the value to 0x04 to send an RTCM message at 1/4 RATE_MEAS
// (i.e. assumes you will be logging RAWX data at 4 Hz. Adjust accordingly)
// 0x209102bf (CFG-MSGOUT-RTCM_3X_TYPE1005_UART2)
// 0x209102ce (CFG-MSGOUT-RTCM_3X_TYPE1077_UART2)
// 0x209102d3 (CFG-MSGOUT-RTCM_3X_TYPE1087_UART2)
// 0x209102d8 (CFG-MSGOUT-RTCM_3X_TYPE1127_UART2)
// 0x2091031a (CFG-MSGOUT-RTCM_3X_TYPE1097_UART2)
// 0x20910305 (CFG-MSGOUT-RTCM_3X_TYPE1230_UART2)
static const uint8_t setRTCMon[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x22, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0xbf, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0xce, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0xd3, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0xd8, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0x1a, 0x03, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0x05, 0x03, 0x91, 0x20,  0x28 };  // Change the last byte from 0x01 to 0x28 to send an RTCM message at 1/40 RATE_MEAS

// Disable RTCM message output on UART2
// UBX-CFG-VALSET message with the following key IDs and values of 0:
// 0x209102bf (CFG-MSGOUT-RTCM_3X_TYPE1005_UART2)
// 0x209102ce (CFG-MSGOUT-RTCM_3X_TYPE1077_UART2)
// 0x209102d3 (CFG-MSGOUT-RTCM_3X_TYPE1087_UART2)
// 0x209102d8 (CFG-MSGOUT-RTCM_3X_TYPE1127_UART2)
// 0x2091031a (CFG-MSGOUT-RTCM_3X_TYPE1097_UART2)
// 0x20910305 (CFG-MSGOUT-RTCM_3X_TYPE1230_UART2)
static const uint8_t setRTCMoff[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x22, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0xbf, 0x02, 0x91, 0x20,  0x00,
  0xce, 0x02, 0x91, 0x20,  0x00,
  0xd3, 0x02, 0x91, 0x20,  0x00,
  0xd8, 0x02, 0x91, 0x20,  0x00,
  0x1a, 0x03, 0x91, 0x20,  0x00,
  0x05, 0x03, 0x91, 0x20,  0x00 };

// Enable NMEA messages on UART2 for test purposes
// UBX-CFG-VALSET message with key ID of 0x10760002 (CFG-UART2OUTPROT-NMEA) and value of 1:
static const uint8_t setUART2nmea[] = {
  0xb5, 0x62,  0x06, 0x8a,  0x09, 0x00,
  0x00, 0x01, 0x00, 0x00,
  0x02, 0x00, 0x76, 0x10,  0x01 };

// Send message in u-blox UBX format
// Calculates and appends the two checksum bytes
// Doesn't add the 0xb5 and 0x62 sync chars (these need to be included at the start of the message)
// Extracts the message length from bytes 4 and 5
void sendUBX(const uint8_t *message) {
  int csum1 = 0; // Checksum bytes
  int csum2 = 0;
  int len = message[4] + (message[5] * 256) + 6; // Calculate the length
#ifdef DEBUG
    Serial.print("Sending UBX packet: 0x");
#endif
  for (int i=0; i<len; i++) { // For each byte in the message
    Serial1.write(message[i]); // Write the byte
#ifdef DEBUG // DEBUG: print the message byte in HEX format
    if (message[i] < 16) {Serial.print("0");}
    Serial.print(message[i], HEX);
    Serial.print(", 0x");
#endif
    if (i >= 2) { // Don't include the sync chars in the checksum
      csum1 = csum1 + message[i]; // Update the checksum bytes
      csum2 = csum2 + csum1;
    }
  }
  csum1 = csum1 & 0xff; // Limit checksums to 8-bits
  csum2 = csum2 & 0xff;
  Serial1.write((uint8_t)csum1); // Send the checksum bytes
  Serial1.write((uint8_t)csum2);
#ifdef DEBUG // DEBUG: print the checksum bytes in HEX format
  if (csum1 < 16) {Serial.print("0");}
  Serial.print((uint8_t)csum1, HEX);
  Serial.print(", 0x");
  if (csum2 < 16) {Serial.print("0");}
  Serial.println((uint8_t)csum2, HEX);
#endif
}

// ExtInt interrupt service routine
void ExtInt() {
  ExtIntTimer = millis() + 1000; // Set the timer value to 1000 milliseconds from now
}

// RTC alarm interrupt
// Must be kept as short as possible. Update the alarm time in the main loop, not here.
void alarmMatch()
{
  alarmFlag = true; // Set alarm flag
}

// TimerCounter3 functions to copy Serial1 receive data into SerialBuffer
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
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

// NeoPixel Functions
// WB2812B blue LED has the highest forward voltage and is slightly dim at 3.3V. The red and green values are adapted accordingly (222 instead of 255).

#ifdef NeoPixel

void LED_off() // Turn NeoPixel off
{
  pixels.setPixelColor(0,0,0,0);
  pixels.show();
}

void LED_dim_white() // Set LED to dim white
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(222,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_blue() // Set LED to dim blue
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0,0,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_green() // Set LED to dim green
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0,222,0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_cyan() // Set LED to dim cyan
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_white() // Set LED to white
{
  pixels.setPixelColor(0, pixels.Color(222,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_red() // Set LED to red
{
  pixels.setPixelColor(0, pixels.Color(222,0,0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_green() // Set LED to green
{
  pixels.setPixelColor(0, pixels.Color(0,222,0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_blue() // Set LED to blue
{
  pixels.setPixelColor(0, pixels.Color(0,0,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_cyan() // Set LED to cyan
{
  pixels.setPixelColor(0, pixels.Color(0,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_magenta() // Set LED to magenta
{
  pixels.setPixelColor(0, pixels.Color(222,0,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

#endif

void setup()
{
#ifdef NeoPixel
  // Initialise the NeoPixel
  pixels.begin(); // This initializes the NeoPixel library.
  delay(100); // Seems necessary to make the NeoPixel start reliably 
  pixels.setBrightness(LED_Brightness); // Initialize the LED brightness
  LED_off(); // Set NeoPixel off
#ifndef NoLED
  LED_dim_blue(); // Set NeoPixel to dim blue
#endif
#else
  // initialize digital pins RedLED and GreenLED as outputs.
  pinMode(RedLED, OUTPUT); // Red LED
  pinMode(GreenLED, OUTPUT); // Green LED
  digitalWrite(RedLED, LOW); // Turn Red LED off
  digitalWrite(GreenLED, LOW); // Turn Green LED off
#ifndef NoLED
  // flash red and green LEDs on reset
  for (int i=0; i <= 4; i++) {
    digitalWrite(RedLED, HIGH);
    delay(200);
    digitalWrite(RedLED, LOW);
    digitalWrite(GreenLED, HIGH);
    delay(200);
    digitalWrite(GreenLED, LOW);
  }
#endif
#endif

  // initialize modePin (A0) as an input for the Base/Rover mode select switch
  pinMode(modePin, INPUT_PULLUP);

  // initialize swPin (A1) as an input for the stop switch
  pinMode(swPin, INPUT_PULLUP);

  // initialise ExtIntPin (A2) as an input for the EVENT switch
  pinMode(ExtIntPin, INPUT_PULLUP);
  // Attach the interrupt service routine
  // Interrupt on falling edge of the ExtInt signal
  attachInterrupt(ExtIntPin, ExtInt, FALLING);
  ExtIntTimer = millis(); // Initialise the ExtInt LED timer

  // initialise SurveyInPin (A3) as an input for the SURVEY_IN switch
  pinMode(SurveyInPin, INPUT_PULLUP);

  delay(10000); // Allow 10 sec for user to open serial monitor (Comment this line if required)
  //while (!Serial); // OR Wait for user to run python script or open serial monitor (Comment this line as required)

  Serial.begin(115200);

  Serial.println("RAWX_Logger_F9P");
  Serial.println("Log GNSS RAWX data to SD card");
#ifndef NeoPixel
  Serial.println("Green LED = Initial GNSS Fix");
  Serial.println("Red LED Flash = SD Write");
#else
  Serial.println("Blue = Init");
  Serial.println("Dim Cyan = Waiting for GNSS Fix");
  Serial.println("Cyan = Checking GNSS Fix");
  Serial.println("Green flicker = SD Write");
  Serial.println("Magenta + Green flash = TIME fix in Survey_In mode");
  Serial.println("White = EVENT (ExtInt) detected");
#endif
  Serial.println("Continuous Red indicates a problem or that logging has been stopped");
  Serial.println("Initializing GNSS...");

#ifndef NoLED
#ifdef NeoPixel
  LED_blue(); // Set NeoPixel to blue
#endif
#endif

  // u-blox F9P Init
  // 38400 is the default baud rate for u-blox F9P
  GPS.begin(38400);
  // Change the ZED-F9P UART Baud rate
  sendUBX(setUART1BAUD); // Set ZED-F9P UART1 baud rate to 230400
  // Allow time for Baud rate change
  delay(1100);
  // Restart serial communications
  GPS.begin(230400); // Restart Serial1 at 230400 baud

  // Disable RAWX messages
  sendUBX(setRAWXoff);
  delay(100);
  
  // Enable NMEA messages GGA and RMC; disable the others
  sendUBX(setNMEAon);
  delay(100); // Wait

  // Set NMEA TALKERID to GP
  sendUBX(setTALKERID);
  delay(100);

  // Set Navigation/Measurement Rate to 1Hz
  sendUBX(setRATE_1Hz);
  delay(100);

  // Set UART2 Baud rate
  sendUBX(setUART2BAUD_115200);
  delay(100);

  // Check the modePin and set the navigation dynamic model
  if (digitalRead(modePin) == LOW) {
    Serial.println("BASE mode selected");
    sendUBX(setNAVstationary); // Set Static Navigation Mode (use this for the Base Logger)    
  }
  else {
    base_mode = false; // Clear base_mode flag
    Serial.println("ROVER mode selected");
    // Select one mode for the mobile Rover Logger
    //sendUBX(setNAVportable); // Set Portable Navigation Mode
    //sendUBX(setNAVpedestrian); // Set Pedestrian Navigation Mode
    //sendUBX(setNAVautomotive); // Set Automotive Navigation Mode
    //sendUBX(setNAVsea); // Set Sea Navigation Mode
    sendUBX(setNAVair1g); // Set Airborne <1G Navigation Mode
    //sendUBX(setNAVair2g); // Set Airborne <2G Navigation Mode
    //sendUBX(setNAVair4g); // Set Airborne <4G Navigation Mode
    //sendUBX(setNAVwrist); // Set Wrist Navigation Mode
  }
  delay(1100);
  
  while(Serial1.available()){Serial1.read();} // Flush RX buffer so we don't confuse Adafruit_GPS with UBX acknowledgements

  Serial.println("GNSS initialized!");

#ifndef NoLED
#ifndef NeoPixel
  // flash the red LED during SD initialisation
  digitalWrite(RedLED, HIGH);
#endif
#endif

  // Initialise SD card
  Serial.println("Initializing SD card...");
  // See if the SD card is present and can be initialized
  if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
    Serial.println("Panic!! SD Card Init failed, or not present!");
    Serial.println("Waiting for reset...");
#ifndef NoLED
#ifdef NeoPixel
    LED_red(); // Set NeoPixel to red
#endif
#endif
    // don't do anything more:
    while(1);
  }
  Serial.println("SD Card initialized!");

#ifndef NoLED
#ifdef NeoPixel
  LED_dim_cyan(); // Set NeoPixel to dim cyan now that the SD card is initialised
#else
  // turn red LED off
  digitalWrite(RedLED, LOW);
#endif
#endif

  Serial.println("Waiting for GNSS fix...");
}

void loop() // run over and over again
{
  switch(loop_step) {
    case init: {
      // read data from the GNSS
      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
      // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          break; // we can fail to parse a sentence in which case we should just wait for another
    
#ifdef DEBUG
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" Quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
          Serial.print("HDOP: "); Serial.println(GPS.HDOP);
        }
#endif
  
        // read battery voltage
        vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
#ifdef DEBUG
        Serial.print("Battery(V): ");
        Serial.println(vbat, 2);
#endif
      
        // turn green LED on to indicate GNSS fix
        // or set NeoPixel to cyan
        if (GPS.fix) {
#ifndef NoLED
#ifdef NeoPixel
          LED_cyan(); // Set NeoPixel to cyan
#else
          digitalWrite(GreenLED, HIGH);
#endif
#endif
          // increment valfix and cap at maxvalfix
          // don't do anything fancy in terms of decrementing valfix as we want to keep logging even if the fix is lost
          valfix += 1;
          if (valfix > maxvalfix) valfix = maxvalfix;
        }
        else {
#ifndef NoLED
#ifdef NeoPixel
          LED_dim_cyan(); // Set NeoPixel to dim cyan
#else
          digitalWrite(GreenLED, LOW); // Turn green LED off
#endif
#endif
        }
  
        if (valfix == maxvalfix) { // wait until we have enough valid fixes
          
          // Set and start the RTC
          alarmFlag = false; // Make sure alarm flag is clear
          rtc.begin(); // Start the RTC
          rtc.setTime(GPS.hour, GPS.minute, GPS.seconds); // Set the time
          rtc.setDate(GPS.day, GPS.month, GPS.year); // Set the date
          rtc.setAlarmSeconds(0); // Set RTC Alarm Seconds to zero
          uint8_t nextAlarmMin = ((GPS.minute+INTERVAL)/INTERVAL)*INTERVAL; // Calculate next alarm minutes
          nextAlarmMin = nextAlarmMin % 60; // Correct hour rollover
          rtc.setAlarmMinutes(nextAlarmMin); // Set RTC Alarm Minutes
          rtc.enableAlarm(rtc.MATCH_MMSS); // Alarm Match on minutes and seconds
          rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt

          // check if voltage is > LOWBAT(V), if not then don't try to log any data
          if (vbat < LOWBAT) {
            Serial.println("Low Battery!");
            break;
          }

          // Disable the GPGGA and GPRMC messages
          sendUBX(setNMEAoff);
          delay(100);

          // Set the RAWX measurement rate
          //sendUBX(setRATE_20Hz); // Set Navigation/Measurement Rate to 20 Hz
          //sendUBX(setRATE_10Hz); // Set Navigation/Measurement Rate to 10 Hz
          //sendUBX(setRATE_5Hz); // Set Navigation/Measurement Rate to 5 Hz
          sendUBX(setRATE_4Hz); // Set Navigation/Measurement Rate to 4 Hz
          //sendUBX(setRATE_2Hz); // Set Navigation/Measurement Rate to 2 Hz
          //sendUBX(setRATE_1Hz); // Set Navigation/Measurement Rate to 1 Hz
          
          delay(1100); // Wait

          // If we are in BASE mode, check the SURVEY_IN pin
          if (base_mode == true) {
            if (digitalRead(SurveyInPin) == LOW) {
              // We are in BASE mode and the SURVEY_IN pin is low so send the extra UBX messages:
              Serial.println("SURVEY_IN mode selected");
              survey_in_mode = true; // Set the survey_in_mode flag true
              sendUBX(setRTCMon); // Enable the RTCM messages on UART2
              delay(1100);
              sendUBX(setSurveyIn); // Enable SURVEY_IN mode
              delay(1100);
            }
          }
          
          while(Serial1.available()){Serial1.read();} // Flush RX buffer to clear UBX acknowledgements

          // Now that Serial1 should be idle and the buffer empty, start TC3 interrupts to copy all new data into SerialBuffer
          // Set the timer interval to 10 * 10 / 230400 = 0.000434 secs (10 bytes * 10 bits (1 start, 8 data, 1 stop) at 230400 baud)
          startTimerInterval(0.000434); 
          
          loop_step = start_rawx; // start rawx messages
        }
      }
    }
    break;

    // (Re)Start RAWX messages
    case start_rawx: {
      sendUBX(setRAWXon); // (Re)Start the RXM_RAWX, RXM_SFRBX and TIM_TM2 messages

      bufferPointer = 0; // (Re)initialise bufferPointer

      while (SerialBuffer.available() < 10) { ; } // Wait for one UBX acknowledgement's worth of data (10 bytes)
      
      for (int y=0;y<10;y++) { // Get ten bytes
        serBuffer[bufferPointer] = SerialBuffer.read_char(); // Add a character to serBuffer
        bufferPointer++; // Increment the pointer   
      }
      // Now check if these bytes were an acknowledgement
      if ((serBuffer[bufferPointer - 10] == 0xB5) and (serBuffer[bufferPointer - 9] == 0x62) and (serBuffer[bufferPointer - 8] == 0x05)) {
        // This must be an acknowledgement so simply ignore it and decrement bufferPointer by 10
        bufferPointer -= 10;
      }

      loop_step = open_file; // start logging rawx data
    }
    break;

    // Open the log file
    case open_file: {
      
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
      
      // Do the divides to convert date and time to char
      char secT = RTCseconds/10 + '0';
      char secU = RTCseconds%10 + '0';
      char minT = RTCminutes/10 + '0';
      char minU = RTCminutes%10 + '0';
      char hourT = RTChours/10 + '0';
      char hourU = RTChours%10 + '0';
      char dayT = RTCday/10 +'0';
      char dayU = RTCday%10 +'0';
      char monT = RTCmonth/10 +'0';
      char monU = RTCmonth%10 +'0';
      char yearT = RTCyear/10 +'0';
      char yearU = RTCyear%10 +'0';
  
      // filename is limited to 8.3 characters so use format: YYYYMMDD/b_HHMMSS.ubx or YYYYMMDD/r_HHMMSS.ubx
      rawx_filename[2] = yearT;
      rawx_filename[3] = yearU;
      rawx_filename[4] = monT;
      rawx_filename[5] = monU;
      rawx_filename[6] = dayT;
      rawx_filename[7] = dayU;
      if (base_mode == false) rawx_filename[9] = 'r';
      rawx_filename[11] = hourT;
      rawx_filename[12] = hourU;
      rawx_filename[13] = minT;
      rawx_filename[14] = minU;
      rawx_filename[15] = secT;
      rawx_filename[16] = secU;
      
      dirname[2] = yearT;
      dirname[3] = yearU;
      dirname[4] = monT;
      dirname[5] = monU;
      dirname[6] = dayT;
      dirname[7] = dayU;


      // flash red LED to indicate SD write (leave on if an error occurs)
      // or flash NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_green(); // Set the NeoPixel to green
#else
      LED_off(); // Turn NeoPixel off if NoLogLED
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off for NoLogLED
      digitalWrite(GreenLED, LOW); // Turn the green LED off for NoLogLED
#endif
#endif
#endif

      // try to create subdirectory (even if it exists already)
      sd.mkdir(dirname);
      
      // Open the rawx file for fast writing
      if (rawx_dataFile.open(rawx_filename, O_CREAT | O_WRITE | O_EXCL)) {
        Serial.print("Logging to ");
        Serial.println(rawx_filename);
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("Panic!! Error opening RAWX file!");
        Serial.println("Waiting for reset...");
#ifndef NoLED
#ifdef NeoPixel
      LED_red(); // Set the NeoPixel to red to indicate a problem
#else
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate a problem
#endif
#endif
        // don't do anything more:
        while(1);
      }

#ifdef DEBUG
      // Set the log file creation time
      if (!rawx_dataFile.timestamp(T_CREATE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file create timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_CREATE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif

      // Now that SD write is complete
      // Turn the Red LED off or set NeoPixel to dim green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_dim_green();
#endif
#else
      digitalWrite(RedLED, LOW); // turn red LED off
#endif
#endif

      bytes_written = 0; // Clear bytes_written

      ubx_state = looking_for_B5; // set ubx_state to expect B5
      ubx_length = 0; // set ubx_length to zero
          
      loop_step = write_file; // start logging rawx data
    }
    break;

    // Stuff bytes into serBuffer and write when we have reached SDpacket
    case write_file: {
      if (SerialBuffer.available()) {
        uint8_t c = SerialBuffer.read_char();
        serBuffer[bufferPointer] = c;
        bufferPointer++;
        if (bufferPointer == SDpacket) {
          bufferPointer = 0;
          // Flash the red LED to indicate an SD write
          // or flash the NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
          if (millis() < ExtIntTimer) {
            LED_white(); // Set the NeoPixel to white to indicate an ExtInt
          }
          else {
            LED_green(); // Set the NeoPixel to green
          }
#endif
#else
#ifndef NoLogLED
          digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
          numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
          //rawx_dataFile.sync(); // Sync the file system
          bytes_written += SDpacket;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
          if (millis() < ExtIntTimer) {
            LED_white(); // Set the NeoPixel to white to indicate an ExtInt
          }
          else {
            LED_dim_green(); // Set the NeoPixel to dim green
          }
#endif
#else
#ifndef NoLogLED
          digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#endif
#ifdef DEBUG
          if (numBytes != SDpacket) {
            Serial.print("SD write error! Write size was ");
            Serial.print(SDpacket);
            Serial.print(". ");
            Serial.print(numBytes);
            Serial.println(" were written.");
          }
#endif
//#ifdef DEBUG
//          Serial.print("SD Write: ");
//          Serial.print(SDpacket);
//          Serial.println(" Bytes");
//          Serial.print(bytes_written);
//          Serial.println(" Bytes written so far");
//#endif
        }
        // Process data bytes according to ubx_state:
        // Sync Char 1: 0xB5
        // Sync Char 2: 0x62
        // Class byte
        // ID byte
        // Length: two bytes, little endian
        // Payload: length bytes
        // Checksum: two bytes
        // Only allow a new file to be opened when a complete packet has been processed and ubx_state has returned to "looking_for_B5"
        // Or when a data error is detected (sync_lost)
        switch (ubx_state) {
          case (looking_for_B5): {
            if (c == 0xB5) { // Have we found Sync Char 1 (0xB5) when we were expecting one?
              ubx_state = looking_for_62; // Now look for Sync Char 2 (0x62)
            }
            else {
              Serial.println("Panic!! Was expecting Sync Char 0xB5 but did not receive one!");
              ubx_state = sync_lost;
            }
          }
          break;
          case (looking_for_62): {
            if (c == 0x62) { // Have we found Sync Char 2 (0x62) when we were expecting one?
              ubx_expected_checksum_A = 0; // Reset the expected checksum
              ubx_expected_checksum_B = 0;
              ubx_state = looking_for_class; // Now look for Class byte
            }
            else {
              Serial.println("Panic!! Was expecting Sync Char 0x62 but did not receive one!");
              ubx_state = sync_lost;
            }
          }
          break;
          // RXM_RAWX is class 0x02 ID 0x15
          // RXM_SFRBF is class 0x02 ID 0x13
          // TIM_TM2 is class 0x0d ID 0x03
          // NAV_POSLLH is class 0x01 ID 0x02
          // NAV_PVT is class 0x01 ID 0x07
          // NAV-STATUS is class 0x01 ID 0x03
          case (looking_for_class): {
            ubx_class = c;
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = looking_for_ID; // Now look for ID byte
#ifdef DEBUG
            // Class syntax checking
            if ((ubx_class != 0x02) and (ubx_class != 0x0d) and (ubx_class != 0x01)) {
              Serial.println("Panic!! Was expecting Class of 0x02 or 0x0d or 0x01 but did not receive one!");
              ubx_state = sync_lost;
            }
#endif
          }
          break;
          case (looking_for_ID): {
            ubx_ID = c;
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = looking_for_length_LSB; // Now look for length LSB
#ifdef DEBUG
            // ID syntax checking
            if ((ubx_class == 0x02) and ((ubx_ID != 0x15) and (ubx_ID != 0x13))) {
              Serial.println("Panic!! Was expecting ID of 0x15 or 0x13 but did not receive one!");
              ubx_state = sync_lost;
            }
            else if ((ubx_class == 0x0d) and (ubx_ID != 0x03)) {
              Serial.println("Panic!! Was expecting ID of 0x03 but did not receive one!");
              ubx_state = sync_lost;
            }
            else if ((ubx_class == 0x01) and ((ubx_ID != 0x02) and (ubx_ID != 0x07) and (ubx_ID != 0x03))) {
              Serial.println("Panic!! Was expecting ID of 0x02 or 0x07 or 0x03 but did not receive one!");
              ubx_state = sync_lost;
            }
#endif
          }
          break;
          case (looking_for_length_LSB): {
            ubx_length = c; // Store the length LSB
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = looking_for_length_MSB; // Now look for length MSB
          }
          break;
          case (looking_for_length_MSB): {
            ubx_length = ubx_length + (c * 256); // Add the length MSB
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = processing_payload; // Now look for payload bytes (length: ubx_length)
          }
          break;
          case (processing_payload): {
#ifdef DEBUG
            // If this is a NAV_PVT message, check the flags byte (byte offset 21) and report the carrSoln
            if ((ubx_class == 0x01) and (ubx_ID == 0x07)) { // Is this a NAV_PVT message (class 0x01 ID 0x07)?
              if (ubx_length == 71) { // Is this byte offset 21? (ubx_length will be 92 for byte offset 0, so will be 71 for byte offset 21)
                Serial.print("NAV_PVT carrSoln: ");
                if ((c & 0xc0) == 0x00) {
                  Serial.println("none");
                }
                else if ((c & 0xc0) == 0x40) {
                  Serial.println("floating");
                }
                else if ((c & 0xc0) == 0x80) {
                  Serial.println("fixed");
                }
              }
            }
#endif
            // If this is a NAV_STATUS message, check the gpsFix byte (byte offset 4) and flash the green LED (or make the NeoPixel magenta) if the fix is TIME
            if ((ubx_class == 0x01) and (ubx_ID == 0x03)) { // Is this a NAV_STATUS message (class 0x01 ID 0x03)?
              if (ubx_length == 12) { // Is this byte offset 4? (ubx_length will be 16 for byte offset 0, so will be 12 for byte offset 4)
#ifdef DEBUG
                Serial.print("NAV_STATUS gpsFix: ");
                if (c == 0x00) {
                  Serial.println("no fix");
                }
                else if (c == 0x01) {
                  Serial.println("dead reckoning");
                }
                else if (c == 0x02) {
                  Serial.println("2D-fix");
                }
                else if (c == 0x03) {
                  Serial.println("3D-fix");
                }
                else if (c == 0x04) {
                  Serial.println("GPS + dead reckoning");
                }
                else if (c == 0x05) {
                  Serial.println("time");
                }
                else {
                  Serial.println("reserved");
                }
#endif
                if (c == 0x05) { // Have we got a TIME fix?
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
                  if (millis() > ExtIntTimer) {
                    LED_magenta(); // Set the NeoPixel to magenta only if we are not already showing white
                  }
#endif
#else
#ifndef NoLogLED
                  digitalWrite(GreenLED, !digitalRead(GreenLED)); // Toggle the green LED
#endif
#endif
#endif            
                }
                else {
#ifndef NoLED
#ifndef NeoPixel
#ifndef NoLogLED
                  digitalWrite(GreenLED, HIGH); // If the fix is not TIME, leave the green LED on
#endif
#endif
#endif
                }
              }
            }
            ubx_length = ubx_length - 1; // Decrement length by one
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            if (ubx_length == 0) {
              ubx_expected_checksum_A = ubx_expected_checksum_A & 0xff; // Limit checksums to 8-bits
              ubx_expected_checksum_B = ubx_expected_checksum_B & 0xff;
              ubx_state = looking_for_checksum_A; // If we have received length payload bytes, look for checksum bytes
            }
          }
          break;
          case (looking_for_checksum_A): {
            ubx_checksum_A = c;
            ubx_state = looking_for_checksum_B;
          }
          break;
          case (looking_for_checksum_B): {
            ubx_checksum_B = c;
            ubx_state = looking_for_B5; // All bytes received so go back to looking for a new Sync Char 1 unless there is a checksum error
            if ((ubx_expected_checksum_A != ubx_checksum_A) or (ubx_expected_checksum_B != ubx_checksum_B)) {
              Serial.println("Panic!! Checksum error!");
              ubx_state = sync_lost;
            }
          }
          break;
        }
      }
      else {
        // read battery voltage
        vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
      }
      // Check if the stop button has been pressed or battery is low
      // or if there has been an RTC alarm and it is time to open a new file
      if (digitalRead(swPin) == LOW) stop_pressed = true;
      if ((stop_pressed == true) or (vbat < LOWBAT)) {
        loop_step = close_file; // now close the file
        break;
      }
      else if ((alarmFlag == true) and (ubx_state == looking_for_B5)) {
        loop_step = new_file; // now close the file and open a new one
        break;
      }
      else if (ubx_state == sync_lost) {
        loop_step = restart_file; // Sync has been lost so stop RAWX messages and open a new file before restarting RAWX
      }
    }
    break;

    // Close the current log file and open a new one without stopping RAWX messages
    case new_file: {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
//        Serial.print("Final SD Write: ");
//        Serial.print(bufferPointer);
//        Serial.println(" Bytes");
//        Serial.print(bytes_written);
//        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      // An RTC alarm was detected, so set the RTC alarm time to the next INTERVAL and loop back to open_file.
      // We only receive an RTC alarm on a minute mark, so it doesn't matter that the RTC seconds will have moved on at this point.
      alarmFlag = false; // Clear the RTC alarm flag
      uint8_t rtc_mins = rtc.getMinutes(); // Read the RTC minutes
      rtc_mins = rtc_mins + INTERVAL; // Add the INTERVAL to the RTC minutes
      rtc_mins = rtc_mins % 60; // Correct for hour rollover
      rtc.setAlarmMinutes(rtc_mins); // Set next alarm time (minutes only - hours are ignored)
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_cyan(); // Set the NeoPixel to cyan
#else
      LED_off(); // Turn NeoPixel off if NoLogLED
#endif
#endif
#endif
      loop_step = open_file; // loop round again and open a new file
      bytes_written = 0; // Clear bytes_written
    }
    break;

    // Disable RAWX messages, save any residual data and close the file for the last time
    case close_file: {
      sendUBX(setRAWXoff); // Disable RAWX messages
      int waitcount = 0;
      // leave 10 bytes in the serial buffer as this _should_ be the message acknowledgement
      while (waitcount < dwell) { // Wait for residual data
        while (SerialBuffer.available() > 10) { // Leave 10 bytes in the serial buffer
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
          bufferPointer++;
          if (bufferPointer == SDpacket) { // Write a full packet
            bufferPointer = 0;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
            digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
            numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
            //rawx_dataFile.sync(); // Sync the file system
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
            digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
            bytes_written += SDpacket;
#ifdef DEBUG
            if (numBytes != SDpacket) {
              Serial.print("SD write error! Write size was ");
              Serial.print(SDpacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
//            Serial.print("SD Write: ");
//            Serial.print(SDpacket);
//            Serial.println(" Bytes");
//            Serial.print(bytes_written);
//            Serial.println(" Bytes written so far");
#endif
          }
        }
        waitcount++;
        delay(1);
      }
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Penultimate/Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
        Serial.print(bytes_written);
        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // We now have exactly 10 bytes left in the buffer. Let's check if they contain an acknowledgement or residual data.
      // If they contain residual data, save it to file. This means we have probably already saved acknowledgement(s)
      // to file and there's now very little we can do about that except hope that RTKLib knows to ignore them!
      for (int y=0;y<10;y++) { // Add ten bytes
        serBuffer[bufferPointer] = SerialBuffer.read_char(); // Add a character to serBuffer
        bufferPointer++; // Increment the pointer   
      }
      // Now check if these bytes were an acknowledgement
      if ((serBuffer[bufferPointer - 10] == 0xB5) and (serBuffer[bufferPointer - 9] == 0x62) and (serBuffer[bufferPointer - 8] == 0x05)) {
        // This must be an acknowledgement so simply ignore it and decrement bufferPointer by 10
        bufferPointer -= 10;
      }
      // If the last 10 bytes did contain any data, write it to file now
      if (bufferPointer > 0) {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      // Either the battery is low or the user pressed the stop button:
      if (stop_pressed == true) {
        // Stop switch was pressed so just wait for a reset
#ifndef NoLED
#ifdef NeoPixel
        LED_red(); // Set the NeoPixel to red
#else
        digitalWrite(RedLED, HIGH); // Turn the red LED on
#endif
#endif
        Serial.println("Waiting for reset...");
        while(1); // Wait for reset
      }
      else {
        // Low battery was detected so wait for the battery to recover
        Serial.println("Battery must be low - waiting for it to recover...");
#ifndef NoLED
#ifdef NeoPixel
        LED_red(); // Set the NeoPixel to red
#else
        digitalWrite(RedLED, HIGH); // Turn the red LED on
#endif
#endif
        // Check the battery voltage. Make sure it has been OK for at least 5 seconds before continuing
        int high_for = 0;
        while (high_for < 500) {
          // read battery voltage
          vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
          if (vbat < LOWBAT) {
            high_for = 0; // If battery voltage is low, reset the count
          }
          else {
            high_for++; // Increase the count
          }
          delay(10); // Wait 10msec
        }
        // Now loop round again and restart rawx messages before opening a new file
#ifndef NoLED
#ifdef NeoPixel
        LED_cyan(); // Set the NeoPixel to cyan
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off
#endif
#endif
        loop_step = start_rawx;
      }
    }
    break;

    // RAWX data lost sync so disable RAWX messages, save any residual data, close the file, open another and restart RAWX messages
    // Don't update the next RTC alarm - leave it as it is
    case restart_file: {
      sendUBX(setRAWXoff); // Disable RAWX messages
      int waitcount = 0;
      // leave 10 bytes in the serial buffer as this _should_ be the message acknowledgement
      while (waitcount < dwell) { // Wait for residual data
        while (SerialBuffer.available() > 10) { // Leave 10 bytes in the serial buffer
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
          bufferPointer++;
          if (bufferPointer == SDpacket) { // Write a full packet
            bufferPointer = 0;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
            digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
            numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
            //rawx_dataFile.sync(); // Sync the file system
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
            digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
            bytes_written += SDpacket;
#ifdef DEBUG
            if (numBytes != SDpacket) {
              Serial.print("SD write error! Write size was ");
              Serial.print(SDpacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
//            Serial.print("SD Write: ");
//            Serial.print(SDpacket);
//            Serial.println(" Bytes");
//            Serial.print(bytes_written);
//            Serial.println(" Bytes written so far");
#endif
          }
        }
        waitcount++;
        delay(1);
      }
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Penultimate/Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
        Serial.print(bytes_written);
        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // We now have exactly 10 bytes left in the buffer. Let's check if they contain an acknowledgement or residual data.
      // If they contain residual data, save it to file. This means we have probably already saved acknowledgement(s)
      // to file and there's now very little we can do about that except hope that RTKLib knows to ignore them!
      for (int y=0;y<10;y++) { // Add ten bytes
        serBuffer[bufferPointer] = SerialBuffer.read_char(); // Add a character to serBuffer
        bufferPointer++; // Increment the pointer   
      }
      // Now check if these bytes were an acknowledgement
      if ((serBuffer[bufferPointer - 10] == 0xB5) and (serBuffer[bufferPointer - 9] == 0x62) and (serBuffer[bufferPointer - 8] == 0x05)) {
        // This must be an acknowledgement so simply ignore it and decrement bufferPointer by 10
        bufferPointer -= 10;
      }
      // If the last 10 bytes did contain any data, write it to file now
      if (bufferPointer > 0) {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_dim_green(); // Set the NeoPixel to dim green
#endif
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      loop_step = start_rawx; // loop round again and restart rawx messages before opening a new file
    }
    break;  
  }
}
