// Copyright 2016 sillyfrog
// Copyright 2017 sillyfrog, crankyoldgit
// Copyright 2018-2022 crankyoldgit
// Copyright 2019 pasna (IRDaikin160 class / Daikin176 class)

/// @file
/// @brief Support for Daikin A/C protocols.
/// @see Daikin http://harizanov.com/2012/02/control-daikin-air-conditioner-over-the-internet/
/// @see Daikin https://github.com/mharizanov/Daikin-AC-remote-control-over-the-Internet/tree/master/IRremote
/// @see Daikin http://rdlab.cdmt.vn/project-2013/daikin-ir-protocol
/// @see Daikin https://github.com/blafois/Daikin-IR-Reverse
/// @see Daikin128 https://github.com/crankyoldgit/IRremoteESP8266/issues/827
/// @see Daikin152 https://github.com/crankyoldgit/IRremoteESP8266/issues/873
/// @see Daikin152 https://github.com/ToniA/arduino-heatpumpir/blob/master/DaikinHeatpumpARC480A14IR.cpp
/// @see Daikin152 https://github.com/ToniA/arduino-heatpumpir/blob/master/DaikinHeatpumpARC480A14IR.h
/// @see Daikin160 https://github.com/crankyoldgit/IRremoteESP8266/issues/731
/// @see Daikin2 https://docs.google.com/spreadsheets/d/1f8EGfIbBUo2B-CzUFdrgKQprWakoYNKM80IKZN4KXQE/edit#gid=236366525&range=B25:D32
/// @see Daikin2 https://github.com/crankyoldgit/IRremoteESP8266/issues/582
/// @see Daikin2 https://github.com/crankyoldgit/IRremoteESP8266/issues/1535
/// @see Daikin2 https://www.daikin.co.nz/sites/default/files/daikin-split-system-US7-FTXZ25-50NV1B.pdf
/// @see Daikin216 https://github.com/crankyoldgit/IRremoteESP8266/issues/689
/// @see Daikin216 https://github.com/danny-source/Arduino_DY_IRDaikin
/// @see Daikin64 https://github.com/crankyoldgit/IRremoteESP8266/issues/1064
/// @see Daikin200 https://github.com/crankyoldgit/IRremoteESP8266/issues/1802

#ifndef _IR_DAIKIN_HPP
#define _IR_DAIKIN_HPP


#define LOCAL_DEBUG

const uint16_t kDaikinStateLength = 35;
const uint16_t kDaikinBits = kDaikinStateLength * 8;
const uint16_t kDaikinStateLengthShort = kDaikinStateLength - 8;
const uint16_t kDaikinBitsShort = kDaikinStateLengthShort * 8;
const uint16_t kDaikinDefaultRepeat = 0;
const uint16_t kMarkExcess = 50;

#define DAIKIN_KHZ      38


/// Native representation of a Daikin A/C message.
union DaikinESPProtocol{
  uint8_t raw[kDaikinStateLength];  ///< The state of the IR remote.
  struct {
    // Byte 0~5
    uint64_t          :48;
    // Byte 6
    uint64_t          :4;
    uint64_t Comfort  :1;
    uint64_t          :3;
    // Byte 7
    uint64_t Sum1     :8;  // checksum of the first part

    // Byte 8~12
    uint64_t              :40;
    // Byte 13~14
    uint64_t CurrentTime  :11;  // Current time, mins past midnight
    uint64_t CurrentDay   :3;  // Day of the week (SUN=1, MON=2, ..., SAT=7)
    uint64_t              :2;
    // Byte 15
    uint64_t Sum2         :8;  // checksum of the second part

    // Byte 16~20
    uint64_t          :40;
    // Byte 21
    uint64_t Power    :1;
    uint64_t OnTimer  :1;
    uint64_t OffTimer :1;
    uint64_t          :1;  // always 1
    uint64_t Mode     :3;
    uint64_t          :1;
    // Byte 22
    uint64_t          :1;
    uint64_t Temp     :7;  // Temp should be between 10 - 32
    // Byte 23
    uint64_t          :8;

    // Byte 24
    uint64_t SwingV   :4;  // 0000 =  off, 1111 = on
    uint64_t Fan      :4;
    // Byte 25
    uint64_t SwingH   :4;  // 0000 =  off, 1111 = on
    uint64_t          :4;
    // Byte 26~28
    uint64_t OnTime   :12;  // timer mins past midnight
    uint64_t OffTime  :12;  // timer mins past midnight
    // Byte 29
    uint64_t Powerful :1;
    uint64_t          :4;
    uint64_t Quiet    :1;
    uint64_t          :2;
    // Byte 30~31
    uint64_t          :0;

    // Byte 32
    uint8_t             :1;
    uint8_t Sensor      :1;
    uint8_t Econo       :1;
    uint8_t             :4;
    uint8_t WeeklyTimer :1;
    // Byte 33
    uint8_t       :1;
    uint8_t Mold  :1;
    uint8_t       :6;
    // Byte 34
    uint8_t Sum3  :8;  // checksum of the third part
  };
};


// Constants
const uint16_t kHeader = 2;        // Usual nr. of header entries.
const uint16_t kFooter = 2;        // Usual nr. of footer (stop bits) entries.


const uint8_t kDaikinAuto = 0b000;  // temp 25
const uint8_t kDaikinDry =  0b010;  // temp 0xc0 = 96 degrees c
const uint8_t kDaikinCool = 0b011;
const uint8_t kDaikinHeat = 0b100;  // temp 23
const uint8_t kDaikinFan =  0b110;  // temp not shown, but 25
const uint8_t kDaikinMinTemp = 10;  // Celsius
const uint8_t kDaikinMaxTemp = 32;  // Celsius
const uint8_t kDaikinFanMin = 1;
const uint8_t kDaikinFanMed = 3;
const uint8_t kDaikinFanMax = 5;
const uint8_t kDaikinFanAuto = 0b1010;  // 10 / 0xA
const uint8_t kDaikinFanQuiet = 0b1011;  // 11 / 0xB
const uint8_t kDaikinSwingOn =  0b1111;
const uint8_t kDaikinSwingOff = 0b0000;
const uint16_t kDaikinHeaderLength = 5;
const uint8_t kDaikinSections = 3;
const uint8_t kDaikinSection1Length = 8;
const uint8_t kDaikinSection2Length = 8;
const uint8_t kDaikinSection3Length =
    kDaikinStateLength - kDaikinSection1Length - kDaikinSection2Length;
const uint8_t kDaikinByteChecksum1 = 7;
const uint8_t kDaikinByteChecksum2 = 15;
// const uint8_t kDaikinBitEye = 0b10000000;
const uint16_t kDaikinUnusedTime = 0x600;
const uint8_t kDaikinTolerance = 35;
const uint16_t kDaikinMarkExcess = kMarkExcess;
const uint16_t kDaikinHdrMark = 3650;   // kDaikinBitMark * 8
const uint16_t kDaikinHdrSpace = 1623;  // kDaikinBitMark * 4
const uint16_t kDaikinBitMark = 428;
const uint16_t kDaikinZeroSpace = 428;
const uint16_t kDaikinOneSpace = 1280;
const uint16_t kDaikinGap = 29000;
// Note bits in each octet swapped so can be sent as a single value
const uint64_t kDaikinFirstHeader64 =
    0b1101011100000000000000001100010100000000001001111101101000010001;


// Legacy defines.
#define DAIKIN_COOL kDaikinCool
#define DAIKIN_HEAT kDaikinHeat
#define DAIKIN_FAN kDaikinFan
#define DAIKIN_AUTO kDaikinAuto
#define DAIKIN_DRY kDaikinDry
#define DAIKIN_MIN_TEMP kDaikinMinTemp
#define DAIKIN_MAX_TEMP kDaikinMaxTemp
#define DAIKIN_FAN_MIN kDaikinFanMin
#define DAIKIN_FAN_MAX kDaikinFanMax
#define DAIKIN_FAN_AUTO kDaikinFanAuto
#define DAIKIN_FAN_QUIET kDaikinFanQuiet

/// Class for handling detailed Daikin 280-bit A/C messages.
class IRDaikinESP {
 public:
  explicit IRDaikinESP(IRrecv *rcv, IRsend *snd);

#if SEND_DAIKIN
  void send(const uint16_t repeat = kDaikinDefaultRepeat);
  /// Run the calibration to calculate uSec timing offsets for this platform.
  /// @return The uSec timing offset needed per modulation of the IR Led.
  /// @note This will produce a 65ms IR signal pulse at 38kHz.
  ///   Only ever needs to be run once per object instantiation, if at all.
  int8_t calibrate(void) { return _irsend.calibrate(); }
#endif
  bool decode();
  bool decodeDaikin();
  void on(void);
  void off(void);
  void setPower(const bool on);
  bool getPower(void) const;
  void setTemp(const uint8_t temp);
  uint8_t getTemp(void) const;
  void setFan(const uint8_t fan);
  uint8_t getFan(void) const;
  void setMode(const uint8_t mode);
  uint8_t getMode(void) const;
  void setSwingVertical(const bool on);
  bool getSwingVertical(void) const;
  void setSwingHorizontal(const bool on);
  bool getSwingHorizontal(void) const;
  bool getQuiet(void) const;
  void setQuiet(const bool on);
  bool getPowerful(void) const;
  void setPowerful(const bool on);
  void setSensor(const bool on);
  bool getSensor(void) const;
  void setEcono(const bool on);
  bool getEcono(void) const;
  void setMold(const bool on);
  bool getMold(void) const;
  void setComfort(const bool on);
  bool getComfort(void) const;
  void enableOnTimer(const uint16_t starttime);
  void disableOnTimer(void);
  uint16_t getOnTime(void) const;
  bool getOnTimerEnabled(void) const;
  void enableOffTimer(const uint16_t endtime);
  void disableOffTimer(void);
  uint16_t getOffTime(void) const;
  bool getOffTimerEnabled(void) const;
  void setCurrentTime(const uint16_t mins_since_midnight);
  uint16_t getCurrentTime(void) const;
  void setCurrentDay(const uint8_t day_of_week);
  uint8_t getCurrentDay(void) const;
  void setWeeklyTimerEnable(const bool on);
  bool getWeeklyTimerEnable(void) const;
  uint8_t* getRaw(void);
  void setRaw(const uint8_t new_code[],
              const uint16_t length = kDaikinStateLength);
  static bool validChecksum(uint8_t state[],
                            const uint16_t length = kDaikinStateLength);
#if 0
  static uint8_t convertMode(const stdAc::opmode_t mode);
  static uint8_t convertFan(const stdAc::fanspeed_t speed);
  static stdAc::opmode_t toCommonMode(const uint8_t mode);
  static stdAc::fanspeed_t toCommonFanSpeed(const uint8_t speed);
  stdAc::state_t toCommon(void) const;
#endif
  String toString(void) const;
#ifndef UNIT_TEST

 private:
  IRsend *_irsend;  ///< instance of the IR send class
  IRrecv *_irrecv;
#else
  /// @cond IGNORE
  IRsendTest _irsend;  ///< instance of the testing IR send class
  /// @endcond
#endif
  // # of bytes per command
  DaikinESPProtocol _;
  void stateReset(void);
  void checksum(void);
};


struct PulseDistanceWidthProtocolConstants DaikinProtocolConstants = { (decode_type_t)0, DAIKIN_KHZ, kDaikinHdrMark, kDaikinHdrSpace,
kDaikinBitMark, kDaikinOneSpace, kDaikinBitMark, kDaikinZeroSpace, PROTOCOL_IS_LSB_FIRST,
        (kDaikinDefaultRepeat / MICROS_IN_ONE_MILLI), NULL };


/// Convert a uint64_t (unsigned long long) to a string.
/// Arduino String/toInt/Serial.print() can't handle printing 64 bit values.
/// @param[in] input The value to print
/// @param[in] base The output base.
/// @returns A String representation of the integer.
/// @note Based on Arduino's Print::printNumber()
String uint64ToString(uint64_t input, uint8_t base) {
  String result = "";
  // prevent issues if called with base <= 1
  if (base < 2) base = 10;
  // Check we have a base that we can actually print.
  // i.e. [0-9A-Z] == 36
  if (base > 36) base = 10;

  // Reserve some string space to reduce fragmentation.
  // 16 bytes should store a uint64 in hex text which is the likely worst case.
  // 64 bytes would be the worst case (base 2).
  result.reserve(16);

  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

/// Convert a uint64_t (unsigned long long) to a string.
/// Arduino String/toInt/Serial.print() can't handle printing 64 bit values.
/// @param[in] input The value to print
/// @returns A String representation of the integer.
/// @note Based on Arduino's Print::printNumber()
String uint64ToString(uint64_t input) {
  return uint64ToString(input, 10);
}

  /// Create a String with a colon separated "label: value" pair suitable for
  /// Humans.
  /// @param[in] value The value to come after the label.
  /// @param[in] label The label to precede the value.
  /// @param[in] precomma Should the output string start with ", " or not?
  /// @return The resulting String.
  String addLabeledString(const String value, const String label,
                          const bool precomma = false) {
    String result = "";
    // ", " + ": " = 4 chars
    result.reserve(4 + value.length() + label.length());
    if (precomma) result += ", ";
    result += label;
    result += ": ";
    return result + value;
  }

  /// Create a String with a colon separated flag suitable for Humans.
  /// e.g. "Power: On"
  /// @param[in] value The value to come after the label.
  /// @param[in] label The label to precede the value.
  /// @param[in] precomma Should the output string start with ", " or not?
  /// @return The resulting String.
  String addBoolToString(const bool value, const String label,
                         const bool precomma = false) {
    return addLabeledString(value ? "On" : "Off", label, precomma);
  }

  /// Create a String with a colon separated labeled Integer suitable for
  /// Humans.
  /// e.g. "Foo: 23"
  /// @param[in] value The value to come after the label.
  /// @param[in] label The label to precede the value.
  /// @param[in] precomma Should the output string start with ", " or not?
  /// @return The resulting String.
  String addIntToString(const uint16_t value, const String label,
                        const bool precomma = false) {
    return addLabeledString(uint64ToString(value), label, precomma);
  }


    /// Create a String of the 3-letter day of the week from a numerical day of
  /// the week. e.g. "Day: 1 (Mon)"
  /// @param[in] day_of_week A numerical version of the sequential day of the
  ///  week. e.g. Saturday = 7 etc.
  /// @param[in] offset Days to offset by.
  ///  e.g. For different day starting the week.
  /// @param[in] precomma Should the output string start with ", " or not?
  /// @return The resulting String.
  String addDayToString(const uint8_t day_of_week, const int8_t offset,
                        const bool precomma = false) {
    String result = "";
    result.reserve(19);  // ", Day: N (UNKNOWN)"
    return result + addIntToString(day_of_week, "Day", precomma);
  }

  /// Create a String of human output for the given operating mode.
  /// e.g. "Mode: 1 (Cool)"
  /// @param[in] mode The operating mode to display.
  /// @param[in] automatic The numeric value for Auto mode.
  /// @param[in] cool The numeric value for Cool mode.
  /// @param[in] heat The numeric value for Heat mode.
  /// @param[in] dry The numeric value for Dry mode.
  /// @param[in] fan The numeric value for Fan mode.
  /// @return The resulting String.
  String addModeToString(const uint8_t mode, const uint8_t automatic,
                         const uint8_t cool, const uint8_t heat,
                         const uint8_t dry, const uint8_t fan) {
    String result = "";
    result.reserve(22);  // ", Mode: NNN (UNKNOWN)"
    result += addIntToString(mode, "Mode");
    result += " (";
    if (mode == automatic) result += "Auto";
    else if (mode == cool) result += "Cool";
    else if (mode == heat) result += "Heat";
    else if (mode == dry)  result += "Dry";
    else if (mode == fan)  result += "Fan";
    else
      result += "UNKNOWN";
    return result + ')';
  }

  /// Create a String of human output for a given temperature.
  /// e.g. "Temp: 25C"
  /// @param[in] degrees The temperature in degrees.
  /// @param[in] celsius Is the temp Celsius or Fahrenheit.
  ///  true is C, false is F
  /// @param[in] precomma Should the output string start with ", " or not?
  /// @param[in] isSensorTemp Is the value a room (ambient) temp. or target?
  /// @return The resulting String.
  String addTempToString(const uint16_t degrees, const bool celsius = true,
                         const bool precomma=false, const bool isSensorTemp=false) {
    String result = addIntToString(degrees, (isSensorTemp)?
                                   "Temp" : "TTemp", precomma);
    result += celsius ? 'C' : 'F';
    return result;
  }

  /// Create a String of human output for the given fan speed.
  /// e.g. "Fan: 0 (Auto)"
  /// @param[in] speed The numeric speed of the fan to display.
  /// @param[in] high The numeric value for High speed. (second highest)
  /// @param[in] low The numeric value for Low speed.
  /// @param[in] automatic The numeric value for Auto speed.
  /// @param[in] quiet The numeric value for Quiet speed.
  /// @param[in] medium The numeric value for Medium speed.
  /// @param[in] maximum The numeric value for Highest speed. (if > high)
  /// @param[in] medium_high The numeric value for third-highest speed.
  ///                        (if > medium)
  /// @return The resulting String.
  String addFanToString(const uint8_t speed, const uint8_t high,
                        const uint8_t low, const uint8_t automatic,
                        const uint8_t quiet, const uint8_t medium,
                        const uint8_t maximum, const uint8_t medium_high) {
    String result = "";
    result.reserve(21);  // ", Fan: NNN (UNKNOWN)"
    result += addIntToString(speed, "Fan");
    result += " (";
    if (speed == high)              result += "High";
    else if (speed == low)          result += "Low";
    else if (speed == automatic)    result += "Auto";
    else if (speed == quiet)        result += "Quiet";
    else if (speed == medium)       result += "Medium";
    else if (speed == maximum)      result += "Max";
    else if (speed == medium_high)  result += "MedHigh";
    else
      result += "UNKNOWN";
    return result + ')';
  }

  /// Convert a nr. of minutes into a 24h clock format Human-readable string.
  /// e.g. "23:59"
  /// @param[in] mins Nr. of Minutes.
  /// @return A human readable string.
  String minsToString(const uint16_t mins) {
    String result = "";
    result.reserve(5);  // 23:59 is the typical worst case.
    if (mins / 60 < 10) result += '0';  // Zero pad the hours
    result += uint64ToString(mins / 60) + ":";
    if (mins % 60 < 10) result += '0';  // Zero pad the minutes.
    result += uint64ToString(mins % 60);
    return result;
  }

  /// Sum all the bytes together in an integer.
  /// @param[in] data The integer to be summed.
  /// @param[in] count The number of bytes to sum. Starts from LSB. Max of 8.
  /// @param[in] init Starting value of the calculation to use. (Default is 0)
  /// @param[in] byteonly true, the result is 8 bits. false, it's 16 bits.
  /// @return The 8/16-bit calculated result of all the bytes and init value.
  uint16_t sumBytes(const uint64_t data, const uint8_t count,
                    const uint8_t init, const bool byteonly) {
    uint16_t sum = init;
    uint64_t copy = data;
    const uint8_t nrofbytes = (count < 8) ? count : (64 / 8);
    for (uint8_t i = 0; i < nrofbytes; i++, copy >>= 8) sum += (copy & 0xFF);
    return byteonly ? sum & 0xFF : sum;
  }

/// Sum all the bytes of an array and return the least significant 8-bits of
/// the result.
/// @param[in] start A ptr to the start of the byte array to calculate over.
/// @param[in] length How many bytes to use in the calculation.
/// @param[in] init Starting value of the calculation to use. (Default is 0)
/// @return The 8-bit calculated result of all the bytes and init value.
uint8_t sumBytes(const uint8_t * const start, const uint16_t length,
                 const uint8_t init = 0) {
  uint8_t checksum = init;
  const uint8_t *ptr;
  for (ptr = start; ptr - start < length; ptr++) checksum += *ptr;
  return checksum;
}


#undef SEND_DAIKIN
#if SEND_DAIKIN
/// Send a Daikin 280-bit A/C formatted message.
/// Status: STABLE
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
/// @see https://github.com/mharizanov/Daikin-AC-remote-control-over-the-Internet/tree/master/IRremote
/// @see https://github.com/blafois/Daikin-IR-Reverse
void IRsend::sendDaikin(const unsigned char data[], const uint16_t nbytes,
                        const uint16_t repeat) {
  if (nbytes < kDaikinStateLengthShort)
    return;  // Not enough bytes to send a proper message.

  for (uint16_t r = 0; r <= repeat; r++) {
    uint16_t offset = 0;
    // Send the header, 0b00000
    sendGeneric(0, 0,  // No header for the header
                kDaikinBitMark, kDaikinOneSpace, kDaikinBitMark,
                kDaikinZeroSpace, kDaikinBitMark, kDaikinZeroSpace + kDaikinGap,
                (uint64_t)0b00000, kDaikinHeaderLength, 38, false, 0, 50);
    // Data #1
    if (nbytes < kDaikinStateLength) {  // Are we using the legacy size?
      // Do this as a constant to save RAM and keep in flash memory
      sendGeneric(kDaikinHdrMark, kDaikinHdrSpace, kDaikinBitMark,
                  kDaikinOneSpace, kDaikinBitMark, kDaikinZeroSpace,
                  kDaikinBitMark, kDaikinZeroSpace + kDaikinGap,
                  kDaikinFirstHeader64, 64, 38, false, 0, 50);
    } else {  // We are using the newer/more correct size.
      sendGeneric(kDaikinHdrMark, kDaikinHdrSpace, kDaikinBitMark,
                  kDaikinOneSpace, kDaikinBitMark, kDaikinZeroSpace,
                  kDaikinBitMark, kDaikinZeroSpace + kDaikinGap,
                  data, kDaikinSection1Length, 38, false, 0, 50);
      offset += kDaikinSection1Length;
    }
    // Data #2
    sendGeneric(kDaikinHdrMark, kDaikinHdrSpace, kDaikinBitMark,
                kDaikinOneSpace, kDaikinBitMark, kDaikinZeroSpace,
                kDaikinBitMark, kDaikinZeroSpace + kDaikinGap,
                data + offset, kDaikinSection2Length, 38, false, 0, 50);
    offset += kDaikinSection2Length;
    // Data #3
    sendGeneric(kDaikinHdrMark, kDaikinHdrSpace, kDaikinBitMark,
                kDaikinOneSpace, kDaikinBitMark, kDaikinZeroSpace,
                kDaikinBitMark, kDaikinZeroSpace + kDaikinGap,
                data + offset, nbytes - offset, 38, false, 0, 50);
  }

}
#endif  // SEND_DAIKIN

/// Class constructor.
/// @param[in] pin GPIO to be used when sending.
/// @param[in] inverted Is the output signal to be inverted?
/// @param[in] use_modulation Is frequency modulation to be used?
IRDaikinESP::IRDaikinESP(IRrecv *rcv, IRsend *snd)
      : _irsend(snd), _irrecv(rcv) { stateReset(); }


#if SEND_DAIKIN
/// Send the current internal state as an IR message.
/// @param[in] repeat Nr. of times the message will be repeated.
void IRDaikinESP::send(const uint16_t repeat) {
  _irsend.sendDaikin(getRaw(), kDaikinStateLength, repeat);
}
#endif  // SEND_DAIKIN

/// Verify the checksum is valid for a given state.
/// @param[in] state The array to verify the checksum of.
/// @param[in] length The length of the state array.
/// @return true, if the state has a valid checksum. Otherwise, false.
bool IRDaikinESP::validChecksum(uint8_t state[], const uint16_t length) {
  // Data #1
  if (length < kDaikinSection1Length ||
      state[kDaikinByteChecksum1] != sumBytes(state, kDaikinSection1Length - 1))
    return false;
  // Data #2
  if (length < kDaikinSection1Length + kDaikinSection2Length ||
      state[kDaikinByteChecksum2] != sumBytes(state + kDaikinSection1Length,
                                              kDaikinSection2Length - 1))
    return false;
  // Data #3
  if (length < kDaikinSection1Length + kDaikinSection2Length + 2 ||
      state[length - 1] != sumBytes(state + kDaikinSection1Length +
                                    kDaikinSection2Length,
                                    length - (kDaikinSection1Length +
                                              kDaikinSection2Length) - 1))
    return false;
  return true;
}

/// Calculate and set the checksum values for the internal state.
void IRDaikinESP::checksum(void) {
  _.Sum1 = sumBytes(_.raw, kDaikinSection1Length - 1);
  _.Sum2 = sumBytes(_.raw + kDaikinSection1Length, kDaikinSection2Length - 1);
  _.Sum3 = sumBytes(_.raw + kDaikinSection1Length + kDaikinSection2Length,
                    kDaikinSection3Length - 1);
}

/// Reset the internal state to a fixed known good state.
void IRDaikinESP::stateReset(void) {
  for (uint8_t i = 0; i < kDaikinStateLength; i++) _.raw[i] = 0x0;

  _.raw[0] = 0x11;
  _.raw[1] = 0xDA;
  _.raw[2] = 0x27;
  _.raw[4] = 0xC5;
  // _.raw[7] is a checksum byte, it will be set by checksum().
  _.raw[8] = 0x11;
  _.raw[9] = 0xDA;
  _.raw[10] = 0x27;
  _.raw[12] = 0x42;
  // _.raw[15] is a checksum byte, it will be set by checksum().
  _.raw[16] = 0x11;
  _.raw[17] = 0xDA;
  _.raw[18] = 0x27;
  _.raw[21] = 0x49;
  _.raw[22] = 0x1E;
  _.raw[24] = 0xB0;
  _.raw[27] = 0x06;
  _.raw[28] = 0x60;
  _.raw[31] = 0xC0;
  // _.raw[34] is a checksum byte, it will be set by checksum().
  checksum();
}

/// Get a PTR to the internal state/code for this protocol.
/// @return PTR to a code for this protocol based on the current internal state.
uint8_t *IRDaikinESP::getRaw(void) {
  checksum();  // Ensure correct settings before sending.
  return _.raw;
}

/// Set the internal state from a valid code for this protocol.
/// @param[in] new_code A valid code for this protocol.
/// @param[in] length Length of the code in bytes.
void IRDaikinESP::setRaw(const uint8_t new_code[], const uint16_t length) {
  uint8_t offset = 0;
  if (length == kDaikinStateLengthShort) {  // Handle the "short" length case.
    offset = kDaikinStateLength - kDaikinStateLengthShort;
    stateReset();
  }
  for (uint8_t i = 0; i < length && i < kDaikinStateLength; i++)
    _.raw[i + offset] = new_code[i];
}

/// Change the power setting to On.
void IRDaikinESP::on(void) { setPower(true); }

/// Change the power setting to Off.
void IRDaikinESP::off(void) { setPower(false); }

/// Change the power setting.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setPower(const bool on) {
  _.Power = on;
}

/// Get the value of the current power setting.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getPower(void) const {
  return _.Power;
}

/// Set the temperature.
/// @param[in] temp The temperature in degrees celsius.
void IRDaikinESP::setTemp(const uint8_t temp) {
  uint8_t degrees = max(temp, kDaikinMinTemp);
  degrees = min(degrees, kDaikinMaxTemp);
  _.Temp = degrees;
}

/// Get the current temperature setting.
/// @return The current setting for temp. in degrees celsius.
uint8_t IRDaikinESP::getTemp(void) const { return _.Temp; }

/// Set the speed of the fan.
/// @param[in] fan The desired setting.
/// @note 1-5 or kDaikinFanAuto or kDaikinFanQuiet
void IRDaikinESP::setFan(const uint8_t fan) {
  // Set the fan speed bits, leave low 4 bits alone
  uint8_t fanset;
  if (fan == kDaikinFanQuiet || fan == kDaikinFanAuto)
    fanset = fan;
  else if (fan < kDaikinFanMin || fan > kDaikinFanMax)
    fanset = kDaikinFanAuto;
  else
    fanset = 2 + fan;
  _.Fan = fanset;
}

/// Get the current fan speed setting.
/// @return The current fan speed.
uint8_t IRDaikinESP::getFan(void) const {
  uint8_t fan = _.Fan;
  if (fan != kDaikinFanQuiet && fan != kDaikinFanAuto) fan -= 2;
  return fan;
}

/// Get the operating mode setting of the A/C.
/// @return The current operating mode setting.
uint8_t IRDaikinESP::getMode(void) const {
  return _.Mode;
}

/// Set the operating mode of the A/C.
/// @param[in] mode The desired operating mode.
void IRDaikinESP::setMode(const uint8_t mode) {
  switch (mode) {
    case kDaikinAuto:
    case kDaikinCool:
    case kDaikinHeat:
    case kDaikinFan:
    case kDaikinDry:
      _.Mode = mode;
      break;
    default:
      _.Mode = kDaikinAuto;
  }
}

/// Set the Vertical Swing mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setSwingVertical(const bool on) {
  _.SwingV = (on ? kDaikinSwingOn : kDaikinSwingOff);
}

/// Get the Vertical Swing mode of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getSwingVertical(void) const {
  return _.SwingV;
}

/// Set the Horizontal Swing mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setSwingHorizontal(const bool on) {
  _.SwingH = (on ? kDaikinSwingOn : kDaikinSwingOff);
}

/// Get the Horizontal Swing mode of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getSwingHorizontal(void) const {
  return _.SwingH;
}

/// Set the Quiet mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setQuiet(const bool on) {
  _.Quiet = on;
  // Powerful & Quiet mode being on are mutually exclusive.
  if (on) setPowerful(false);
}

/// Get the Quiet mode status of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getQuiet(void) const {
  return _.Quiet;
}

/// Set the Powerful (Turbo) mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setPowerful(const bool on) {
  _.Powerful = on;
  if (on) {
    // Powerful, Quiet, & Econo mode being on are mutually exclusive.
    setQuiet(false);
    setEcono(false);
  }
}

/// Get the Powerful (Turbo) mode of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getPowerful(void) const {
  return _.Powerful;
}

/// Set the Sensor mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setSensor(const bool on) {
  _.Sensor = on;
}

/// Get the Sensor mode of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getSensor(void) const {
  return _.Sensor;
}

/// Set the Economy mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setEcono(const bool on) {
  _.Econo = on;
  // Powerful & Econo mode being on are mutually exclusive.
  if (on) setPowerful(false);
}

/// Get the Economical mode of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getEcono(void) const {
  return _.Econo;
}

/// Set the Mould mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setMold(const bool on) {
  _.Mold = on;
}

/// Get the Mould mode status of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getMold(void) const {
  return _.Mold;
}

/// Set the Comfort mode of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setComfort(const bool on) {
  _.Comfort = on;
}

/// Get the Comfort mode of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getComfort(void) const {
  return _.Comfort;
}

/// Set the enable status & time of the On Timer.
/// @param[in] starttime The number of minutes past midnight.
void IRDaikinESP::enableOnTimer(const uint16_t starttime) {
  _.OnTimer = true;
  _.OnTime = starttime;
}

/// Clear and disable the On timer.
void IRDaikinESP::disableOnTimer(void) {
  _.OnTimer = false;
  _.OnTime = kDaikinUnusedTime;
}

/// Get the On Timer time to be sent to the A/C unit.
/// @return The number of minutes past midnight.
uint16_t IRDaikinESP::getOnTime(void) const {
  return _.OnTime;
}

/// Get the enable status of the On Timer.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getOnTimerEnabled(void) const {
  return _.OnTimer;
}

/// Set the enable status & time of the Off Timer.
/// @param[in] endtime The number of minutes past midnight.
void IRDaikinESP::enableOffTimer(const uint16_t endtime) {
  _.OffTimer = true;
  _.OffTime = endtime;
}

/// Clear and disable the Off timer.
void IRDaikinESP::disableOffTimer(void) {
  _.OffTimer = false;
  _.OffTime = kDaikinUnusedTime;
}

/// Get the Off Timer time to be sent to the A/C unit.
/// @return The number of minutes past midnight.
uint16_t IRDaikinESP::getOffTime(void) const {
  return _.OffTime;
}

/// Get the enable status of the Off Timer.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getOffTimerEnabled(void) const {
  return _.OffTimer;
}

/// Set the clock on the A/C unit.
/// @param[in] mins_since_midnight Nr. of minutes past midnight.
void IRDaikinESP::setCurrentTime(const uint16_t mins_since_midnight) {
  uint16_t mins = mins_since_midnight;
  if (mins > 24 * 60) mins = 0;  // If > 23:59, set to 00:00
  _.CurrentTime = mins;
}

/// Get the clock time to be sent to the A/C unit.
/// @return The number of minutes past midnight.
uint16_t IRDaikinESP::getCurrentTime(void) const {
  return _.CurrentTime;
}

/// Set the current day of the week to be sent to the A/C unit.
/// @param[in] day_of_week The numerical representation of the day of the week.
/// @note 1 is SUN, 2 is MON, ..., 7 is SAT
void IRDaikinESP::setCurrentDay(const uint8_t day_of_week) {
  _.CurrentDay = day_of_week;
}

/// Get the current day of the week to be sent to the A/C unit.
/// @return The numerical representation of the day of the week.
/// @note 1 is SUN, 2 is MON, ..., 7 is SAT
uint8_t IRDaikinESP::getCurrentDay(void) const {
  return _.CurrentDay;
}

/// Set the enable status of the Weekly Timer.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRDaikinESP::setWeeklyTimerEnable(const bool on) {
  // Bit is cleared for `on`.
  _.WeeklyTimer = !on;
}

/// Get the enable status of the Weekly Timer.
/// @return true, the setting is on. false, the setting is off.
bool IRDaikinESP::getWeeklyTimerEnable(void) const {
  return !_.WeeklyTimer;
}

#if 0
/// Convert a stdAc::opmode_t enum into its native mode.
/// @param[in] mode The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IRDaikinESP::convertMode(const stdAc::opmode_t mode) {
  switch (mode) {
    case stdAc::opmode_t::kCool: return kDaikinCool;
    case stdAc::opmode_t::kHeat: return kDaikinHeat;
    case stdAc::opmode_t::kDry: return kDaikinDry;
    case stdAc::opmode_t::kFan: return kDaikinFan;
    default: return kDaikinAuto;
  }
}

/// Convert a stdAc::fanspeed_t enum into it's native speed.
/// @param[in] speed The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IRDaikinESP::convertFan(const stdAc::fanspeed_t speed) {
  switch (speed) {
    case stdAc::fanspeed_t::kMin: return kDaikinFanQuiet;
    case stdAc::fanspeed_t::kLow: return kDaikinFanMin;
    case stdAc::fanspeed_t::kMedium: return kDaikinFanMed;
    case stdAc::fanspeed_t::kHigh: return kDaikinFanMax - 1;
    case stdAc::fanspeed_t::kMax: return kDaikinFanMax;
    default: return kDaikinFanAuto;
  }
}

/// Convert a native mode into its stdAc equivalent.
/// @param[in] mode The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::opmode_t IRDaikinESP::toCommonMode(const uint8_t mode) {
  switch (mode) {
    case kDaikinCool: return stdAc::opmode_t::kCool;
    case kDaikinHeat: return stdAc::opmode_t::kHeat;
    case kDaikinDry: return stdAc::opmode_t::kDry;
    case kDaikinFan: return stdAc::opmode_t::kFan;
    default: return stdAc::opmode_t::kAuto;
  }
}

/// Convert a native fan speed into its stdAc equivalent.
/// @param[in] speed The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::fanspeed_t IRDaikinESP::toCommonFanSpeed(const uint8_t speed) {
  switch (speed) {
    case kDaikinFanMax: return stdAc::fanspeed_t::kMax;
    case kDaikinFanMax - 1: return stdAc::fanspeed_t::kHigh;
    case kDaikinFanMed:
    case kDaikinFanMin + 1: return stdAc::fanspeed_t::kMedium;
    case kDaikinFanMin: return stdAc::fanspeed_t::kLow;
    case kDaikinFanQuiet: return stdAc::fanspeed_t::kMin;
    default: return stdAc::fanspeed_t::kAuto;
  }
}

/// Convert the current internal state into its stdAc::state_t equivalent.
/// @return The stdAc equivalent of the native settings.
stdAc::state_t IRDaikinESP::toCommon(void) const {
  stdAc::state_t result{};
  result.protocol = decode_type_t::DAIKIN;
  result.model = -1;  // No models used.
  result.power = _.Power;
  result.mode = toCommonMode(_.Mode);
  result.celsius = true;
  result.degrees = _.Temp;
  result.fanspeed = toCommonFanSpeed(getFan());
  result.swingv = _.SwingV ? stdAc::swingv_t::kAuto :
                                             stdAc::swingv_t::kOff;
  result.swingh = _.SwingH ? stdAc::swingh_t::kAuto :
                                               stdAc::swingh_t::kOff;
  result.quiet = _.Quiet;
  result.turbo = _.Powerful;
  result.clean = _.Mold;
  result.econo = _.Econo;
  // Not supported.
  result.filter = false;
  result.light = false;
  result.beep = false;
  result.sleep = -1;
  result.clock = -1;
  return result;
}
#endif
/// Convert the current internal state into a human readable string.
/// @return A human readable string.
String IRDaikinESP::toString(void) const {
  String result = "";
  result.reserve(230);  // Reserve some heap for the string to reduce fragging.
  result += addBoolToString(_.Power, "Power");
  result += addModeToString(_.Mode, kDaikinAuto, kDaikinCool, kDaikinHeat,
                            kDaikinDry, kDaikinFan);
  result += addTempToString(_.Temp);
  result += addFanToString(getFan(), kDaikinFanMax, kDaikinFanMin,
                           kDaikinFanAuto, kDaikinFanQuiet, kDaikinFanMed, 0xFF, 0xFF);
  result += addBoolToString(_.Powerful, "Powerful");
  result += addBoolToString(_.Quiet, "Quiet");
  result += addBoolToString(getSensor(), "Sensor");
  result += addBoolToString(_.Mold, "Mold");
  result += addBoolToString(_.Comfort, "Comfort");
  result += addBoolToString(_.SwingH, "HSwing");
  result += addBoolToString(_.SwingV, "VSwing");
  result += addLabeledString(minsToString(_.CurrentTime), "Clock");
  result += addDayToString(_.CurrentDay, -1);
  result += addLabeledString(_.OnTimer
                             ? minsToString(_.OnTime) : "Off",
                             "Timer ON");
  result += addLabeledString(_.OffTimer
                             ? minsToString(_.OffTime) : "Off",
                             "Timer OFF");
  result += addBoolToString(getWeeklyTimerEnable(), "Weekly");
  return result;
}

/*
 * returns true if values do match
 */
bool checkDaikinSync( uint16_t* rawbuf, PulseDistanceWidthProtocolConstants *aProtocolConstants) {
// Check header "mark" and "space"
  int startindex = 1;
  for (int i=0; i<kDaikinHeaderLength; i++)
  {
    if (!matchMark(rawbuf[startindex++], aProtocolConstants->DistanceWidthTimingInfo.ZeroMarkMicros)) {
#if defined(LOCAL_DEBUG)
        Serial.print(::getProtocolString(aProtocolConstants->ProtocolIndex));
        Serial.println(F(": Sync mark length is wrong"));
#endif
        return false;
    }
    if (!matchSpace(rawbuf[startindex++], aProtocolConstants->DistanceWidthTimingInfo.ZeroSpaceMicros)) {
#if defined(LOCAL_DEBUG)
        Serial.print(::getProtocolString(aProtocolConstants->ProtocolIndex));
        Serial.println(F(": Sync space length is wrong"));
#endif
        return false;
    }
  }
  if (!matchMark(rawbuf[startindex++], aProtocolConstants->DistanceWidthTimingInfo.ZeroMarkMicros)) {
#if defined(LOCAL_DEBUG)
      Serial.print(::getProtocolString(aProtocolConstants->ProtocolIndex));
      Serial.println(F(": Sync mark length is wrong"));
#endif
      return false;
  }
  return true;
}

/*
 * returns true if values do match
 */
bool checkDaikinHeader(uint16_t* rawbuf, PulseDistanceWidthProtocolConstants *aProtocolConstants) {
// Check header "mark" and "space"
    if (!matchMark(rawbuf[0], aProtocolConstants->DistanceWidthTimingInfo.HeaderMarkMicros)) {
#if defined(LOCAL_TRACE)
        Serial.print(::getProtocolString(aProtocolConstants->ProtocolIndex));
        Serial.println(F(": Header mark length is wrong"));
#endif
        return false;
    }
    if (!matchSpace(rawbuf[1], aProtocolConstants->DistanceWidthTimingInfo.HeaderSpaceMicros)) {
#if defined(LOCAL_TRACE)
        Serial.print(::getProtocolString(aProtocolConstants->ProtocolIndex));
        Serial.println(F(": Header space length is wrong"));
#endif
        return false;
    }
    return true;
}

bool decodeDaikinData(IRrecv *irrecv, PulseDistanceWidthProtocolConstants *aProtocolConstants, uint16_t &rawoffset, uint8_t *bytebuf, uint8_t bytelen)
{
  for (int i=0; i < bytelen; i++)
  {
    if (!irrecv->decodePulseDistanceWidthData(&DaikinProtocolConstants, 8, rawoffset)) {
        #if defined(LOCAL_DEBUG)
        Serial.print(F("Denon: offset: "));
        Serial.print(rawoffset);
        Serial.println(F(" Decode failed"));
        #endif
        return false;
    }
    else
    {
      Serial.print("byte ");
      Serial.print(i);
      Serial.print(": 0x");
      Serial.print((uint8_t)irrecv->decodedIRData.decodedRawData);
      Serial.println();
    }
    bytebuf[i] = (uint8_t)irrecv->decodedIRData.decodedRawData;
    rawoffset += 8 * 2;
  }
  return true;
}


/// Decode the supplied Daikin 280-bit message. (DAIKIN)
/// @return A boolean. True if it can decode it, false if it can't.
/// @see https://github.com/mharizanov/Daikin-AC-remote-control-over-the-Internet/tree/master/IRremote
bool IRDaikinESP::decodeDaikin() {
  // Is there enough data to match successfully?
    Serial.print(F("Denon: "));
    Serial.print(F("Data length="));
    Serial.print(_irrecv->decodedIRData.rawDataPtr->rawlen);
    Serial.println("");
    uint16_t offset = 0;
  
/*
Protocol structure:
  Sync (5 bits)
  gap (20ms)
  header
  Data part 1 (8 Byte)
  stop bit
  gap (20ms)
  header
  Data part 2 (8 Byte)
  stop bit
  gap (20ms)
  header
  Data part 3 (19 Byte)
  stop bit


*/


    if (!checkDaikinSync(_irrecv->decodedIRData.rawDataPtr->rawbuf, &DaikinProtocolConstants)) {
        #if defined(LOCAL_DEBUG)
        Serial.print(F("Denon: "));
        Serial.println(F("Sync Decode failed"));
        #endif
        return false;
    }
    // offset after sync pulses
    offset = 13 ;
    if (!checkDaikinHeader(&_irrecv->decodedIRData.rawDataPtr->rawbuf[offset], &DaikinProtocolConstants)) {
        #if defined(LOCAL_DEBUG)
        Serial.print(F("Denon: "));
        Serial.println(F("Header Decode failed"));
        #endif
        return false;
    }
    offset+=2;

    if (!decodeDaikinData(_irrecv, &DaikinProtocolConstants, offset, &_.raw[0], 8))
    {
        return false;
    }
    offset += 2; // stopbit + gap

    if (!checkDaikinHeader(&_irrecv->decodedIRData.rawDataPtr->rawbuf[offset], &DaikinProtocolConstants)) {
        #if defined(LOCAL_DEBUG)
        Serial.print(F("Denon: "));
        Serial.println(F("Header2 Decode failed"));
        #endif
        return false;
    }
    offset+=2;

    if (!decodeDaikinData(_irrecv, &DaikinProtocolConstants, offset, &_.raw[8], 8))
    {
      return false;
    }
    offset += 2; // stopbit + gap


    if (!checkDaikinHeader(&_irrecv->decodedIRData.rawDataPtr->rawbuf[offset], &DaikinProtocolConstants)) {
        #if defined(LOCAL_DEBUG)
        Serial.print(F("Denon: "));
        Serial.println(F("Header3 Decode failed"));
        #endif
        return false;
    }
    offset+=2;

    if (!decodeDaikinData(_irrecv, &DaikinProtocolConstants, offset, &_.raw[16], 19))
    {
      return false;
    }
  return true;
}

bool IRDaikinESP::decode()
{
    if (!_irrecv->decode())
        return false;
    if (_irrecv->decodedIRData.rawDataPtr->rawlen != 584)
    {
        _irrecv->resume(); // Enable receiving of the next value
        return false;
    }

    if (decodeDaikin()) {

        /*
         * !!!Important!!! Enable receiving of the next value,
         * since receiving has stopped after the end of the current received data packet.
         */
        _irrecv->resume(); // Enable receiving of the next value
        return true;
    }  
    _irrecv->resume(); // Enable receiving of the next value
    return false;
}


#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif


#endif //_IR_DAIKIN_HPP