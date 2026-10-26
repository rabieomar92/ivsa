#pragma once

#include <stdint.h>

enum class ModbusProfileDataType : uint8_t {
  UInt16 = 0,
  Int16,
  UInt32,
  Int32,
  UInt64,
  Int64,
  Float32,
  Float64,
  Bcd16,
  Bcd32,
  UInt32Mod10000,
  Int32Mod10000,
  Boolean
};

// A schema-3 field describes one numeric engineering value independently.
// wireByteOrder names the canonical most-to-least-significant bytes in the
// order they arrive from consecutive Modbus registers: AB, ABCD, CDAB, etc.
struct ModbusFieldProfile {
  bool enabled;
  uint8_t readFunction;
  uint16_t address;
  uint8_t registerCount;
  ModbusProfileDataType dataType;
  char wireByteOrder[9];
  double scale;
  double offset;
  uint64_t bitMask;
  uint8_t bitShift;
  bool hasBitField;
  bool bitFieldSigned;
  bool hasInvalidRaw;
  uint64_t invalidRaw;
  bool hasValidRange;
  double validMinimum;
  double validMaximum;
};

struct ModbusVectorProfile {
  uint16_t startRegister;
  float scale;
  bool signedRaw;
  uint16_t yRegister;
  uint16_t zRegister;
  bool usesExplicitAxisRegisters;
};

struct ModbusScalarProfile {
  uint16_t registerAddress;
  float scale;
  bool signedRaw;
};

struct ModbusSensorProfile {
  char profileKey[32];
  char sensorType[48];
  char modelName[64];
  char registerMapVersion[32];
  uint32_t defaultBaud;
  uint8_t defaultSlaveId;
  uint8_t readFunction;
  ModbusVectorProfile acceleration;
  ModbusVectorProfile velocity;
  ModbusVectorProfile displacement;
  ModbusVectorProfile frequency;
  ModbusScalarProfile temperature;
  float frequencyMinimum;
  float frequencyMaximum;
  bool supportsVendorClock;
  uint16_t clockStartRegister;
  uint16_t clockRegisterCount;
  bool supportsContiguousFastFrame;
  uint16_t fastFrameStartRegister;
  uint16_t fastFrameRegisterCount;
  bool accelerationIsInstantaneousWaveform;
  uint32_t minimumPollIntervalMs;
  bool velocityIsRms;
  bool usesGenericFieldMap;
  char serialFormat[4];
  uint8_t maximumReadRegisters;
  uint8_t bulkReadFunction;
  ModbusFieldProfile accelerationFields[3];
  ModbusFieldProfile velocityFields[3];
  ModbusFieldProfile displacementFields[3];
  ModbusFieldProfile frequencyFields[3];
  ModbusFieldProfile temperatureField;
};

// Add compile-time fallback sensors here, then select one with a PlatformIO
// build flag. Uploaded schema-3 profiles support per-field addresses,
// datatypes, byte orders, scaling, offsets, and optional measurement groups.
#if defined(SENSOR_PROFILE_CUSTOM)
constexpr ModbusSensorProfile DEFAULT_SENSOR_PROFILE = {
  "custom",
  "generic_modbus_vibration",
  "Custom Modbus vibration sensor",
  "custom-v1",
  9600,
  0x01,
  0x03,
  { 0x0000, 1.0f, true,  0, 0, false }, // acceleration X/Y/Z
  { 0x0003, 1.0f, false, 0, 0, false }, // velocity X/Y/Z
  { 0x0006, 1.0f, false, 0, 0, false }, // displacement X/Y/Z
  { 0x0009, 1.0f, false, 0, 0, false }, // frequency X/Y/Z
  { 0x000C, 1.0f, true  }, // temperature
  0.0f,
  65535.0f,
  false,
  0,
  0,
  false,
  0,
  0,
  true,
  100,
  true,
  false,
  "",
  0,
  0,
  {},
  {},
  {},
  {},
  {}
};
#else
constexpr ModbusSensorProfile DEFAULT_SENSOR_PROFILE = {
  "default_modbus_vibration",
  "generic_modbus_vibration",
  "Profile-defined Modbus vibration sensor",
  "default-register-map-v1",
  9600,
  0x50,
  0x03,
  { 0x0034, (16.0f / 32768.0f) * 9.80665f, true,  0, 0, false },
  { 0x003A, 0.01f,                         false, 0, 0, false },
  { 0x0041, 1.0f,                          false, 0, 0, false },
  { 0x0044, 0.1f,                          false, 0, 0, false },
  { 0x0040, 0.01f,                         true  },
  0.1f,
  6553.5f,
  true,
  0x0030,
  4,
  true,
  0x0034,
  19,
  true,
  100,
  true,
  false,
  "",
  0,
  0,
  {},
  {},
  {},
  {},
  {}
};
#endif

constexpr uint32_t SENSOR_PROFILE_KNOWN_BAUDS[] = {
  1200, 2400, 4800, 9600, 14400, 19200, 38400,
  56000, 57600, 115200, 230400, 460800, 921600
};

// A built-in fast frame is safe only when every configured field decoded from it
// is inside the declared block and stays within the Modbus 125-register limit.
static_assert(
  !DEFAULT_SENSOR_PROFILE.supportsContiguousFastFrame ||
  (DEFAULT_SENSOR_PROFILE.fastFrameRegisterCount > 0 &&
   DEFAULT_SENSOR_PROFILE.fastFrameRegisterCount <= 125 &&
   DEFAULT_SENSOR_PROFILE.acceleration.startRegister >= DEFAULT_SENSOR_PROFILE.fastFrameStartRegister &&
   DEFAULT_SENSOR_PROFILE.acceleration.startRegister + 2 <
     DEFAULT_SENSOR_PROFILE.fastFrameStartRegister + DEFAULT_SENSOR_PROFILE.fastFrameRegisterCount &&
   DEFAULT_SENSOR_PROFILE.velocity.startRegister >= DEFAULT_SENSOR_PROFILE.fastFrameStartRegister &&
   DEFAULT_SENSOR_PROFILE.velocity.startRegister + 2 <
     DEFAULT_SENSOR_PROFILE.fastFrameStartRegister + DEFAULT_SENSOR_PROFILE.fastFrameRegisterCount &&
   DEFAULT_SENSOR_PROFILE.displacement.startRegister >= DEFAULT_SENSOR_PROFILE.fastFrameStartRegister &&
   DEFAULT_SENSOR_PROFILE.displacement.startRegister + 2 <
     DEFAULT_SENSOR_PROFILE.fastFrameStartRegister + DEFAULT_SENSOR_PROFILE.fastFrameRegisterCount &&
   DEFAULT_SENSOR_PROFILE.frequency.startRegister >= DEFAULT_SENSOR_PROFILE.fastFrameStartRegister &&
   DEFAULT_SENSOR_PROFILE.frequency.startRegister + 2 <
     DEFAULT_SENSOR_PROFILE.fastFrameStartRegister + DEFAULT_SENSOR_PROFILE.fastFrameRegisterCount &&
   DEFAULT_SENSOR_PROFILE.temperature.registerAddress >= DEFAULT_SENSOR_PROFILE.fastFrameStartRegister &&
   DEFAULT_SENSOR_PROFILE.temperature.registerAddress <
     DEFAULT_SENSOR_PROFILE.fastFrameStartRegister + DEFAULT_SENSOR_PROFILE.fastFrameRegisterCount),
  "Sensor profile fast frame does not contain every decoded register");
