# VibScope generic Modbus sensor profiles

VibScope firmware 1.8 uses sensor-profile schema 3. Scaling and decoding happen
only in firmware; the website and MQTT consumers receive canonical engineering
units regardless of sensor manufacturer.

Profiles are uploaded from the always-available setup page at
`http://192.168.4.1`. The controller validates the whole profile, writes and
reads back two CRC-protected NVS copies, and activates it only after persistence
succeeds. Schema-1 and schema-2 JSON and stored profiles remain accepted and are
migrated to schema 3 automatically.

## Protocol boundary

Modbus defines 16-bit registers and the wire format for functions such as 03 and
04. It does not standardize how a manufacturer combines registers into larger
integers, floats, byte/word orders, packed flags, fixed-point values, or
engineering formulas. Schema 3 makes those vendor choices explicit per field.

VibScope uses zero-based PDU addresses. A manual that labels holding registers
as 40001, 40002, and so on must be converted to addresses 0, 1, and so on.
Profiles must state:

```json
"addressConvention": "pdu_zero_based"
```

This prevents the common one-register and 40001-offset mistakes.

## Schema-3 structure

The top-level sections are:

- Identity: `profileKey`, `sensorType`, `modelName`, and
  `registerMapVersion`.
- `modbus`: RTU transport, baud, slave ID, character format, default function,
  address convention, and the sensor's safe maximum read size.
- `measurements`: acceleration, velocity, displacement, frequency, and
  temperature field descriptors.
- `frequencyRangeHz`: accepted sensor-reported vibration-frequency range.
- `acquisition`: the minimum safe polling interval.
- `vendorClock`: optional WTVB01-compatible four-register clock.
- `bulkRead`: optional single-function block containing every measurement
  field.

Canonical units are mandatory:

- Acceleration: `m/s^2`
- Velocity: `mm/s`
- Displacement: `um`
- Frequency: `Hz`
- Temperature: `C`

Acceleration and velocity require complete X/Y/Z fields, and temperature is
required. Displacement and frequency may each provide complete X/Y/Z fields or
explicitly disable all three with `{"enabled": false}`. When frequency is
disabled, its range must be `0..0`. Firmware publishes disabled groups as
`null` rather than inventing values from unrelated registers.

## Field descriptor

Every measurement axis is independent:

```json
{
  "enabled": true,
  "function": 3,
  "address": 100,
  "registerCount": 2,
  "dataType": "float32",
  "byteOrder": "CDAB",
  "scale": 1.0,
  "offset": 0.0,
  "bitField": {
    "mask": "0x00FFFFFF",
    "shift": 0,
    "signed": true
  },
  "invalidRaw": "0xFFFFFFFF",
  "validRange": {
    "minimum": -1000.0,
    "maximum": 1000.0
  }
}
```

Required fields:

- `enabled`: currently `true` for required VibScope measurements.
- `function`: `3` for holding registers or `4` for input registers.
- `address`: zero-based PDU register address.
- `registerCount`: must match the datatype.
- `dataType`: raw vendor encoding.
- `byteOrder`: received-byte order relative to canonical most-significant byte
  `A`, then `B`, `C`, and so on.
- `scale` and `offset`: `engineering = decoded * scale + offset`.

Optional fields:

- `bitField`: extracts packed integer bits. `signed: true` performs sign
  extension; signed masks must be contiguous after shifting.
- `invalidRaw`: an exact raw sentinel. Use a quoted decimal or hexadecimal
  string for values that may exceed JSON's exact integer range.
- `validRange`: rejects non-finite or out-of-range engineering values.

## Supported datatypes

| `dataType` | Registers | Meaning |
|---|---:|---|
| `uint16`, `int16` | 1 | Standard 16-bit integer |
| `uint32`, `int32` | 2 | Standard 32-bit integer |
| `uint64`, `int64` | 4 | Standard 64-bit integer |
| `float32` | 2 | IEEE-754 binary32 |
| `float64` | 4 | IEEE-754 binary64 |
| `bcd16`, `bcd32` | 1 or 2 | Packed binary-coded decimal |
| `uint32_mod10000` | 2 | High word × 10000 + low word |
| `int32_mod10000` | 2 | Signed modulo-10000 variant |
| `boolean` | 1 | Zero/nonzero, normally with `bitField` |

Fixed-point/Q-format values use an integer datatype plus `scale`. Bias-based
temperatures use `offset`. Unsigned or signed 24-bit packed values use a
32-bit datatype plus `bitField`.

Raw integers are preserved as 64-bit values internally, and raw values larger
than JavaScript's exact integer range are serialized as quoted strings.
However, the existing VibScope calculation and telemetry structures store the
final engineering measurements as IEEE-754 binary32 values. Consequently,
`uint64`, `int64`, and `float64` profiles are useful for decoding and scaling,
but they do not promise more than binary32 precision after the engineering
value enters the measurement pipeline.

## Byte and word order

The characters name canonical bytes in the order received:

- `AB`: standard 16-bit Modbus word.
- `BA`: byte-swapped 16-bit word.
- `ABCD`: high word then low word.
- `CDAB`: word-swapped 32-bit value.
- `BADC`: bytes swapped inside each 16-bit word.
- `DCBA`: fully reversed 32-bit value.
- 64-bit fields may use any valid permutation of `ABCDEFGH`.

Every character must appear exactly once. A datatype's byte order length must
equal its encoded byte width.

## Serial formats and reads

Supported RTU character formats are `8N1`, `8N2`, `8E1`, `8E2`, `8O1`, and
`8O2`. Explicit profile baud rates from 300 through 2,000,000 are accepted.
Automatic probing remains limited to a conservative list of common rates.
RTU silent timing is calculated from the selected character format, including
the additional bit used by parity-plus-two-stop-bit formats. Response timeouts
also expand with frame size and baud rate, allowing a full 125-register
response to complete at 300 baud.

Functions 03 and 04 may be selected independently for every field. Fields with
the same function are coalesced when their span fits `maximumReadRegisters`;
otherwise they are read independently. A `bulkRead` may cover 1-125 registers,
the Modbus maximum, but must contain every enabled measurement field and all
those fields must use its function. If a device rejects the bulk block, firmware
falls back to smaller profile-driven reads.

## Measurement semantics

Acceleration semantics:

- `instantaneous_waveform`: firmware performs baseline removal and calculates
  waveform RMS.
- `sensor_reported_magnitude`: firmware publishes the scaled vendor magnitude
  directly and does not invent waveform RMS, peak, or peak-to-peak values.

Velocity semantics:

- `sensor_reported_rms`: RMS alarm thresholds are allowed.
- `sensor_reported_unspecified`: the value is published, but RMS alarm
  thresholds are disabled and status remains `DiagnosticOnly`.

## Included profiles

- `sensor_profiles/witmotion_wtvb01.json`
- `sensor_profiles/bx2601a_wz01m.json`
- `sensor_profiles/bydr3016zw.json`
- `sensor_profiles/banner_qm30vt3.json`

The WitMotion profile uses signed 16-bit waveform acceleration and unsigned
16-bit sensor-reported engineering registers.

The BX2601A profile maps its sparse Z/Y/X register groups explicitly. Its
acceleration scale is `0.01 g × 9.80665 = 0.0980665 m/s^2`. The manual does not
identify acceleration or velocity as RMS, so the profile does not claim that
semantics or apply RMS velocity alarms. Its greater-than-300-ms update statement
is represented by a conservative 350-ms polling floor. Bulk reading is
disabled because the manual does not prove that one 22-register request is
accepted.

The BYDR3016ZW profile maps velocity registers 0-2, temperature register 3,
acceleration registers 4-6, and displacement registers 19-21. The manual's
"magnification 100" values are divided by 100 through a `0.01` scale.
Displacement uses scale 1. Its default time-domain calculation cycle is
`3 × 0.32 s`, represented by a 960-ms polling floor. Bulk reading is disabled
because the manual demonstrates a seven-register request but does not state
that one 22-register block is supported. The manual does not document a
per-axis vibration-frequency register in Hz; spectrum energy and peak-bin
"order" registers are deliberately not mislabeled as frequency. The profile
therefore disables frequency and publishes it as unavailable. `8N1` is the
normal Modbus RTU default but is an inference because this manual does not
explicitly state parity and stop bits; verify it on the physical sensor.

The Banner QM30VT3 profile converts 40001-based manual labels to
zero-based PDU addresses. It uses full-band RMS acceleration at profile
addresses 33-35, RMS velocity at 36/38/40, peak velocity component frequency
at 25-27, and Celsius temperature at 42. Acceleration is converted from
`0.001 g/count` to `0.00980665 m/s^2/count`. Velocity uses
`0.001 mm/s/count`, frequency uses `0.1 Hz/count`, and temperature uses
`0.01 C/count`. The sensor has no displacement register, so displacement is
explicitly disabled. Default communication is 19200 baud, slave 1, 8N1.
Bulk reading is initially disabled, and the default 5300-Hz FMax sample
duration provides a 300-ms polling floor.

A simpler tutorial for creating new profiles from a register manual is in
`SENSOR_PROFILE_BEGINNER_GUIDE.md`.

## Deliberately unsupported cases

No finite JSON convention can safely infer every proprietary sensor. Firmware
changes are still required for:

- Modbus ASCII instead of RTU.
- Coil/discrete functions 01/02 or vendor-specific function codes.
- A measurement assembled from non-consecutive registers within one value.
- Per-field slave IDs or multiple physical sensors in one profile.
- Registers that require unlock writes, bank switching, command/response
  sequences, or write-before-read behavior.
- Strings, variable-length payloads, 128-bit values, decimal floating point,
  signed/overpunched BCD, vendor checksums inside register data, or dynamic
  register maps.
- Nonlinear formulas, polynomials, lookup tables, logarithmic encoding, or
  cross-field compensation.
- Sensors that omit acceleration, velocity, or temperature.

Do not force such a device into a nearly matching datatype. Add an audited
decoder or a future schema capability and test it against known raw frames.

## Production workflow

1. Copy the closest schema-3 profile.
2. Convert all manual addresses to zero-based PDU addresses.
3. Enter each field's function, datatype, byte order, scale, and offset.
4. Disable `bulkRead` unless one documented safe block contains every field.
5. Upload through the setup AP.
6. Download the active profile and compare it with the source file.
7. Compare raw and engineering values against a trusted Modbus tool.
8. Power-cycle repeatedly and verify `profileStorageHealthy`.

The upload build allocates a new `2607-12-#####` device serial. Normal builds
reuse the current serial so firmware binaries, labels, and production records
remain consistent.
