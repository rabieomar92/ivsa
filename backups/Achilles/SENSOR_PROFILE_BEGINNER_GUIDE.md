# How to make a VibScope sensor profile

This guide is written for someone who can read a sensor manual but has not
worked with Modbus before. A sensor profile is a JSON file that tells VibScope:

1. how to talk to the sensor;
2. which registers contain each measurement; and
3. how to turn the register number into a real engineering value.

The website does not fix sensor scaling. The profile tells the firmware how to
produce the standard VibScope units.

## Before you start

Find these facts in the sensor manual:

- Protocol: it must be Modbus RTU over RS-485.
- Baud rate, slave address, parity, data bits, and stop bits.
- Read function: usually 03 for holding registers or 04 for input registers.
- Register address for X, Y, and Z acceleration.
- Register address for X, Y, and Z velocity.
- Register address for X, Y, and Z displacement, if available.
- Register address for X, Y, and Z vibration frequency, if available.
- Register address for temperature.
- Datatype, scale, units, and update time for every value.

Do not guess a missing fact. Write it in an "items to verify" list and confirm
it with the manufacturer or a trusted Modbus program.

## Five JSON rules

JSON is strict, but its basic rules are simple:

1. Names and text must be inside double quotes: `"baud"`.
2. Put a colon between a name and its value: `"baud": 9600`.
3. Put a comma after an item when another item follows it.
4. Do not put a comma after the final item in an object.
5. Write booleans as lowercase `true` or `false`. JSON does not allow comments.

Use a period for decimals, for example `0.01`, even if your country normally
uses a decimal comma.

Braces `{ }` contain an object. Brackets `[ ]` contain a list. A missing comma,
extra comma, or unmatched brace makes the whole profile invalid. Start by
copying a working profile instead of typing every brace yourself.

## Step 1: Copy a working profile

Start with the profile that looks most like your sensor:

- `sensor_profiles/witmotion_wtvb01.json`
- `sensor_profiles/bx2601a_wz01m.json`
- `sensor_profiles/bydr3016zw.json`
- `sensor_profiles/banner_qm30vt3.json`

Keep `"schemaVersion": 3`.

Change the identity strings. Their maximum lengths are:

| Name | Maximum |
|---|---:|
| `profileKey` | 31 characters |
| `sensorType` | 47 characters |
| `modelName` | 63 characters |
| `registerMapVersion` | 31 characters |

Use a short `registerMapVersion` that identifies the manual, for example:

```json
"registerMapVersion": "QM30VT3-244688-20260210"
```

## Step 2: Enter the communication settings

Example:

```json
"modbus": {
  "mode": "rtu",
  "baud": 19200,
  "slaveId": 1,
  "serialFormat": "8N1",
  "addressConvention": "pdu_zero_based",
  "defaultReadFunction": 3,
  "maximumReadRegisters": 16
}
```

`serialFormat` means:

- `8N1`: 8 data bits, no parity, 1 stop bit
- `8N2`: 8 data bits, no parity, 2 stop bits
- `8E1` or `8E2`: even parity
- `8O1` or `8O2`: odd parity

The allowed slave addresses are 1 to 247. Use the factory default from the
manual unless the physical sensor has been reconfigured.

Use `maximumReadRegisters: 16` when the manual does not state a maximum. Raise
it only when the manual or a physical test proves that larger reads work.

## Step 3: Convert the register address

VibScope profiles always use zero-based PDU addresses.

Some manuals already use zero-based addresses:

| Manual address | Profile address |
|---:|---:|
| 0 | 0 |
| 9 | 9 |
| `0x34` | 52 |

Some manuals use labels beginning at 40001:

```text
profile address = manual 4xxxx address - 40001
```

Examples:

| Manual label | Profile address |
|---:|---:|
| 40001 | 0 |
| 40026 | 25 |
| 40043 | 42 |

Do not subtract 40001 from a manual that already gives address 0 or a hexadecimal
address such as `0x34`.

When uncertain, inspect a request example. A request containing starting bytes
`00 34` uses profile address `0x0034`, which is decimal 52.

## Step 4: Describe one field

A normal 16-bit field looks like this:

```json
"x": {
  "enabled": true,
  "function": 3,
  "address": 36,
  "registerCount": 1,
  "dataType": "uint16",
  "byteOrder": "AB",
  "scale": 0.001,
  "offset": 0.0
}
```

The firmware calculates:

```text
engineering value = decoded register value × scale + offset
```

For example, raw value 1234 with scale `0.001` becomes `1.234`.

## Step 5: Choose the datatype

Use exactly what the manual states:

| Manual wording | `dataType` | Registers |
|---|---|---:|
| unsigned short, uint16 | `uint16` | 1 |
| signed short, int16 | `int16` | 1 |
| unsigned 32-bit | `uint32` | 2 |
| signed 32-bit | `int32` | 2 |
| unsigned 64-bit | `uint64` | 4 |
| signed 64-bit | `int64` | 4 |
| IEEE single precision | `float32` | 2 |
| IEEE double precision | `float64` | 4 |
| packed BCD, 4 digits | `bcd16` | 1 |
| packed BCD, 8 digits | `bcd32` | 2 |
| unsigned modulo 10000 | `uint32_mod10000` | 2 |
| signed modulo 10000 | `int32_mod10000` | 2 |
| packed on/off bits | `boolean` | 1 |

Do not call a value signed merely because the engineering range can be
negative. The manual or response example must show that the register itself is
signed.

## Step 6: Choose byte order

For one normal Modbus register, use:

```json
"byteOrder": "AB"
```

For a two-register 32-bit value, common choices are:

| Manual wording | Byte order |
|---|---|
| high word first, big-endian | `ABCD` |
| low word first, word-swapped | `CDAB` |
| bytes swapped in each word | `BADC` |
| fully reversed | `DCBA` |

The letters describe where each received byte belongs. Every letter must appear
once. Never guess byte order from the datatype alone. Compare a known physical
value or a response example.

## Step 7: Calculate scale and offset

### Manual says "divide by 100"

```json
"scale": 0.01,
"offset": 0.0
```

### Manual says "one decimal place"

```json
"scale": 0.1,
"offset": 0.0
```

### Manual says "Scale (exp) = -3"

Ten to the power of -3 is `0.001`:

```json
"scale": 0.001,
"offset": 0.0
```

### Manual reports acceleration in g

VibScope uses `m/s^2`. Multiply the sensor scale by `9.80665`.

If the sensor value is `0.001 g` per count:

```text
0.001 × 9.80665 = 0.00980665 m/s^2 per count
```

Use:

```json
"scale": 0.00980665,
"offset": 0.0
```

### Manual reports temperature in degrees Fahrenheit

If raw temperature has scale `0.01 °F`:

```text
C = raw × 0.01 × 5/9 - 32 × 5/9
```

The matching profile values are approximately:

```json
"scale": 0.00555555555555556,
"offset": -17.7777777777778
```

## Step 8: Use the required VibScope units

Profiles must output:

| Measurement | Unit |
|---|---|
| Acceleration | `m/s^2` |
| Velocity | `mm/s` |
| Displacement | `um` |
| Frequency | `Hz` |
| Temperature | `C` |

Conversion examples:

- inches/second to mm/s: multiply by `25.4`
- g to m/s²: multiply by `9.80665`
- millimetres to micrometres: multiply by `1000`

Include the unit conversion inside `scale`.

## Step 9: State what the measurement means

Acceleration must use one of:

- `instantaneous_waveform`: each read is an instantaneous acceleration sample;
  firmware may calculate RMS from several samples.
- `sensor_reported_magnitude`: the sensor already calculated RMS, peak, or
  another magnitude. Firmware must not treat it as a waveform sample.

Velocity must use one of:

- `sensor_reported_rms`: the manual explicitly says RMS.
- `sensor_reported_unspecified`: the manual does not say RMS. VibScope will
  publish it but will not apply RMS alarm thresholds.

Never select RMS merely because the value looks reasonable.

## Step 10: Disable a measurement the sensor does not have

Acceleration, velocity, and temperature are required by the current VibScope
telemetry contract.

Displacement and frequency are optional. Disable all three axes together:

```json
"displacement": {
  "unit": "um",
  "semantics": "sensor_reported",
  "x": {"enabled": false},
  "y": {"enabled": false},
  "z": {"enabled": false}
}
```

For disabled frequency, also use:

```json
"frequencyRangeHz": {
  "minimum": 0.0,
  "maximum": 0.0
}
```

Disabled measurements are sent as `null`. Do not substitute a spectrum-energy
register, sample rate, FFT-bin number, or transfer rate.

## Step 11: Set the polling interval

`minimumPollIntervalMs` prevents polling faster than the sensor can update.

Examples:

- 100 ms update time: use at least `100`
- 300 ms update time: use at least `300`
- `3 × 0.32 seconds`: use `960`

If the manual gives no update time, start conservatively and verify with the
manufacturer.

## Step 12: Leave bulk reading disabled at first

The safest initial setting is:

```json
"bulkRead": {
  "enabled": false,
  "function": 3,
  "startAddress": 0,
  "registerCount": 0
}
```

Enable bulk reading only when one documented contiguous request contains every
enabled field and the physical sensor answers it reliably.

## Optional safety checks

Use `invalidRaw` when the manual defines an unavailable-data code:

```json
"invalidRaw": "0xFFFF"
```

Use `validRange` when the manual gives a trustworthy engineering range:

```json
"validRange": {
  "minimum": -40.0,
  "maximum": 150.0
}
```

Use quoted decimal or hexadecimal strings for 64-bit masks and invalid values.

## Final test checklist

Before shipping:

1. Validate that the JSON opens without a syntax error.
2. Upload the profile through `http://192.168.4.1`.
3. Download the active profile and compare it with the source file.
4. Compare every raw register with a trusted Modbus program.
5. Hand-calculate at least one value for every measurement type.
6. Check temperature against the approximate room temperature.
7. Check that a stopped machine produces believable values.
8. Check that unavailable groups appear as `null`, not zero.
9. Test at least ten power cycles.
10. Confirm that the profile remains active and `profileStorageHealthy` is true.
11. Run for several hours while watching CRC failures, timeouts, and free heap.
12. Save the exact manual, profile, firmware version, and device serial in the
    production record.

## Stop and request a firmware change when

Do not force a profile to fit if the sensor requires:

- Modbus ASCII instead of RTU;
- function codes other than 03 or 04 for measurements;
- a write command, unlock command, or bank switch before every read;
- a value assembled from non-consecutive registers;
- a nonlinear equation, lookup table, or compensation using another register;
- a custom checksum inside the register data;
- strings, variable-length data, or a proprietary binary packet;
- missing acceleration, velocity, or temperature.

The complete technical convention is in `SENSOR_PROFILES.md`, and the
machine-readable rules are in `sensor_profiles/schema-v3.json`.
