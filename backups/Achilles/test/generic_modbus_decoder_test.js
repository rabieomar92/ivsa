"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");

const typeBytes = {
  uint16: 2, int16: 2, uint32: 4, int32: 4,
  uint64: 8, int64: 8, float32: 4, float64: 8,
  bcd16: 2, bcd32: 4,
  uint32_mod10000: 4, int32_mod10000: 4,
  boolean: 2
};

function canonicalBytes(wire, order) {
  assert.equal(wire.length, order.length);
  const canonical = Buffer.alloc(wire.length);
  const seen = new Set();
  for (let i = 0; i < order.length; i++) {
    const target = order.charCodeAt(i) - 65;
    assert.ok(target >= 0 && target < order.length);
    assert.equal(seen.has(target), false);
    seen.add(target);
    canonical[target] = wire[i];
  }
  return canonical;
}

function rawBigInt(bytes) {
  let raw = 0n;
  for (const byte of bytes) raw = (raw << 8n) | BigInt(byte);
  return raw;
}

function signExtend(raw, bits) {
  return BigInt.asIntN(bits, raw);
}

function decode(field, wire) {
  const bytes = canonicalBytes(Buffer.from(wire), field.byteOrder);
  const bits = bytes.length * 8;
  const raw = rawBigInt(bytes);
  if (field.invalidRaw !== undefined &&
      raw === BigInt(field.invalidRaw)) {
    return {valid: false, raw};
  }
  let transformed = raw;
  if (field.bitField) {
    transformed =
      (raw & BigInt(field.bitField.mask)) >> BigInt(field.bitField.shift);
  }

  let numeric;
  if (field.bitField?.signed) {
    const shiftedMask =
      BigInt(field.bitField.mask) >> BigInt(field.bitField.shift);
    const width = shiftedMask.toString(2).length;
    numeric = Number(signExtend(transformed, width));
  } else {
    switch (field.dataType) {
      case "uint16":
      case "uint32":
      case "uint64":
        numeric = Number(transformed);
        break;
      case "int16":
      case "int32":
      case "int64":
        numeric = Number(signExtend(transformed, bits));
        break;
      case "float32":
        numeric = bytes.readFloatBE(0);
        break;
      case "float64":
        numeric = bytes.readDoubleBE(0);
        break;
      case "bcd16":
      case "bcd32": {
        let multiplier = 1;
        let value = 0;
        let work = raw;
        for (let i = 0; i < bytes.length * 2; i++) {
          const digit = Number(work & 0xfn);
          if (digit > 9) return {valid: false, raw};
          value += digit * multiplier;
          multiplier *= 10;
          work >>= 4n;
        }
        numeric = value;
        break;
      }
      case "uint32_mod10000": {
        const high = Number((raw >> 16n) & 0xffffn);
        const low = Number(raw & 0xffffn);
        if (low > 9999) return {valid: false, raw};
        numeric = high * 10000 + low;
        break;
      }
      case "int32_mod10000": {
        const high = Number(signExtend((raw >> 16n) & 0xffffn, 16));
        const low = Number(signExtend(raw & 0xffffn, 16));
        if (low < -9999 || low > 9999) return {valid: false, raw};
        numeric = high * 10000 + low;
        break;
      }
      case "boolean":
        numeric = transformed === 0n ? 0 : 1;
        break;
      default:
        throw new Error(`Unsupported test datatype ${field.dataType}`);
    }
  }
  const value = numeric * field.scale + field.offset;
  const valid = Number.isFinite(value) &&
    (!field.validRange ||
      (value >= field.validRange.minimum &&
       value <= field.validRange.maximum));
  return {valid, raw, value};
}

const base = {scale: 1, offset: 0};
assert.equal(decode({...base, dataType: "uint16", byteOrder: "AB"}, [0x12, 0x34]).value, 0x1234);
assert.equal(decode({...base, dataType: "uint16", byteOrder: "BA"}, [0x34, 0x12]).value, 0x1234);
assert.equal(decode({...base, dataType: "int16", byteOrder: "AB"}, [0xff, 0x9c]).value, -100);

for (const [order, wire] of Object.entries({
  ABCD: [0x12, 0x34, 0x56, 0x78],
  CDAB: [0x56, 0x78, 0x12, 0x34],
  BADC: [0x34, 0x12, 0x78, 0x56],
  DCBA: [0x78, 0x56, 0x34, 0x12]
})) {
  assert.equal(
    decode({...base, dataType: "uint32", byteOrder: order}, wire).value,
    0x12345678
  );
}

assert.equal(
  decode({...base, dataType: "float32", byteOrder: "CDAB"},
    [0x00, 0x00, 0x3f, 0x80]).value,
  1
);
assert.equal(
  decode({...base, dataType: "float32", byteOrder: "ABCD"},
    [0x7f, 0xc0, 0x00, 0x00]).valid,
  false
);
const pi = Buffer.alloc(8);
pi.writeDoubleBE(Math.PI);
assert.equal(
  decode({...base, dataType: "float64", byteOrder: "ABCDEFGH"}, pi).value,
  Math.PI
);
assert.equal(
  decode({...base, dataType: "int32", byteOrder: "ABCD"},
    [0xff, 0xff, 0xcf, 0xc7]).value,
  -12345
);
assert.equal(
  decode({...base, dataType: "int64", byteOrder: "ABCDEFGH"},
    [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xc7]).value,
  -12345
);
assert.equal(
  decode({...base, dataType: "bcd16", byteOrder: "AB"}, [0x12, 0x34]).value,
  1234
);
assert.equal(
  decode({...base, dataType: "bcd16", byteOrder: "AB"}, [0x1a, 0x34]).valid,
  false
);
assert.equal(
  decode({...base, dataType: "uint32_mod10000", byteOrder: "ABCD"},
    [0x00, 0x0c, 0x0d, 0x80]).value,
  123456
);
assert.equal(
  decode({...base, dataType: "int32_mod10000", byteOrder: "ABCD"},
    [0xff, 0xff, 0xfc, 0x18]).value,
  -11000
);
assert.equal(
  decode({...base, dataType: "boolean", byteOrder: "AB"}, [0x00, 0x01]).value,
  1
);
assert.equal(
  decode({...base, dataType: "boolean", byteOrder: "AB"}, [0x00, 0x00]).value,
  0
);
assert.equal(
  decode({
    ...base, dataType: "uint16", byteOrder: "AB",
    bitField: {mask: "0x0F00", shift: 8, signed: false}
  }, [0xaf, 0x00]).value,
  15
);
assert.equal(
  decode({
    ...base, dataType: "uint16", byteOrder: "AB",
    bitField: {mask: "0x0FFF", shift: 0, signed: true}
  }, [0x0f, 0x9c]).value,
  -100
);
assert.equal(
  decode({...base, dataType: "uint16", byteOrder: "AB", invalidRaw: "0xFFFF"},
    [0xff, 0xff]).valid,
  false
);
assert.equal(
  decode({
    dataType: "uint16", byteOrder: "AB", scale: 0.1, offset: -40,
    validRange: {minimum: -40, maximum: 85}
  }, [0x04, 0xe2]).value,
  85
);
assert.equal(
  decode({
    dataType: "uint16", byteOrder: "AB", scale: 0.25, offset: -10
  }, [0x00, 0x28]).value,
  0
);

function permutations(text) {
  if (text.length <= 1) return [text];
  const result = [];
  for (let i = 0; i < text.length; i++) {
    for (const suffix of permutations(text.slice(0, i) + text.slice(i + 1))) {
      result.push(text[i] + suffix);
    }
  }
  return result;
}

for (const order of permutations("ABCD")) {
  const canonical = Buffer.from([0x12, 0x34, 0x56, 0x78]);
  const wire = Buffer.from([...order].map(letter => canonical[letter.charCodeAt(0) - 65]));
  assert.equal(
    decode({...base, dataType: "uint32", byteOrder: order}, wire).value,
    0x12345678,
    `32-bit permutation ${order} decoded incorrectly`
  );
}

function bitsPerCharacter(serialFormat) {
  if (serialFormat === "8N1") return 10;
  if (serialFormat === "8E2" || serialFormat === "8O2") return 12;
  return 11;
}
assert.equal(bitsPerCharacter("8N1"), 10);
assert.equal(bitsPerCharacter("8N2"), 11);
assert.equal(bitsPerCharacter("8E1"), 11);
assert.equal(bitsPerCharacter("8E2"), 12);
const worstCaseResponseMs = Math.ceil((255 * 12 * 1000) / 300) + 150;
assert.ok(worstCaseResponseMs < 15000);

const projectRoot = path.resolve(__dirname, "..");
for (const filename of [
  "witmotion_wtvb01.json",
  "bx2601a_wz01m.json",
  "bydr3016zw.json",
  "banner_qm30vt3.json"
]) {
  const sensorProfile = JSON.parse(
    fs.readFileSync(path.join(projectRoot, "sensor_profiles", filename), "utf8")
  );
  assert.equal(sensorProfile.schemaVersion, 3);
  assert.equal(sensorProfile.modbus.mode, "rtu");
  assert.equal(sensorProfile.modbus.addressConvention, "pdu_zero_based");
  assert.ok(sensorProfile.profileKey.length >= 1 && sensorProfile.profileKey.length <= 31);
  assert.ok(sensorProfile.sensorType.length >= 1 && sensorProfile.sensorType.length <= 47);
  assert.ok(sensorProfile.modelName.length >= 1 && sensorProfile.modelName.length <= 63);
  assert.ok(
    sensorProfile.registerMapVersion.length >= 1 &&
    sensorProfile.registerMapVersion.length <= 31
  );
  assert.deepEqual(
    ["x", "y", "z"].map(axis =>
      sensorProfile.measurements.acceleration[axis].enabled),
    [true, true, true]
  );
  assert.deepEqual(
    ["x", "y", "z"].map(axis =>
      sensorProfile.measurements.velocity[axis].enabled),
    [true, true, true]
  );
  assert.equal(sensorProfile.measurements.temperature.value.enabled, true);
  for (const optionalGroup of ["displacement", "frequency"]) {
    const enabled = ["x", "y", "z"].map(axis =>
      sensorProfile.measurements[optionalGroup][axis].enabled);
    assert.ok(
      enabled.every(Boolean) || enabled.every(value => !value),
      `${filename} ${optionalGroup} must enable all axes or none`
    );
  }
  if (!sensorProfile.measurements.frequency.x.enabled) {
    assert.deepEqual(sensorProfile.frequencyRangeHz, {minimum: 0, maximum: 0});
  }
  for (const measurement of ["acceleration", "velocity", "displacement", "frequency"]) {
    for (const axis of ["x", "y", "z"]) {
      const field = sensorProfile.measurements[measurement][axis];
      if (!field.enabled) {
        assert.ok(
          measurement === "frequency" || measurement === "displacement",
          `${filename} disables a required measurement`
        );
        continue;
      }
      assert.equal(field.registerCount * 2, typeBytes[field.dataType]);
      assert.equal(field.byteOrder.length, typeBytes[field.dataType]);
      assert.equal(
        new Set(field.byteOrder).size,
        field.byteOrder.length,
        `${filename} ${measurement}.${axis} byteOrder repeats a byte`
      );
      for (let i = 0; i < field.byteOrder.length; i++) {
        assert.equal(
          field.byteOrder.includes(String.fromCharCode(65 + i)),
          true,
          `${filename} ${measurement}.${axis} byteOrder is not a permutation`
        );
      }
      assert.ok([3, 4].includes(field.function));
      assert.ok(field.address + field.registerCount <= 65536);
    }
  }
  const temperature = sensorProfile.measurements.temperature.value;
  assert.equal(temperature.registerCount * 2, typeBytes[temperature.dataType]);
  assert.equal(new Set(temperature.byteOrder).size, temperature.byteOrder.length);
  assert.ok(sensorProfile.registerMapVersion.length <= 31);
  assert.ok(sensorProfile.modbus.maximumReadRegisters >= 1);
  assert.ok(sensorProfile.modbus.maximumReadRegisters <= 125);
  if (sensorProfile.bulkRead.enabled) {
    assert.ok(sensorProfile.bulkRead.registerCount >= 1);
    assert.ok(
      sensorProfile.bulkRead.registerCount <=
      sensorProfile.modbus.maximumReadRegisters
    );
  } else {
    assert.equal(sensorProfile.bulkRead.registerCount, 0);
  }
}

const bydr = JSON.parse(
  fs.readFileSync(
    path.join(projectRoot, "sensor_profiles", "bydr3016zw.json"), "utf8"
  )
);
assert.deepEqual(
  ["x", "y", "z"].map(axis => bydr.measurements.velocity[axis].address),
  [0, 1, 2]
);
assert.deepEqual(
  ["x", "y", "z"].map(axis => bydr.measurements.acceleration[axis].address),
  [4, 5, 6]
);
assert.deepEqual(
  ["x", "y", "z"].map(axis => bydr.measurements.displacement[axis].address),
  [19, 20, 21]
);
assert.deepEqual(
  ["x", "y", "z"].map(axis => bydr.measurements.frequency[axis].enabled),
  [false, false, false]
);
assert.equal(bydr.measurements.temperature.value.address, 3);
assert.equal(bydr.measurements.temperature.value.dataType, "int16");
assert.equal(bydr.bulkRead.enabled, false);
assert.equal(bydr.acquisition.minimumPollIntervalMs, 960);

const banner = JSON.parse(
  fs.readFileSync(
    path.join(projectRoot, "sensor_profiles", "banner_qm30vt3.json"), "utf8"
  )
);
assert.equal(banner.modbus.baud, 19200);
assert.equal(banner.modbus.serialFormat, "8N1");
assert.deepEqual(
  ["x", "y", "z"].map(axis => banner.measurements.acceleration[axis].address),
  [33, 34, 35]
);
assert.deepEqual(
  ["x", "y", "z"].map(axis => banner.measurements.velocity[axis].address),
  [36, 38, 40]
);
assert.deepEqual(
  ["x", "y", "z"].map(axis => banner.measurements.frequency[axis].address),
  [25, 26, 27]
);
assert.deepEqual(
  ["x", "y", "z"].map(axis =>
    banner.measurements.displacement[axis].enabled),
  [false, false, false]
);
assert.equal(banner.measurements.temperature.value.address, 42);
assert.equal(banner.measurements.acceleration.x.scale, 0.001 * 9.80665);
assert.equal(banner.measurements.velocity.x.scale, 0.001);
assert.equal(banner.measurements.frequency.x.scale, 0.1);
assert.equal(banner.measurements.temperature.value.scale, 0.01);
assert.equal(banner.bulkRead.enabled, false);
assert.equal(banner.acquisition.minimumPollIntervalMs, 300);

console.log("Generic Modbus decoder vectors passed.");
