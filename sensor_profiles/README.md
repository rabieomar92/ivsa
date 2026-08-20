# VibScope sensor profiles

VibScope accepts sensor profile schema version 3 only. Every deployable JSON
profile in this directory must validate against `schema-v3.json`.

Schema versions 1 and 2, the former `registers` layout, and the old
`sensorprof` NVS namespace are intentionally unsupported. Convert a profile to
schema 3 before uploading it; the controller will reject older files rather
than guessing their meaning.

The bundled profiles are:

- `banner_qm30vt3.json`
- `bx2601a_wz01m.json`
- `bydr3016zw.json`
- `witmotion_wtvb01.json`
