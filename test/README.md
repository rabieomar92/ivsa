# Athena firmware checks

Run the four JavaScript checks listed in the root README with Node.js. Compile
and run the two C++ LED checks with a C++11 compiler. `status_led_stubs/` supplies
host substitutes for Arduino and the ESP timer, so those checks need no board.

- `generic_modbus_decoder_test.js`: sensor profile decoding and numeric vectors.
- `mqtt_long_outage_recovery_test.js`: bounded retry and heartbeat recovery.
- `reliability_contract_test.js`: persistence, profiles, boot and runtime contracts.
- `live_status_page_test.js`: live setup status, errors, recovery and edit safety.
- `status_led_pattern_test.cpp`: pattern timing, priorities, transitions and rollover.
- `status_led_service_test.cpp`: independent timer, sensor fault/recovery and fallback.

The separate `src/main_2.cpp` diagnostic image is also a fixture for the reliability
check. It is excluded from the production firmware by `build_src_filter`.

The frozen archive has its original historical tests for provenance. Run the
current tests in this directory against Athena; historical tests expect older
firmware names and environments.
