# Sec+1 host regression tests

Run `python -m pytest tests/components/test_secplus1_native.py -v` with the
project's test dependencies and `g++` installed. The normal `tests/` suite
includes these scenarios. Compilation enables UndefinedBehaviorSanitizer.

The runner compiles the unchanged `secplus1.cpp`, its production headers,
UART parser, transmission queue, and callback implementation. It also extracts
and compiles the production component's command methods, callback registration,
and scheduler IDs. The extraction keeps those methods in sync with the code
under test without pulling the entire ESPHome peripheral stack into a host test.

`runtime.h` supplies a deterministic clock, keyed scheduler, UART, and GPIO stub.
`ratgdo.h` substitutes publishing and unrelated component fields. The opener in
`secplus1_test.cpp` answers queries, can ignore presses or inject noise, and
models both supported stop/reverse sequences. Protocol base-method stubs throw
if a test accidentally bypasses a Sec+1 override.

Coverage includes noisy missed commands, one-retry bounds, cancellation before
transmission, newer command intent, required release preservation, duplicate
suppression, timeout cleanup, bounded warm-up queues, fast polling, confirmed
direction changes, boot/deferred commands, passive-panel probe suppression, and
holding a press while the opener does not answer status queries (deaf windows,
garbled replies, the press deadline, gated retries, and direction sequences),
queries that read back from the bus as another byte, the idle probe rate, and
fast probing after a wall-panel press.

`python -m pytest tests/components/test_secplus1_query_tx.py -v` runs the
checked query sender (`secplus1_query_tx.h`) against a simulated wired-AND line
and a simulated opener UART. It sweeps interfering pulses and timing stalls
across the frame and asserts the opener never decodes a panel command. The
scenario suite above also runs with the checked sender enabled.

These tests exercise software ordering and timing. Apart from that line model,
they do not model electrical bus behavior, motor travel, obstruction hardware,
or opener wake-up behavior. The real bus's rise time and the real opener's
handling of an invalid frame are not established here.
Firmware builds and operator-supervised board tests remain necessary before
hardware deployment or marking an upstream PR ready.
