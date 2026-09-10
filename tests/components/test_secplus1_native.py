"""Run production Sec+1 C++ against a simulated clock, UART, and opener.

The source/header, queue, parser, callbacks, recovery, component command/state/
position methods, and cover control/mapping are real. ESPHome scheduling,
subscriber delivery, transport, and hardware are replaced; these tests
cannot establish electrical timing or real-opener compatibility.
"""

from pathlib import Path
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
NATIVE = ROOT / "tests" / "native"
COMPONENT = ROOT / "components" / "ratgdo"


def function_definition(source: str, pattern: str) -> str:
    match = re.search(pattern, source)
    assert match, pattern
    brace = source.index("{", match.end())
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start() : end]


@pytest.fixture(
    scope="session", params=[0, 1], ids=["diagnostics_off", "diagnostics_on"]
)
def native_runner(tmp_path_factory, request):
    compiler = shutil.which("g++")
    assert compiler, "Native firmware tests require g++"
    build = tmp_path_factory.mktemp("secplus1-native")
    # Relocate only the .cpp so its quoted ratgdo.h include selects the fake
    # component. All protocol headers and implementation bytes stay unchanged.
    shutil.copyfile(COMPONENT / "secplus1.cpp", build / "secplus1.cpp")
    for header in ("defines", "gpio", "hal", "helpers", "log", "optional", "scheduler"):
        target = build / "esphome" / "core" / f"{header}.h"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('#pragma once\n#include "runtime.h"\n')

    header = (COMPONENT / "ratgdo.h").read_text()
    ids = function_definition(header, r"namespace scheduler_ids\s*")
    (build / "scheduler_ids.inc").write_text(
        f"namespace esphome::ratgdo {{\n{ids}\n}}\n"
    )
    callback = function_definition(
        header, r"    template <typename F>\s+void on_door_state\(F&& callback\)"
    )
    (build / "register_callback.inc").write_text(callback)
    source = (COMPONENT / "ratgdo.cpp").read_text()
    # Preserve protocol-specific timeout preprocessing from production.
    start = source.index("// Door state updates")
    prefix = source[:start]
    conditional = prefix.rfind("#ifdef PROTOCOL_SECPLUSV1")
    constant = source.index(
        "static constexpr uint32_t DOOR_STATE_CALLBACK_TIMEOUT", start
    )
    end = source.index(";", constant) + 1
    if conditional > source.index("static constexpr int SYNC_DELAY"):
        start, end = conditional, source.index("#endif", end) + len("#endif")
    methods = [source[start:end]]
    methods.extend(
        function_definition(source, rf"void RATGDOComponent::{name}\(")
        for name in (
            "door_open",
            "door_close",
            "door_stop",
            "door_toggle",
            "door_action",
            "set_door_state_expiry",
            "cancel_door_state_expiry",
            "door_move_to_position",
            "schedule_door_position_sync",
            "door_position_update",
            "cancel_position_sync_callbacks",
            "cancel_position_target_",
            "update_position_target_",
        )
    )
    methods.append(
        function_definition(
            source,
            r"void RATGDOComponent::received\(const DoorState door_state, uint32_t observed_at\)",
        )
    )
    cover_source = (COMPONENT / "cover" / "ratgdo_cover.cpp").read_text()
    methods.extend(
        function_definition(cover_source, rf"void RATGDOCover::{name}\(")
        for name in ("control", "on_door_state")
    )
    (build / "component_methods.cpp").write_text(
        '#include "cover_stub.h"\nnamespace esphome::ratgdo {\nusing namespace scheduler_ids;\n'
        + "\n".join(methods)
        + "\n}\n"
    )
    executable = build / "secplus1-test"
    result = subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-O1",
            "-g",
            "-fno-rtti",
            "-fsanitize=undefined",
            "-fno-sanitize-recover=all",
            "-DPROTOCOL_SECPLUSV1",
            f"-DRATGDO_SECPLUS1_DIAGNOSTICS={request.param}",
            "-include",
            str(NATIVE / "runtime.h"),
            "-I",
            str(build),
            "-I",
            str(NATIVE),
            "-I",
            str(COMPONENT),
            str(build / "secplus1.cpp"),
            str(build / "component_methods.cpp"),
            str(NATIVE / "secplus1_test.cpp"),
            str(NATIVE / "protocol_stub.cpp"),
            "-o",
            str(executable),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return executable


@pytest.mark.parametrize(
    "scenario",
    [
        "miss_with_noise",
        "retry_stays_bounded",
        "movement_cancels_retry",
        "candidate_blocks_retry",
        "new_close_cancels_open",
        "stop_preserves_release",
        "duplicate_suppressed",
        "watch_timeout",
        "correction_polls_fast",
        "motion_polls_fast",
        "close_while_opening",
        "stop_while_closing",
        "open_from_stopped",
        "supersede_direction_sequence",
        "deferred_stop",
        "passive_panel",
        "warmup_queue_bounded",
        "boot_and_deferred_actions",
        "direction_waits_for_confirmation",
        "timeout_owners_independent",
        "opening_at_estimated_open",
        "slider_direction_mismatch",
        "slider_pending_target_does_not_change_estimator",
        "position_reversal_and_stop",
        "partial_open_stops_between_endpoints",
        "partial_stop_before_confirmation",
        "partial_timer_survives_new_command",
        "stopped_at_estimated_closed",
        "partial_accounts_for_confirmation_delay",
        "partial_waits_through_direction_sequence",
        "partial_latest_target_while_moving",
        "partial_unverified_expires",
        "partial_new_command_cancels_active_timer",
        "endpoints_require_status",
        "partial_rejects_untrusted_start",
        "partial_satisfied_cancels_queued_start",
        "partial_abort_preserves_release",
        "stale_candidate_does_not_backdate_position",
    ],
)
def test_secplus1_scenario(native_runner, scenario):
    result = subprocess.run(
        [str(native_runner), scenario], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stdout + result.stderr
