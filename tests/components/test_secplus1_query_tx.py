"""Run the checked Sec+1 query sender against a simulated bus and opener.

The sender is a template over the bus, so the production algorithm runs
unchanged on a simulated wired-AND line with interfering pulses and timing
stalls. A simulated opener UART decodes the line. These tests cannot establish
the real bus's rise time or how the real opener treats an abandoned frame.
"""

from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def query_tx_runner(tmp_path_factory):
    compiler = shutil.which("g++")
    assert compiler, "Native firmware tests require g++"
    executable = tmp_path_factory.mktemp("secplus1-query-tx") / "query-tx-test"
    result = subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-O2",
            "-g",
            "-fsanitize=undefined",
            "-fno-sanitize-recover=all",
            "-I",
            str(ROOT / "components" / "ratgdo"),
            str(ROOT / "tests" / "native" / "secplus1_query_tx_test.cpp"),
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
        "clean_frame_is_delivered",
        "timer_wraparound",
        "busy_line_is_left_alone",
        "frame_fits_between_periodic_pulses",
        "wide_periodic_pulses_never_make_a_command",
        "unchecked_sender_can_emit_press",
        "pulse_never_makes_a_command",
        "stall_never_makes_a_command",
        "stall_with_pulse_never_makes_a_command",
        "forced_abort_is_not_a_command",
        "abort_reports_pulse_width",
        "dead_output_is_reported",
        "slow_line_is_aborted",
    ],
)
def test_query_tx_scenario(query_tx_runner, scenario):
    result = subprocess.run(
        [str(query_tx_runner), scenario], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stdout + result.stderr
