import argparse
import os
import shlex
import shutil
import subprocess
import sys


DEFAULT_ARDUPILOT_DIR = "/home/ammar/ardu_ws/src/ardupilot"
DEFAULT_SIM_VEHICLE = (
    "/home/ammar/ardu_ws/src/ardupilot/Tools/autotest/sim_vehicle.py"
)


def build_sitl_command(args):
    parts = [
        "cd",
        shlex.quote(args.ardupilot_dir),
        "&&",
        "python3",
        shlex.quote(args.sim_vehicle),
        "-v",
        "Rover",
        "-f",
        "motorboat-skid",
        "--model",
        "JSON",
        "--no-rebuild",
        "--no-extra-ports",
        "--out",
        "udp:127.0.0.1:14550",
        "--out",
        "udp:127.0.0.1:14551",
        "--add-param-file",
        shlex.quote(args.param_file),
    ]
    return " ".join(parts)


def terminal_command(title, shell_command):
    keep_open = (
        shell_command
        + "; status=$?; echo; "
        + "echo 'ArduPilot SITL selesai dengan status '$status'.'; "
        + "echo 'Tutup terminal ini jika simulasi sudah selesai.'; exec bash"
    )
    candidates = [
        ["gnome-terminal", "--title", title, "--wait", "--", "bash", "-lc", keep_open],
        ["x-terminal-emulator", "-T", title, "-e", "bash", "-lc", keep_open],
        ["xterm", "-T", title, "-e", "bash", "-lc", keep_open],
        ["konsole", "--new-tab", "-p", f"tabtitle={title}", "-e", "bash", "-lc", keep_open],
    ]
    for command in candidates:
        if shutil.which(command[0]):
            return command
    return None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Open ArduRover SITL in a separate terminal for the ASV sim."
    )
    parser.add_argument("--param-file", required=True)
    parser.add_argument("--ardupilot-dir", default=DEFAULT_ARDUPILOT_DIR)
    parser.add_argument("--sim-vehicle", default=DEFAULT_SIM_VEHICLE)
    parser.add_argument("--no-terminal", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    sitl_command = build_sitl_command(args)

    if not os.path.exists(args.sim_vehicle):
        print(f"sim_vehicle.py tidak ditemukan: {args.sim_vehicle}", file=sys.stderr)
        return 2
    if not os.path.exists(args.param_file):
        print(f"File parameter ArduPilot tidak ditemukan: {args.param_file}", file=sys.stderr)
        return 2

    if args.no_terminal:
        return subprocess.call(["bash", "-lc", sitl_command])

    command = terminal_command("ArduPilot SITL - Gamantaray ASV", sitl_command)
    if command is None:
        print("Terminal GUI tidak ditemukan; menjalankan SITL di terminal launch.")
        return subprocess.call(["bash", "-lc", sitl_command])

    return subprocess.call(command)


if __name__ == "__main__":
    sys.exit(main())
