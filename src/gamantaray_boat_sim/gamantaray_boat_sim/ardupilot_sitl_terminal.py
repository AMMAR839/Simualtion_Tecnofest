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
DEFAULT_VEHICLE_BINARY = "/home/ammar/ardu_ws/src/ardupilot/build/sitl/bin/ardurover"
DEFAULT_HOME = "-6.200000,106.816666,0,0"


def ardupilot_defaults(args):
    defaults = [
        "Tools/autotest/default_params/rover.parm",
        "Tools/autotest/default_params/motorboat.parm",
        "Tools/autotest/default_params/rover-skid.parm",
        args.param_file,
    ]
    return ",".join(shlex.quote(item) for item in defaults)


def build_direct_sitl_command(args):
    sim_port_in = int(args.sim_port_in) + (int(args.instance) * 10)
    sim_port_out = int(args.sim_port_out) + (int(args.instance) * 10)
    return " ".join(
        [
            "cd",
            shlex.quote(args.ardupilot_dir),
            "&&",
            shlex.quote(args.vehicle_binary),
            "--model",
            "JSON",
            "--speedup",
            shlex.quote(str(args.speedup)),
            "--home",
            shlex.quote(args.home),
            "--slave",
            "0",
            "--defaults",
            ardupilot_defaults(args),
            "--sim-address=127.0.0.1",
            "--sim-port-in",
            shlex.quote(str(sim_port_in)),
            "--sim-port-out",
            shlex.quote(str(sim_port_out)),
            f"-I{int(args.instance)}",
        ]
    )


def build_sitl_command(args):
    if not args.use_sim_vehicle:
        return build_direct_sitl_command(args)

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
    parser.add_argument("--vehicle-binary", default=DEFAULT_VEHICLE_BINARY)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--speedup", type=float, default=1.0)
    parser.add_argument(
        "--home",
        default=DEFAULT_HOME,
        help="SITL home as lat,lon,alt,heading. Default matches the TEKNOFEST world origin.",
    )
    parser.add_argument(
        "--sim-port-in",
        type=int,
        default=9003,
        help="ArduPilot JSON sensor input port. Instance offset is added automatically.",
    )
    parser.add_argument(
        "--sim-port-out",
        type=int,
        default=9002,
        help="ArduPilot JSON servo output port. Instance offset is added automatically.",
    )
    parser.add_argument(
        "--use-sim-vehicle",
        action="store_true",
        help="Use sim_vehicle.py instead of launching the ArduRover binary directly.",
    )
    parser.add_argument("--no-terminal", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    args.param_file = os.path.abspath(args.param_file)
    args.ardupilot_dir = os.path.abspath(args.ardupilot_dir)
    args.sim_vehicle = os.path.abspath(args.sim_vehicle)
    args.vehicle_binary = os.path.abspath(args.vehicle_binary)

    if args.use_sim_vehicle and not os.path.exists(args.sim_vehicle):
        print(f"sim_vehicle.py tidak ditemukan: {args.sim_vehicle}", file=sys.stderr)
        return 2
    if not args.use_sim_vehicle and not os.path.exists(args.vehicle_binary):
        print(f"Binary ArduRover tidak ditemukan: {args.vehicle_binary}", file=sys.stderr)
        return 2
    if not os.path.exists(args.param_file):
        print(f"File parameter ArduPilot tidak ditemukan: {args.param_file}", file=sys.stderr)
        return 2

    sitl_command = build_sitl_command(args)

    if args.no_terminal:
        return subprocess.call(["bash", "-lc", sitl_command])

    command = terminal_command("ArduPilot SITL - Gamantaray ASV", sitl_command)
    if command is None:
        print("Terminal GUI tidak ditemukan; menjalankan SITL di terminal launch.")
        return subprocess.call(["bash", "-lc", sitl_command])

    return subprocess.call(command)


if __name__ == "__main__":
    sys.exit(main())
