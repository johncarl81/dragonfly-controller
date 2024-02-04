#! /usr/bin/env python3
import argparse, subprocess, time, tempfile, math

def run_controllers(args):
    processes = []

    # Start dragonfly controllers
    for i in range(0, args.drones):
        name = f"dragonfly{i + 1}"

        pump_log = open(f"/workspace/logs/{name}_pump.log", "a")
        run_log = open(f"/workspace/logs/{name}_run.log", "a")

        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly announce {name}", shell=True))
        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly command {name}", shell=True))
        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly virtualco2publisher {name}", shell=True))
        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly virtualRangefinder {name}", shell=True))
        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly pump {name} --sim", shell=True, stdout=pump_log, stderr=subprocess.STDOUT))
        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly logger {name} --sim", shell=True, stdout=run_log, stderr=subprocess.STDOUT))

    for p in processes:
        p.wait()

def get_args():
    parser = argparse.ArgumentParser(description='Dragonfly controller')

    parser.add_argument(
        '--drones',
        type=int,
        default=1)

    args = parser.parse_args()
    return args

def main():
    args = get_args()
    run_controllers(args)

if __name__ == '__main__':
    main()
