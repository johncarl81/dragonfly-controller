#! /usr/bin/env python3
import argparse, subprocess, time, tempfile, math

def run_controllers(args):
    processes = []

    # Start dragonfly controllers
    for i in range(0, args.drones):

        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly announce dragonfly{i + 1}", shell=True))
        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly command dragonfly{i + 1}", shell=True))
        processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run dragonfly virtualco2publisher dragonfly{i+1}", shell=True))

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
