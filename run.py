#! /usr/bin/env python3
import argparse
import subprocess
import tempfile
import os
from string import Template

def template(templateFileName, values):
    fp = tempfile.NamedTemporaryFile(mode = "w")

    template = Template(open(templateFileName).read())
    result = template.substitute(values)

    fp.write(result)

    fp.seek(0)
    return fp

def run(args):
    processes = []
    tempfiles = []

    dragonfly_dir = "/home/ubuntu/dev/dragonfly"

    env = os.environ.copy()
    if args.cyclone_network:
        parameters = {'networkInterface': args.cyclone_network}
        cyclone_dds_config = template(f"{dragonfly_dir}/templates/cyclonedds.xml.template", parameters)
        env['CYCLONEDDS_URI'] = f"file://{cyclone_dds_config.name}"

    processes.append(subprocess.Popen(f"ros2 daemon start", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 launch {dragonfly_dir}/config/apm.launch name:={args.name} tgt_system:={args.sysid_thismav} fcu_url:=/dev/ttyUSB0:921600", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run dragonfly co2publisher {args.name}", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run dragonfly logger {args.name} >> {dragonfly_dir}/logs/run.log", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run dragonfly command {args.name} >> {dragonfly_dir}/logs/command.log", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run dragonfly announce {args.name}", env=env, shell=True))

    for p in processes:
        p.wait()

    for file in tempfiles:
        file.close()

def get_args():
    parser = argparse.ArgumentParser(description='Dragonfly controller')

    parser.add_argument(
        '--name',
        type=str)
    parser.add_argument(
        '--sysid_thismav',
        type=int)
    parser.add_argument(
        '--cyclone_network',
        type=str,
        default=None)

    args = parser.parse_args()
    return args

def main():
    args = get_args()
    run(args)

if __name__ == '__main__':
    main()
