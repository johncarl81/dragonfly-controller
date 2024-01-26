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

    dragonfly_dir = "/home/ubuntu/dev/dragonfly"

    env = os.environ.copy()
    if args.cyclone_network:
        cyclone_parameters = {'networkInterface': args.cyclone_network}
        cyclone_dds_config = template(f"{dragonfly_dir}/templates/cyclonedds.xml.template", cyclone_parameters)
        env['CYCLONEDDS_URI'] = f"file://{cyclone_dds_config.name}"

    mavros_parameters = {
        'name': args.name,
        'sysid_thismav': args.sysid_thismav,
    }
    mavros_params = template(f"{dragonfly_dir}/templates/mavros.launch.yaml.template", mavros_parameters)

    runLog = open(f"{dragonfly_dir}/logs/run.log", "a")
    pumpLog = open(f"{dragonfly_dir}/logs/pump.log", "a")
    commandLog = open(f"{dragonfly_dir}/logs/command.log", "a")

    processes.append(subprocess.Popen(f"ros2 daemon start", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run mavros mavros_node --ros-args -r __ns:=/{args.name}/mavros --params-file {mavros_params.name}", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run dragonfly virtualco2publisher {args.name}", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run dragonfly co2publisher {args.name}", env=env, shell=True))
    processes.append(subprocess.Popen(f"ros2 run dragonfly logger {args.name}", env=env, shell=True, stdout=runLog))
    processes.append(subprocess.Popen(f"ros2 run dragonfly pump {args.name}", env=env, shell=True, stdout=pumpLog))
    processes.append(subprocess.Popen(f"ros2 run dragonfly command {args.name}", env=env, shell=True, stdout=commandLog))
    processes.append(subprocess.Popen(f"ros2 run dragonfly announce {args.name}", env=env, shell=True))

    for p in processes:
        p.wait()

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
