#!/usr/bin/env python3
import yaml
from yaml.loader import SafeLoader

def get_command(path):
    print("obtener las misiones para ser usadas y comparadas con la que envian")
    if not (path is None):
        with open(path, 'r') as f:
            list_command = yaml.load(f, Loader=SafeLoader)

            return list_command
    else:
        return None 

if __name__ == "__main__":
    get_command(None)