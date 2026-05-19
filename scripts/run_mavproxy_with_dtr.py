#!/usr/bin/env python3
import os
import runpy
import sys

import serial


class DTRSerial(serial.Serial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        try:
            self.setDTR(True)
        except Exception:
            pass


def main():
    serial.Serial = DTRSerial
    mavproxy_path = os.environ.get(
        "MAVPROXY_SCRIPT",
        os.path.expanduser("~/.local/lib/python3.8/site-packages/MAVProxy/mavproxy.py"),
    )
    sys.argv = [mavproxy_path] + sys.argv[1:]
    runpy.run_path(mavproxy_path, run_name="__main__")


if __name__ == "__main__":
    main()