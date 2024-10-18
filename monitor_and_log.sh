#!/bin/bash
platformio device monitor --port /dev/ttyUSB0 --baud 115200 | tee -a log/output.log