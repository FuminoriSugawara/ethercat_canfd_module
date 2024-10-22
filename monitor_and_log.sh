#!/bin/bash
platformio device monitor --baud 115200 | tee -a log/output.log