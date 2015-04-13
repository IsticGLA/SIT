#!/bin/sh
pkill python3
nohup python3 simulation/simulation.py > /sit/log/simulation_ws.log 2>&1 &


