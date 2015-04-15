#!/bin/sh
pkill python3
if [ ! -f /sit/log/simulation_ws.log ]
then
    mkdir /sit/log
    touch simulation_ws.log
fi
nohup python3 simulation/simulation.py > /sit/log/simulation_ws.log 2>&1 &


