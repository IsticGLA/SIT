#!/bin/sh
pkill python3
if [ ! -f log/flask.log ]
then
    mkdir log
    touch simulation_ws.log
fi
nohup python3 simulation/simulation.py > log/flask.log 2>&1 &


