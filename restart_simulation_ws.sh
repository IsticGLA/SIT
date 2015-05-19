#!/bin/sh
pkill python3
if [ ! -f /sit/log/flask.log ]
then
    mkdir /sit/log
    touch /sit/log/flask.log
fi
nohup python3 simulation/simulation.py > /sit/log/flask.log 2>&1 &


