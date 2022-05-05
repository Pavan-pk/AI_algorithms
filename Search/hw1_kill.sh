#!/bin/bash

# Execute this script to kill all processes related to hw1!
#
# WARNING: This will kill any process with hw1 in the string. We hope that its
# only CSE571 related, but if you suspect otherwise then do NOT run this script.

killall gzserver
killall gzclient
pkill -f "python .*hw1.*" --signal sigkill
pkill -f roscore
