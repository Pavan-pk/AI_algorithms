#!/bin/bash

# Execute this script to kill all processes related to hw2!
#
# WARNING: This will kill any process with hw2 in the string. We hope that its
# only cse471 related, but if you suspect otherwise then do NOT run this script.

killall gzserver
killall gzclient
pkill -f "python .*hw2.*" --signal sigkill
pkill -f roscore
