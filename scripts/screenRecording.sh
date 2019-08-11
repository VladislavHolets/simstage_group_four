#!/bin/bash
recordmydesktop --on-the-fly-encoding switch -o ~/Documents/records/reactive.ogv &
PID=$!
sleep 1800
kill -INT $PID
