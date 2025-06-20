#!/bin/bash
# $1 - ip groundstation $2 - bitrate $3 - duration
iperf3 -u -c $1 -b $2 -t $3 -f k -V | tee iperf_$(($(date +%s%N)/1000000)).txt &