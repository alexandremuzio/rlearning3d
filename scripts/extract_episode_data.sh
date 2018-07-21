#!/usr/bin/env bash

FILE_IN=$1
FILE_OUT=$2

if [ "$#" -lt 2 ]; then
    echo "Usage: ./extract_episode_data.sh <file_in> <file_out>"
    exit 1
fi


# Extracting y-speed (6th column)
cat ${FILE_IN} | awk '/Episode: 1/ {p=1}; p; /Episode: 2/ {p=0}' | grep 'Speed' | awk  '{print $6}' > ${FILE_OUT}