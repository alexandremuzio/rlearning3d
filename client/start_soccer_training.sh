#!/usr/bin/env bash

SOCCER_DIR=$HOME/dev/itandroids-soccer3d/binaries

if [ "$#" -lt 1 ]; then
    echo "Usage: ./start_training.sh <NUM_ITER>"
    exit 1
fi

NUM_ITER=$1

# Cleans up processes
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

for i in `seq 1 $NUM_ITER`
do
    echo "#######################"
    echo "Starting iteration $i"
    pushd $SOCCER_DIR
    $SOCCER_DIR/start_training.sh 3100 3200 multi &
    popd
    sleep 20
    python -m baselines.ppo1.run_soccer --num-timesteps=2000000 --save-model
    sleep 10
done