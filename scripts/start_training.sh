#!/usr/bin/env bash
echo "### START LEARNING SOCCER ####"

if [ "$#" -lt 2 ]; then
    echo "Usage: ./start_training.sh <SERVER_PORT> <MONITOR_PORT> <TYPE>"
    exit 1
fi

SERVER_PORT=$1
MONITOR_PORT=$2
TYPE=$3

## Check if rcssserver3d is running
#if ! pgrep -x "rcssserver3d" > /dev/null; then
#    echo "rcssserver3d is not running! Exiting..."
#    exit 1
#fi


if [ ! -f SoccerAgentServer_Main ]; then
    echo "SoccerAgentServer_Main does not exist! Exiting..."
    exit 1
fi

if [ "$TYPE" != "single" ] && [ "$TYPE" != "multi" ]; then
  echo "TYPE must be \"single\" or \"multi\""
  exit 1
fi


# Cleans up background processes
trap 'kill -9 $(jobs -p)' EXIT

#echo "Starting Dummy Agent..."
#./DummyAgent_learning 1 "Opponent" &>/dev/null &
#sleep 1

echo "Starting SSERVER3D..."
rcssserver3d --agent-port $SERVER_PORT  --server-port $MONITOR_PORT &>/dev/null &

if [ "$TYPE" == "multi" ]; then
    ./FollowBallAgent_learning 1 "ITAndroids" &>/dev/null &
    last_pid=$!
    sleep 10
    ./FollowBallAgent_learning 1 "Opponent" &>/dev/null &
    kill $last_pid
fi

echo "Starting Learning Agent..."
./SoccerAgentServer_Main --server-port=$SERVER_PORT --monitor-port=$MONITOR_PORT
