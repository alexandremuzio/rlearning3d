# Deep RL Framework

There are two components:
1. The client is the learning algorithms and runs in Python.
1. The soccer server serves as an interface between the client and the soccer simulator. This is the code contained in the tools/learning dir.

## Installation

#### Client (Python - training algorithms)

- Download algorithms repo [here](https://github.com/alexandremuzio/ddpg-humanoid)
- Follow repo installation in the README. Recommend installing it a virtual environment (conda). This will install tensorflow, etc...

## Running
 
#### Server (C++)
- Run _build_proto.sh_. This generates the grpc and protobuf classes inside codegen
- Build the SoccerAgentServer_Main target
- To run the server interface (from binaries folder):

```
./SoccerAgentServer_Main --server-port=3100 --monitor-port=320
```

- If the environment you are trying to learn requires another agent simply build and run the other agent as well.

#### Client (Python - training algorithms)
- Run `python run_codegen.py` to generate the grpc and protobuf classes in python.
- After running the server interface, choose which RL algorithm you would like to learn
- Run the _run_soccer.py_ script for the appropriate algorithm. For example, if you would like to run ppo1 algorithm, run the following command from any directory inside the baselines dir:

```
python -m baselines.ppo1.run_soccer
```

You should now see the agent appear on field.


## Additional information
- Check the oficial [baseline repo](https://github.com/openai/baselines) for some more instructions.
- There are a few rl agents (each for a different task) that you could try running. To change the rl task, change the agent unique_ptr in the SoccerAgentService.cpp to the one you are interested in running. 