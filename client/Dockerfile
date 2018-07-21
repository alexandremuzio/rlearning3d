FROM gcr.io/tensorflow/tensorflow:latest-gpu-py3

WORKDIR /ddpg-humanoid

ADD . /ddpg-humanoid

RUN apt-get update && apt install mpich -y
RUN apt-get install cmake -y
RUN apt-get install zlib1g-dev -y

# Install any needed packages specified in requirements.txt
RUN cd /ddpg-humanoid/baselines && pip install -e .

RUN pip install --trusted-host pypi.python.org -r requirements.txt
