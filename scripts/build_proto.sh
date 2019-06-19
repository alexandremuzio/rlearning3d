#!/usr/bin/env bash

CODEGEN_DIR=../codegen
PROTO_DIR=../api

mkdir ../codegen &> /dev/null

echo "Generating Proto files..."
protoc --proto_path=${PROTO_DIR} --grpc_out=${CODEGEN_DIR} --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ${PROTO_DIR}/soccer3d.proto
protoc  --proto_path=${PROTO_DIR} --cpp_out=${CODEGEN_DIR} ${PROTO_DIR}/soccer3d.proto
