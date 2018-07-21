"""Generates protocol messages and gRPC stubs."""

from grpc_tools import protoc

protoc.main(
    (
        '',
        '-I../api',
        '--python_out=.',
        '--grpc_python_out=.',
        '../api/soccer3d.proto',
    )
)
