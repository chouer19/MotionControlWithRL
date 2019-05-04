INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)
file(GLOB PROTOFILES *.proto)

message("*****************************")
PROTOBUF_GENERATE_PYTHON(PROTO_PY ${PROTOFILES})
#PROTOBUF_GENERATE_PYTHON(PROTO_PY any.proto)
PROTOBUF_GENERATE_PYTHON(ProtoSourcesPy any.proto)
