add_subdirectory(agents)

cmake_minimum_required(VERSION 3.0)
project(learning)

set(LIB_DIR ../../../../itandroids-lib/source)
set(SOURCE_DIR ../../../source)
set(RL_AGENTS_DIR rl_agents)
set(EXTERNAL_DIR external)
set(CODEGEN_DIR codegen)
set(CONFIG_DIR config)
set(UTILS_DIR utils)

# Debugging purposes
set( CMAKE_VERBOSE_MAKEFILE on )
set(GRPC_LINK -lprotobuf -lgrpc++ -lgrpc -lz)

set(CMAKE_EXE_LINKER_FLAGS -static)
set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
set(CMAKE_EXE_LINK_DYNAMIC_C_FLAGS)       # remove -Wl,-Bdynamic
set(CMAKE_EXE_LINK_DYNAMIC_CXX_FLAGS)
set(CMAKE_SHARED_LIBRARY_C_FLAGS)         # remove -fPIC
set(CMAKE_SHARED_LIBRARY_CXX_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)    # remove -rdynamic
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

find_package(Protobuf REQUIRED)

include_directories(${SOURCE_DIR})
include_directories(.)
#include_directories(${EXTERNAL_DIR})
include_directories(${RL_AGENTS_DIR})
include_directories(${CODEGEN_DIR})
include_directories(${UTILS_DIR})

file(GLOB LEARNING_SRCS "*.cpp" "*.cc")
file(GLOB RL_SRCS ${RL_AGENTS_DIR}/*.cpp)
file(GLOB UTILS_SRCS ${UTILS_DIR}/*.cpp)
file(GLOB EXTERNAL_SRCS ${EXTERNAL_DIR}/*.cc ${EXTERNAL_DIR}/*.cpp)
file(GLOB CODEGEN_SRCS ${CODEGEN_DIR}/*.cc ${CODEGEN_DIR}/*.cpp)


#message(WARNING ${CODEGEN_SRCS})
add_executable(SoccerAgentServer_Main ${LEARNING_SRCS} ${RL_SRCS} ${EXTERNAL_SRCS} ${CODEGEN_SRCS} ${UTILS_SRCS})
target_link_libraries(SoccerAgentServer_Main itandroids-lib-library test_source core ${GRPC_LINK}  "-static")


# Add .txts files to binaries/
#file(GLOB AGENT_TXTS "*.conf" "*.txt" "config/*.json")
#foreach (agent_txt ${AGENT_TXTS})
#    add_custom_command(TARGET SoccerAgentServer_Main
#            PRE_BUILD
#            COMMAND ${CMAKE_COMMAND} -E copy
#            ${agent_txt} $<TARGET_FILE_DIR:SoccerAgentServer_Main>/config)
#    #message(WARNING ${agent_txt})
#endforeach ()

# Add starting script to binaries/
add_custom_command(TARGET SoccerAgentServer_Main
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/start_training.sh
        $<TARGET_FILE_DIR:SoccerAgentServer_Main>)

# Move config folder to binaries/
add_custom_command(TARGET SoccerAgentServer_Main
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_DIR}
        $<TARGET_FILE_DIR:SoccerAgentServer_Main>/${CONFIG_DIR})