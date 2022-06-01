# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

find_package(Threads REQUIRED)

add_executable(stream_server main.c rvl_compress.c)
target_link_libraries(stream_server PUBLIC
cJSON::cJSON
PRIVATE k4a::k4a
    )

target_link_libraries(stream_server PRIVATE Threads::Threads
    )
