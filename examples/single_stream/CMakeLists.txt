# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

add_executable(single_stream main.c)
target_link_libraries(single_stream PUBLIC
cJSON::cJSON
PRIVATE k4a::k4a
    )
