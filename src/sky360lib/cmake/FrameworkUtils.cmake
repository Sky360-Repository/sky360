
# This file is part of the LITIV framework; visit the original repository at
# https://github.com/plstcharles/litiv for more information.
#
# Copyright 2015 Pierre-Luc St-Charles; pierre-luc.st-charles<at>polymtl.ca
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

macro(xfix_list_tokens list_name prefix suffix)
    set(${list_name}_TMP)
    foreach(l ${${list_name}})
        list(APPEND ${list_name}_TMP "${prefix}${l}${suffix}")
    endforeach(l ${${list_name}})
    set(${list_name} "${${list_name}_TMP}")
    unset(${list_name}_TMP)
endmacro(xfix_list_tokens)

macro(append_internal_list list_name value)
    if(${list_name})
        set(${list_name} "${${list_name}};${value}" CACHE INTERNAL "Internal list variable")
    else(NOT ${list_name})
        set(${list_name} "${value}" CACHE INTERNAL "Internal list variable")
    endif()
endmacro(append_internal_list)

macro(initialize_internal_list list_name)
    set(${list_name} "" CACHE INTERNAL "Internal list variable")
endmacro(initialize_internal_list)

macro(set_eval name)
    if(${ARGN})
        set(${name} 1)
    else(NOT ${ARGN})
        set(${name} 0)
    endif()
endmacro(set_eval)

macro(add_files list_name)
    foreach(filepath ${ARGN})
        list(APPEND ${list_name} "${filepath}")
    endforeach()
endmacro()

macro(get_subdirectory_list result dir)
    file(GLOB children RELATIVE ${dir} ${dir}/*)
    set(dirlisttemp "")
    foreach(child ${children})
        if(IS_DIRECTORY ${dir}/${child})
            list(APPEND dirlisttemp ${child})
        endif()
    endforeach(child ${children})
    set(${result} ${dirlisttemp})
endmacro(get_subdirectory_list result dir)

function(get_link_libraries output_list target)
    list(APPEND visited_targets ${target})
    get_target_property(target_libs ${target} INTERFACE_LINK_LIBRARIES)
    set(output_list_tmp "")
    if(NOT ("${target_libs}" STREQUAL "target_libs-NOTFOUND"))
        foreach(lib ${target_libs})
            if(TARGET ${lib})
                list(FIND visited_targets ${lib} visited)
                if(${visited} EQUAL -1)
                    get_link_libraries(linked_libs ${lib})
                    list(APPEND output_list_tmp ${linked_libs})
                endif()
            else()
                if(${lib} MATCHES "^-l")
                    string(SUBSTRING ${lib} 2 -1 lib)
                endif()
                string(STRIP ${lib} lib)
                list(FIND output_list_tmp ${lib} found_new)
                list(FIND output_list ${lib} found_old)
                if((${found_new} EQUAL -1) AND (${found_old} EQUAL -1))
                    list(APPEND output_list_tmp ${lib})
                endif()
            endif()
        endforeach()
    else()
        list(APPEND output_list_tmp ${target})
    endif()
    list(REMOVE_DUPLICATES output_list_tmp)
    set(visited_targets ${visited_targets} PARENT_SCOPE)
    set(${output_list} ${output_list_tmp} PARENT_SCOPE)
endfunction()
