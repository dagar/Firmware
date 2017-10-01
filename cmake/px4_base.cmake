############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#=============================================================================
#
#	Defined functions in this file
#
# 	utility functions
#
#		* px4_parse_function_args
#		* px4_add_module
#		* px4_add_common_flags
#		* px4_add_library
#

include(CMakeParseArguments)

#=============================================================================
#
#	px4_parse_function_args
#
#	This function simplifies usage of the cmake_parse_arguments module.
#	It is intended to be called by other functions.
#
#	Usage:
#		px4_parse_function_args(
#			NAME <name>
#			[ OPTIONS <list> ]
#			[ ONE_VALUE <list> ]
#			[ MULTI_VALUE <list> ]
#			REQUIRED <list>
#			ARGN <ARGN>)
#
#	Input:
#		NAME		: the name of the calling function
#		OPTIONS		: boolean flags
#		ONE_VALUE	: single value variables
#		MULTI_VALUE	: multi value variables
#		REQUIRED	: required arguments
#		ARGN		: the function input arguments, typically ${ARGN}
#
#	Output:
#		The function arguments corresponding to the following are set:
#		${OPTIONS}, ${ONE_VALUE}, ${MULTI_VALUE}
#
#	Example:
#		function test()
#			px4_parse_function_args(
#				NAME TEST
#				ONE_VALUE NAME
#				MULTI_VALUE LIST
#				REQUIRED NAME LIST
#				ARGN ${ARGN})
#			message(STATUS "name: ${NAME}")
#			message(STATUS "list: ${LIST}")
#		endfunction()
#
#		test(NAME "hello" LIST a b c)
#
#		OUTPUT:
#			name: hello
#			list: a b c
#
function(px4_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			if (NOT "${OUT_${arg}}" STREQUAL "0")
				message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
			endif()
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()

#=============================================================================
#
#	px4_add_module
#
#	This function builds a static library from a module description.
#
#	Usage:
#		px4_add_module(MODULE <string>
#			[ MAIN <string> ]
#			[ STACK_MAIN <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ INCLUDES <list> ]
#			[ DEPENDS <string> ]
#			[ EXTERNAL ]
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point, if not given, assumed to be library
#		STACK_MAIN		: size of stack for main function
#		COMPILE_FLAGS		: compile flags
#		SRCS			: source files
#		INCLUDES		: include directories
#		DEPENDS			: targets which this module depends on
#		EXTERNAL		: flag to indicate that this module is out-of-tree
#
#	Output:
#		Static library with name matching MODULE.
#
#	Example:
#		px4_add_module(MODULE test
#			SRCS
#				file.cpp
#			DEPENDS
#				git_nuttx
#			)
#
function(px4_add_module)

	px4_parse_function_args(
		NAME px4_add_module
		ONE_VALUE MODULE MAIN STACK_MAIN PRIORITY
		MULTI_VALUE COMPILE_FLAGS SRCS INCLUDES DEPENDS
		OPTIONS EXTERNAL
		REQUIRED MAIN MODULE
		ARGN ${ARGN})

	add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})
	add_dependencies(${MODULE} prebuild_targets)
	set_property(GLOBAL APPEND PROPERTY PX4_LIBRARIES ${MODULE})

	# set defaults if not set
	set_target_properties(${MODULE} PROPERTIES MAIN ${MAIN})
	
	if(NOT STACK_MAIN)
		set(STACK_MAIN 1024)
	endif()
	set_target_properties(${MODULE} PROPERTIES STACK_MAIN ${STACK_MAIN})

	set_target_properties(${MODULE} PROPERTIES PRIORITY SCHED_PRIORITY_DEFAULT)

	target_compile_definitions(${MODULE}
			PRIVATE PX4_MAIN=${MAIN}_app_main
			PRIVATE MODULE_NAME="${MAIN}"
			)

	if(INCLUDES)
		target_include_directories(${MODULE} PRIVATE ${INCLUDES})
	endif()

	if(DEPENDS)
		add_dependencies(${MODULE} ${DEPENDS})
	endif()
	
	if(COMPILE_FLAGS)
		target_compile_options(${MODULE} PRIVATE ${COMPILE_FLAGS})
	endif()

endfunction()

#=============================================================================
#
#	px4_add_common_flags
#
#	Set the default build flags.
#
function(px4_add_common_flags)

	add_compile_options(
		# warnings
		-Wall
		-Wextra
		-Werror

		-Warray-bounds
		-Wdisabled-optimization
		-Wdouble-promotion
		-Wfatal-errors
		-Wfloat-equal
		-Wformat-security
		-Wformat=1
		-Winit-self
		-Wlogical-op
		-Wmissing-declarations
		-Wmissing-field-initializers
		-Wpointer-arith
		-Wshadow
		-Wuninitialized
		-Wunknown-pragmas
		-Wunused-but-set-variable
		-Wunused-variable
		#-Wmissing-include-dirs # TODO: fix and enable
		#-Wconversion

		# disabled warnings
		-Wno-unused-parameter

		# optimizations
		-fdata-sections
		-ffunction-sections
		-fno-common
		-fno-strict-aliasing
		-fomit-frame-pointer
		-funsafe-math-optimizations
		-fvisibility=hidden

		-include visibility.h
		)

	if (${CMAKE_C_COMPILER_ID} MATCHES ".*Clang.*")
		add_compile_options(
			-fcolor-diagnostics
			-Qunused-arguments
			-Wno-address-of-packed-member
			-Wno-unknown-warning-option
			-Wno-unused-const-variable
			-Wno-varargs
			-Wunused-but-set-variable
		)
	else()
		add_compile_options(
			-fdiagnostics-color=always
			-fno-builtin-printf
			-fno-strength-reduce
		)
	endif()

	set(c_flags
		-Wbad-function-cast
		-Wmissing-prototypes
		-Wnested-externs
		-Wstrict-prototypes
		)
	add_compile_options("$<$<COMPILE_LANGUAGE:C>:${c_flags}>")

	set(cxx_flags
		-Wreorder
		-Wno-missing-field-initializers
		
		-fcheck-new
		-fno-exceptions
		-fno-rtti
		-fno-threadsafe-statics

		-DCONFIG_WCHAR_BUILTIN
		-D__CUSTOM_FILE_IO__
		)
	add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${cxx_flags}>")

	# TODO: cleanup and start using INTERFACE_INCLUDE_DIRECTORIES
	include_directories(
		${PX4_BINARY_DIR}
		${PX4_BINARY_DIR}/src
		${PX4_BINARY_DIR}/src/modules

		${PX4_SOURCE_DIR}/src
		${PX4_SOURCE_DIR}/src/include
		${PX4_SOURCE_DIR}/src/include/platforms
		${PX4_SOURCE_DIR}/src/lib
		${PX4_SOURCE_DIR}/src/lib/DriverFramework/framework/include
		${PX4_SOURCE_DIR}/src/lib/log
		${PX4_SOURCE_DIR}/src/lib/matrix
		${PX4_SOURCE_DIR}/src/lib/systemlib
		${PX4_SOURCE_DIR}/src/modules
		)

	add_definitions(
		-D__STDC_FORMAT_MACROS
		)

endfunction()

#=============================================================================
#
#	px4_add_library
#
#	Like add_library but with optimization flag fixup.
#
function(px4_add_library target)
	add_library(${target} STATIC EXCLUDE_FROM_ALL ${ARGN})

	target_compile_definitions(${target} PRIVATE MODULE_NAME="${target}")

	add_dependencies(${target} prebuild_targets)

	set_property(GLOBAL APPEND PROPERTY PX4_LIBRARIES ${target})
endfunction()

#=============================================================================
#
#	px4_find_python_module
#
#	Find a required python module
#
#   Usage
#		px4_find_python_module(module_name [REQUIRED])
#
function(px4_find_python_module module)
	string(TOUPPER ${module} module_upper)
	if(NOT PY_${module_upper})
		if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
			set(PY_${module}_FIND_REQUIRED TRUE)
		endif()
		# A module's location is usually a directory, but for binary modules
		# it's a .so file.
		execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
			"import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
			RESULT_VARIABLE _${module}_status
			OUTPUT_VARIABLE _${module}_location
			ERROR_QUIET 
			OUTPUT_STRIP_TRAILING_WHITESPACE)
		if(NOT _${module}_status)
			set(PY_${module_upper} ${_${module}_location} CACHE STRING
				"Location of Python module ${module}")
		endif()
	endif()
	find_package_handle_standard_args(PY_${module}
		"couldn't find python module ${module}:
		\nfor debian systems try: \
		\n\tsudo apt-get install python-${module} \
		\nor for all other OSs/debian: \
		\n\tsudo -H pip install ${module}\n" PY_${module_upper})

endfunction(px4_find_python_module)
