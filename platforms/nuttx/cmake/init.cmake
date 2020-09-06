############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

if(NOT PX4_BOARD)
	message(FATAL_ERROR "PX4_BOARD must be set (eg px4_fmu-v2)")
endif()

if(NOT PX4_BINARY_DIR)
	message(FATAL_ERROR "PX4_BINARY_DIR must be set")
endif()

if(NOT PX4_BOARD_DIR)
	message(FATAL_ERROR "PX4_BOARD_DIR must be set")
endif()

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config CACHE FILEPATH "PX4 NuttX config" FORCE)

# NuttX defconfig
#  cmake should trigger reconfigure if defconfig changes
set(NUTTX_DEFCONFIG ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig CACHE FILEPATH "path to defconfig" FORCE)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${NUTTX_DEFCONFIG})

set(NUTTX_SRC_DIR ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX)
set(NUTTX_DIR ${NUTTX_SRC_DIR}/nuttx CACHE FILEPATH "NuttX directory" FORCE)
set(APPS_DIR ${NUTTX_SRC_DIR}/apps CACHE FILEPATH "NuttX apps directory" FORCE)

px4_add_git_submodule(TARGET git_nuttx PATH "${NUTTX_DIR}")
px4_add_git_submodule(TARGET git_nuttx_apps PATH "${APPS_DIR}")

execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX)

# make olddefconfig (inflate defconfig to full .config)
if(NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx/.config)

	execute_process(COMMAND git clean -ff -x -d --exclude Kconfig WORKING_DIR ${NUTTX_DIR})
	execute_process(COMMAND git clean -ff -x -d --exclude Kconfig WORKING_DIR ${APPS_DIR})

	# If the board provides a Kconfig Use it or create an empty one
	if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
		execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/boards/dummy/Kconfig)
	else()
		execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/boards/dummy/Kconfig)
	endif()

	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${NUTTX_CONFIG_DIR}/include/nsh_romfsimg.h)

	execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_SRC_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs) # Create a temporary Toplevel Make.defs for the oldconfig step

	#file(RELATIVE_PATH NUTTX_CONFIG_REL_DIR ${NUTTX_DIR} ${NUTTX_CONFIG_DIR})
	#execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config)
	#file(APPEND ${NUTTX_DIR}/.config "CONFIG_ARCH_BOARD_CUSTOM=y\n")
	#file(APPEND ${NUTTX_DIR}/.config "CONFIG_ARCH_BOARD_CUSTOM_DIR=${NUTTX_CONFIG_REL_DIR}\n")
	#file(APPEND ${NUTTX_DIR}/.config "CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y\n")
	#file(APPEND ${NUTTX_DIR}/.config "CONFIG_ARCH_BOARD_CUSTOM_NAME=px4\n")

	if(NOT EXISTS ${NUTTX_CONFIG_DIR}/drivers/Kconfig)
		execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${NUTTX_CONFIG_DIR}/drivers)
		execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_CONFIG_DIR}/drivers/Kconfig)
	endif()

	if(NOT EXISTS ${NUTTX_CONFIG_DIR}/src)
		execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${NUTTX_CONFIG_DIR}/src)
	endif()

	configure_file(${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config COPYONLY)

	execute_process(
		COMMAND ${NUTTX_SRC_DIR}/tools/px4_nuttx_make_olddefconfig.sh
		WORKING_DIRECTORY ${NUTTX_DIR}
		OUTPUT_FILE ${PX4_BINARY_DIR}/NuttX/nuttx_olddefconfig.log
		RESULT_VARIABLE ret
	)

	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/.config ${PX4_BINARY_DIR}/NuttX/nuttx/.config)

	# remove temporary top level Make.defs
	execute_process(COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/Make.defs)
endif()

###############################################################################
# NuttX cmake defconfig
###############################################################################

# parse nuttx config options for cmake
file(STRINGS ${NUTTX_DIR}/.config ConfigContents)
foreach(NameAndValue ${ConfigContents})
	# Strip leading spaces
	string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

	# Find variable name
	string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

	if(Name)
		# Find the value
		string(REPLACE "${Name}=" "" Value ${NameAndValue})

		if(Value)
			# remove extra quotes
			string(REPLACE "\"" "" Value ${Value})

			# Set the variable
			#message(STATUS "${Name} ${Value}")
			set(${Name} ${Value} CACHE INTERNAL "NUTTX DEFCONFIG: ${Name}" FORCE)
		endif()
	endif()
endforeach()
