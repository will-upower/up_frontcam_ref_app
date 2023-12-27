#############################################################################################################################
# Copyright [2023] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
#
# The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
# and/or its licensors ("Renesas") and subject to statutory and contractual protections.
#
# Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
# display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
# purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
# SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
# NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
# OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
# be subject to different terms.
#######################################################################################################################

#----------------------------------------------------------------------------------------------------------------------
# Set app name
#----------------------------------------------------------------------------------------------------------------------
set(app_name "up_frontcam_ref_app")
set(app_version "0.3.0")

cmake_minimum_required(VERSION 3.16.3)

set(CMAKE_C_COMPILER /opt/poky/3.1.11/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc)
set(rcar-xos_DIR /home/ubuntu/Renesas/rcar-xos/v3.18.0/cmake/)
set(rcar-xos-aarch64-gnu-linux-v4h2_DIR /home/ubuntu/Renesas/rcar-xos/v3.18.0/cmake/)
set(OpenCV_DIR /home/ubuntu/Renesas/rcar-xos/v3.18.0/tools/toolchains/poky/sysroots/aarch64-poky-linux/usr/lib/cmake/opencv4/)
set(RCAR_SOC "v4h2")

project(${app_name})

# ROS2 Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

#----------------------------------------------------------------------------------------------------------------------
# R-Car xOS SoC/Board selection
#----------------------------------------------------------------------------------------------------------------------
if(NOT RCAR_SOC) # This guard allows to set RCAR_SOC externally, e.g. via CMake Commandline during configuration
    set(RCAR_SOC
        V3H2 V3U # Allowed values are at the moment V3H1, V3M2. Availability depends on the package you have.
    )
endif()

#----------------------------------------------------------------------------------------------------------------------
# Find R-Car xOS platform with your requested configuration
# - In case you need a specific version, usage: find_package(rcar-xos v0.3.0 EXACT REQUIRED)
# - In case you need at least version, usage: find_package(rcar-xos v0.3.0 REQUIRED)
#
# If the command fails to find the package:
#   Make sure rcar-xos config files can be found via standard variables set in your system:
#   PATH environment variable or rcar-xos_DIR (environment or CMake variable set via commandline)
#
# See R-Car xOS documentation for more help or https://cmake.org/cmake/help/latest/command/find_package.html
#----------------------------------------------------------------------------------------------------------------------
find_package(rcar-xos REQUIRED)

#----------------------------------------------------------------------------------------------------------------------
# Configurations
#----------------------------------------------------------------------------------------------------------------------

# Set to TRUE if you want to support the BCL format.
set(SUPPORT_BCL_FORMAT FALSE)

# Set to TRUE if you want to use IMP Driver instead of IMP Framework.
set(USE_IMPDRV TRUE)

find_package(rcar-xos REQUIRED)
if(${rcar-xos_FOUND})
  message("Found xOS ${rcar-xos_VERSION}")
endif()

cmake_policy(SET CMP0075 NEW)

include(CheckSymbolExists)
include(CMakePushCheckState)
include(CheckIncludeFile)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-DXOS_VERSION_${rcar-xos_VERSION_MAJOR}_${rcar-xos_VERSION_MINOR}_${rcar-xos_VERSION_PATCH}")

STRING(TOUPPER "${RCAR_SOC}" RCAR_SOC_LOCAL)
add_definitions("-DRCAR_SOC_${RCAR_SOC_LOCAL}")
add_definitions("-DRCAR_SOC=${RCAR_SOC_LOCAL}")
STRING(TOUPPER "${RCAR_XIL}" RCAR_XIL_LOCAL)
add_definitions("-DRCAR_XIL_${RCAR_XIL_LOCAL}")
add_definitions("-DRCAR_XIL=${RCAR_XIL_LOCAL}")

add_definitions("-DRCAR_SOC_${RCAR_SOC_CLASS}${RCAR_SOC_GEN}${RCAR_SOC_LEVEL}${RCAR_SOC_VERSION_MAJOR}${RCAR_SOC_VERSION_MINOR}")
add_definitions("-DRCAR_SOC_${RCAR_SOC_CLASS}${RCAR_SOC_GEN}${RCAR_SOC_LEVEL}${RCAR_SOC_VERSION_MAJOR}")

# OpenCV required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )

#----------------------------------------------------------------------------------------------------------------------
# Source files
#----------------------------------------------------------------------------------------------------------------------
set(source
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/front_cam_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/modetest/buffers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/modetest/modetest.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/util/format.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/imr/osaldrv_helper.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/imr/imr_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/displayimage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/draw_detection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/object_detection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/os/vout/Linux/vout_display_linux.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/os/vout/vout_wrapper.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/customize/frontcam_customize.c
)

if(RCAR_SOC STREQUAL "v4h2")
    list(APPEND source
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/src/r_dc_isp_memories.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/src/r_dc_isp_setup.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/src/r_dc_isp_main.c
    )
endif()

if(RCAR_SOC STREQUAL "v4h2")
    if(HAS_RCV_IMPDRV_H)
        LIST(APPEND source
        #${CMAKE_CURRENT_SOURCE_DIR}/application/src/cnn/.librvxtarget/src/rvxt_execute_xOS1.c
        )
    ELSE()
        #LIST(APPEND source ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cnn/.librvxtarget/src/rvxt_execute.c)
    ENDIF()   


    if(SUPPORT_BCL_FORMAT)
        list(APPEND source
           ${CMAKE_CURRENT_SOURCE_DIR}/application/rvx_target/src/rvx_target/rvxt_bcl.c
        )
    endif()
endif()

#----------------------------------------------------------------------------------------------------------------------
# Header files of the application
#----------------------------------------------------------------------------------------------------------------------
set(header
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/include/common.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/modetest/buffers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/util/format.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/imr/include/osaldrv_helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/imr/include/reg_settings.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/opencv.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/draw_detection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/object_detection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/read_coordinate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/read_road_coordinate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/pose.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/postprocess.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/os/vout/Linux/vout_display_linux.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/os/vout/vout_wrapper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/customize/include/customize.h
)

if(RCAR_SOC STREQUAL "v4h2")
    list(APPEND header
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/vout/target/v4h/include/soc_vout_param.h
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/include/r_dc_isp_memories.h
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/include/r_dc_isp_setup.h
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/include/r_dc_isp_main.h
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/include/r_dc_isp_config.h
    )
endif()

#----------------------------------------------------------------------------------------------------------------------
# Include directories used by the application
#----------------------------------------------------------------------------------------------------------------------
set(include_dir
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/os/vout/Linux/
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/os/vout/
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/util/
    ${CMAKE_CURRENT_SOURCE_DIR}/application/3rdparty/libdrm/tests/modetest/
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/imr/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/opencv/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/color_conv/include/
    ${CMAKE_CURRENT_LIST_DIR}/application/inc
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_HEADER_OUTPUT_DIRECTORY}
    ${app_nameINCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/application/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/application/rvx_target/inc
    ${SDKROOT}/sysroots/aarch64-poky-linux/usr/include/libdrm
    ${SDKROOT}/sysroots/aarch64-poky-linux/usr/include/drm
    ${SDKROOT}/sysroots/aarch64-poky-linux/usr/src/debug/libdrm/2.4.101-r0/libdrm-2.4.101
    ${CMAKE_CURRENT_SOURCE_DIR}/application/src/customize/include/
)

if(RCAR_SOC STREQUAL "v4h2")
    list(APPEND include_dir
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/vout/target/v4h/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/isp/target/v4h/include/
    )
endif()

if(SUPPORT_BCL_FORMAT)
    add_compile_definitions(SUPPORT_BCL_FORMAT)
endif()
if(USE_IMPDRV)
    add_compile_definitions(USE_IMPDRV)
endif()

if(RCAR_SOC STREQUAL "v3u")
    add_compile_definitions(DRV_ARCH_RCAR_V3U)
    add_compile_definitions(DRV_ARCH_RCAR_V3U1)
elseif(RCAR_SOC STREQUAL "v3h2")
    add_compile_definitions(DRV_ARCH_RCAR_V3H2)
elseif(RCAR_SOC STREQUAL "v4h2")
    add_compile_definitions(DRV_ARCH_RCAR_V4H)
endif()

if(RCAR_SOC STREQUAL "v3h2")
    STRING(TOUPPER "${RCAR_SOC}" RCAR_SOC_LOCAL)

    add_compile_definitions("DRV_ARCH_RCAR_${RCAR_SOC_LOCAL}")
    add_definitions("-DRCAR_${RCAR_SOC_LOCAL}")

    # for CNN toolchain
    add_definitions("-DRCAR_SOC_${RCAR_SOC_LOCAL}")
    add_definitions("-DRCAR_SOC=${RCAR_SOC_LOCAL}")

    # MAX_MODULE definiton for IMR related logic
    add_compile_definitions(MAX_MODULE=5)
elseif(RCAR_SOC STREQUAL "v3u")
    add_definitions(-DRCAR_V3U)
    add_compile_definitions(MAX_MODULE=6)
elseif(RCAR_SOC STREQUAL "v3h1")
    add_definitions(-DRCAR_V3H1)
    add_compile_definitions(MAX_MODULE=6)
elseif(RCAR_SOC STREQUAL "v3m2")
    add_definitions(-DRCAR_V3M2)
    add_compile_definitions(MAX_MODULE=4)
elseif(RCAR_SOC STREQUAL "v4h")
    add_definitions(-DRCAR_V4H)
    add_compile_definitions(MAX_MODULE=5)
elseif(RCAR_SOC STREQUAL "v4h2")
    add_definitions(-DRCAR_V4H2)
    add_compile_definitions(MAX_MODULE=5)
endif()

if(RCAR_TARGET_OS STREQUAL "linux")
    if (CMAKE_C_COMPILER MATCHES "armclang")
        add_compile_definitions(__linux__=1)
    endif()
endif()

if(RCAR_TARGET_OS STREQUAL "windows")
    set(triplet x86_64-w64-mingw32)
    add_compile_definitions(RCAR_ENV_TARGET_SIL)
elseif(RCAR_TARGET_OS STREQUAL "linux" AND
    CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(triplet x86_64-linux-gnu)
    add_compile_definitions(_POSIX_C_SOURCE=199309L)
    add_compile_definitions(RCAR_ENV_TARGET_SIL)
else()
    set(triplet ${TRIPLE})
endif()
#----------------------------------------------------------------------------------------------------------------------
# Link libraries and Link directories used by the application
#----------------------------------------------------------------------------------------------------------------------
if(RCAR_SOC STREQUAL "v4h2")
    set(link_lib
        #atmlib
        # This sample does not need any other sw compoents
        # Examples could be: osal, osal_wrapper
        impfw
        impfw_dbg
        imp
        atmlib
        osal
        #rvx_target
        drm
        util
        imr
        imrdlg
        m
        cisp            # ISP Core
        tisp            # ISP Top
        ${OpenCV_LIBS}
        dl
        adas_ref_fwk
    )
endif()

if(USE_IMPDRV)
    list(APPEND link_lib
        imp
    )
else()
    list(APPEND link_lib
        impfw
        imp
    )
endif()

if((RCAR_SOC STREQUAL "v3u") OR (RCAR_SOC STREQUAL "v4h") OR (RCAR_SOC STREQUAL "v4h2"))
    list(APPEND link_lib
        vspx
    )
endif()
# [optional] for 3rd Party libraries
set(link_dir
    # Additional Link Directories for 3rd party libraries
    #${CMAKE_CURRENT_SOURCE_DIR}/../refapp_common_fwk/build_linux_dev_board/
)

#----------------------------------------------------------------------------------------------------------------------
# Set document file list
# If not set, it will search for document in ${CMAKE_CURRENT_SOURCE_DIR}/doc by default
#----------------------------------------------------------------------------------------------------------------------
set(document
    # List of document file to install to package
)

set(remote_document 
   # List of remote document files (relative path of rcar-docs/xos3/dev-xos3/frontcam_ref_app folder in Artifactory)
   "frontcam_ref_app/R-CarV4H2_FrontCam_ref_app_User_Manual.pdf" 
)


#----------------------------------------------------------------------------------------------------------------------
# [Optional] Any specifics that an application may require, like OS specifics, SoC specifics
#----------------------------------------------------------------------------------------------------------------------

# Example for enabling OSAL for certain environments
if(RCAR_TARGET_OS STREQUAL "baremetal" )
    list(APPEND link_lib
        startup         # In baremetal environments the application requires at least startup code defined for xOS
    )
else() # Any other Target OS
    list(APPEND link_lib
        osal            # This application makes use of the OSAL API
        osal_wrapper    # The application also needs the implementation of the OSAL API to link.
    )
endif()

if(RCAR_XIL STREQUAL "hil")
        list(APPEND link_lib
            osal_configuration
        )
endif()

# Linker dependencies for baremetal/freertos environments
if (CMAKE_LINKER MATCHES "armlink")
    set(linker_script "${CMAKE_CURRENT_SOURCE_DIR}/application/lscript.scat")
endif()

#----------------------------------------------------------------------------------------------------------------------
# Doxygen document setting
#----------------------------------------------------------------------------------------------------------------------
set(api_doc "NO")

# API doc JSON file location (if not set, use default location ${source_dir}/doc/doxygen/doc_properties.json)
set(api_doc_json "")

# Set to ON to add SoC suffix to document file name
set(api_doc_soc_depend OFF)

# List of module to combine API document, input headers are their public headers
set(doxygen_combine_module
)

# Decide to enable or disable doxygen for private_header, source code (default FALSE)
set(doxygen_enable_for_source "FALSE")

# List of patterns which you don't want to document in official release
set(doxygen_exclude_pattern
)

# Set the image folder location on Artifactory to insert to document
set(doxygen_image_remote_path
    "path/to/image/remote/folder"
)

# Enable release note automation generation (default NO)
set(release_note "YES")

# Release note PDF file name (if not set, use default file name ${module}_ReleaseNote)
set(release_note_file_name "R-CarV4H2_FrontCam_ref_app_ReleaseNote")

# Release note JSON file location (if not set, use default location ${source_dir}/doc/release_note/release_note.json)
set(release_note_json "")

#----------------------------------------------------------------------------------------------------------------------
# Test settings, e.g. which and how to run the tests for this application
#----------------------------------------------------------------------------------------------------------------------
set(test_run_time_test_config_file
    # Set this test variable if you want to select the device/os dependent runtime test data config file
    # If not set. The default config (run_time_test_data.json) will be used
    ${CMAKE_CURRENT_SOURCE_DIR}/run_time_test_data_${RCAR_SOC}.json
)
set(test_args_config_file
    # Set this test variable if you want to select the device dependent test args config file
    # If not set. The default config (test_args.json) will be used for application test.
    ${CMAKE_CURRENT_SOURCE_DIR}/test_args_${RCAR_SOC}.json
)

#----------------------------------------------------------------------------------------------------------------------
# Test settings, e.g. which and how to run the tests for this application
#----------------------------------------------------------------------------------------------------------------------
set(test_arg
    "--help"
)

# set(test_pretest_cmd
#     "export PATH=$PATH:/sbin"
#     "rmmod cmemdrv &> /dev/null"
#     "modprobe cmemdrv bsize=0x1d000000"
# )

#======================================================================================================================
# Set runtime test data
#======================================================================================================================
#set(test_run_time_test_config_file ${CMAKE_CURRENT_SOURCE_DIR}/run_time_test_data_${RCAR_SOC}_${RCAR_TARGET_OS}.json)

#======================================================================================================================
# Set test output file
#======================================================================================================================
set(test_output
    #/ci_test/Output_Buffer.tar.xz
    #final_s_memory_5.csv
    #final_s_memory_1.csv
)
#======================================================================================================================
# Set test_use_shared_library
#======================================================================================================================
# Set this variable for using shared library when executing the test

set(test_use_shared_library YES)
#======================================================================================================================
# DO NOT CHANGE ANYTHING BELOW THIS LINE, except you know what to do ;)
#======================================================================================================================
rcar_configure_application()

# This app include public header from protobuf which produce error, we disable warning for this app

target_compile_options(${app_name} PRIVATE -w)
#endif()

if(RCAR_SOC STREQUAL "v4h2")
    if(TARGET ai_lib AND RCAR_PRIVATE_BUILD)
        set(app_name "frontcam_with_cdnn_ref_app")
        
        list(APPEND source
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/src/cdnn_main.c
        )

        list(APPEND header
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/include/buffer_configuration.h
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/include/cdnn_main.h
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/include/fc_v4h2_semseg_input.h
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/include/fc_v4h2_semseg_netinfo.h
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/include/fc_v4h2_objdet_input.h
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/include/fc_v4h2_objdet_netinfo.h
        )

        list(APPEND include_dir
            ${CMAKE_CURRENT_SOURCE_DIR}/application/src/cdnn/include
        )
        
        list(APPEND link_lib
                ai_lib
        )

        set(test_args_config_file
            # Set this test variable if you want to select the device dependent test args config file
            # If not set. The default config (test_args.json) will be used for application test.
            ${CMAKE_CURRENT_SOURCE_DIR}/test_args_cdnn_${RCAR_SOC}.json
        )

        rcar_configure_application()
        set_target_properties(${app_name} PROPERTIES __DISABLE_INSTALL TRUE)
        if(TARGET ${app_name})
            # Set applications compilation flags
            target_compile_options(${app_name} PRIVATE -w
                -DCDNN
            )

        endif()
    endif()
 endif()

 set(UPower_FRONTCAM_HEADERS
    include/up_frontcam_ref_app_node.hpp
)

 ament_auto_add_library($(PROJECT_NAME) SHARED
    ${source}
    ${header}
    ${include_dir}
    src/up_frontcam_ref_app_node.cpp
 )

 target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
 )

 rclcpp_components_register_node(${PROJECT_NAME}_node
    PLUGIN "upower::UpFrontCamRefAppNode"
    EXECUTABLE ${PROJECT_NAME}_node_exe
 )


 ament_auto_package()




