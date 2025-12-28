cmake_minimum_required(VERSION 3.16)


#Here we include only components required by target esp32p4.
if(IDF_TARGET STREQUAL "esp32p4")
    set(EXTRA_COMPONENT_DIRS
    "${CMAKE_SOURCE_DIR}/esp32p4_components"
    ./esp32p4_components/bsp_extra
    )

    set(SRCS "")

    set(INCLUDE_DIRS "")
    list(APPEND INCLUDE_DIRS "include")

    list(APPEND SRCS
        "src/bsp_board_extra.c"
    )

    foreach(SRC_DIR IN LISTS SRC_DIRS)
        file(GLOB_RECURSE SRC ${SRC_DIR}/*.c)
        list(APPEND SRCS ${SRC})
        list(APPEND INCLUDE_DIRS ${SRC_DIR}/include)
    endforeach()

endif()

#Here we manage sdk configs with a shared configuration and one specific per target.
set(SDKCONFIG_DEFAULTS
  "${CMAKE_SOURCE_DIR}/sdkconfig.defaults"
)

if(IDF_TARGET STREQUAL "esp32s3")
  list(APPEND SDKCONFIG_DEFAULTS
    "${CMAKE_SOURCE_DIR}/sdkconfig.defaults.esp32s3")
elseif(IDF_TARGET STREQUAL "esp32p4")
  list(APPEND SDKCONFIG_DEFAULTS
    "${CMAKE_SOURCE_DIR}/sdkconfig.defaults.esp32p4")
endif()


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rb_project)