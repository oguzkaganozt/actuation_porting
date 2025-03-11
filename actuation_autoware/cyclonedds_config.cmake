# Copyright (c) 2022-2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

macro(create_cdds_lib CYCLONEDDS_ROOT)
  include(ExternalProject)

  # # Get compile options from Zephyr to be passed to the application components.
  # # zephyr_get_system_include_directories_for_lang_as_string(C ext_system_includes)
  # # zephyr_get_include_directories_for_lang_as_string(C ext_includes)
  # # zephyr_get_compile_definitions_for_lang_as_string(C ext_defs)
  # # zephyr_get_compile_options_for_lang_as_String(C ext_Copts)
  # # zephyr_get_compile_options_for_lang_as_String(CXX ext_CXXopts)

  # string(REPLACE " -D__ZEPHYR__=1" "" EXT_DEFS_MODIFIED "${ext_defs}")
  # # set(CMAKE_C_FLAGS "${EXT_DEFS_MODIFIED} -D_POSIX_C_SOURCE=200809L ${ext_system_includes} ${ext_includes} ${ext_Copts} -Wno-error -DDDSRT_HAVE_INET_PTON=1 -Wno-type-limits -DBUILD_SHARED_LIBS=0 -DENABLE_SECURITY=0 -DENABLE_SSL=0 -DENABLE_SOURCE_SPECIFIC_MULTICAST=0 -DENABLE_IPV6=0 -DENABLE_SHM=0 -DBUILD_IDLC=0")
  # # message(FATAL_ERROR "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")

  # if(${ZEPHYR_TARGET} STREQUAL "native_sim")
  #   set(ext_cflags
  #       "${EXT_DEFS_MODIFIED} -D_POSIX_C_SOURCE=200809L ${ext_system_includes} ${ext_includes} ${ext_Copts} -Wno-error -DDDSRT_HAVE_INET_PTON=1 -Wno-type-limits -DBUILD_SHARED_LIBS=0 -DENABLE_SECURITY=0 -DENABLE_SSL=0 -DENABLE_SOURCE_SPECIFIC_MULTICAST=0 -DENABLE_IPV6=0 -DENABLE_SHM=0 -DBUILD_IDLC=0"
  #   )
  # else()
  #   set(ext_cflags
  #       "${ext_defs} -D_POSIX_C_SOURCE=200809L ${ext_system_includes} ${ext_includes} ${ext_Copts} -Wno-error -DDDSRT_HAVE_INET_PTON=1 -Wno-type-limits"
  #   )
  # endif()

  # set(CDDS_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/cyclonedds-prefix/lib)
  # set(CDDS_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cyclonedds-prefix/include)
  
  # # if(${ZEPHYR_TARGET} STREQUAL "native_sim")
  # #   set(CYCLONEDDS_OPTIONS "-DWITH_ZEPHYR=0 -D__ZEPHYR__=0 -DBUILD_SHARED_LIBS=0 -DENABLE_SECURITY=0 -DENABLE_SSL=0 -DENABLE_SOURCE_SPECIFIC_MULTICAST=0 -DENABLE_IPV6=0 -DENABLE_SHM=0 -DBUILD_IDLC=0")
  # # else()
  # #   set(CYCLONEDDS_OPTIONS "-DWITH_ZEPHYR=1 -DBUILD_SHARED_LIBS=1 -DENABLE_SECURITY=0 -DENABLE_SSL=0 -DENABLE_SOURCE_SPECIFIC_MULTICAST=0 -DENABLE_IPV6=0 -DENABLE_SHM=0 -DBUILD_IDLC=0")
  # #   message(STATUS "Building CycloneDDS dynamically for ${ZEPHYR_TARGET}")
  # # endif()

  ExternalProject_Add(cyclonedds
    SOURCE_DIR ${CYCLONEDDS_ROOT}
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/cyclonedds
    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    # CMAKE_ARGS
      # -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY
      # -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
      # -DCMAKE_VERBOSE_MAKEFILE=1
      # -DCMAKE_C_FLAGS=${ext_cflags}
      # -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      # -DCMAKE_SYSTEM_NAME=Generic
      # -DCMAKE_BUILD_TYPE=Debug
      # ${CYCLONEDDS_OPTIONS}
    # DEPENDS zephyr_interface
    BUILD_BYPRODUCTS ${CDDS_LIB_DIR}/libddsc.a
  )

  add_library(cdds_lib STATIC IMPORTED GLOBAL)
  add_dependencies(cdds_lib cyclonedds)
  set_target_properties(cdds_lib PROPERTIES IMPORTED_LOCATION ${CDDS_LIB_DIR}/libddsc.a)

  target_include_directories(app PUBLIC ${CDDS_INCLUDE_DIR})
endmacro()
