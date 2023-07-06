# Various compiler optimizations to apply to all nodes

if(NOT CMAKE_BUILD_TYPE)
  if (DEFINED CMAKE_TOOLCHAIN_FILE)  # Cross-build for Rio
	  set(CMAKE_BUILD_TYPE Release)
  else()
	  set(CMAKE_BUILD_TYPE RelWithDebInfo)
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

execute_process(COMMAND pgrep -f rosetta OUTPUT_VARIABLE OUT RESULT_VARIABLE INTEL)
if(INTEL)
  message("Intel")
else()
  message("M1, skipping -flto")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wno-switch>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-ftrack-macro-expansion=0>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fno-var-tracking-assignments>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-DPCL_ONLY_CORE_POINT_TYPES=ON>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-DNO_EXPLICIT_INSTANTIATIONS>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-DNON_POLLING>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wextra>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wno-psabi>")
#add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-pedantic>")
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fPIC>")

if (DEFINED CMAKE_TOOLCHAIN_FILE)  # Cross-build for Rio
  # Everything is in the toolchain file
else() # Native builds
  set (CMAKE_RANLIB "gcc-ranlib" )
  set (CMAKE_AR     "gcc-ar"     )
  
  if(INTEL)
    set (OPT_FLAGS "${OPT_FLAGS} -O3 -fno-finite-math-only -flto=auto -fno-fat-lto-objects -fuse-linker-plugin -ffunction-sections -fdata-sections -Wl,-gc-sections")
  else()
    set (OPT_FLAGS "${OPT_FLAGS} -O3 -fno-finite-math-only -fno-fat-lto-objects -fuse-linker-plugin -ffunction-sections -fdata-sections -Wl,-gc-sections")
  endif()
  if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (OPT_FLAGS "${OPT_FLAGS} -mcpu=cortex-a15 -mfpu=neon-vfpv4 -fvect-cost-model")
    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2
	set (OPT_FLAGS "${OPT_FLAGS} -march=armv8-a+crypto -mcpu=cortex-a57+crypto -fvect-cost-model")
    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  else() # x86? Mac?
    set (OPT_FLAGS "${OPT_FLAGS} -march=native -mtune=native")
  endif()
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS}")
#set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
#set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS}")
#set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} ${OPT_FLAGS}")
#set(CMAKE_STATIC_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_STATIC_LINKER_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS}")

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

