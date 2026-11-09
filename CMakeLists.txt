cmake_minimum_required(VERSION 3.16)
project(build123d_cpp VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  # The library itself needs C++17; C++20 enables designated initializers for the
  # option structs, e.g. extrude(sketch, 10, {.both = true}).
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD123D_BUILD_EXAMPLES "Build the examples" ON)
option(BUILD123D_BUILD_TESTS "Build the tests" ON)

find_package(OpenCASCADE REQUIRED)
message(STATUS "OpenCASCADE ${OpenCASCADE_VERSION} at ${OpenCASCADE_INSTALL_PREFIX}")

# Toolkit names changed in OCCT 7.8 (TKSTEP* -> TKDESTEP, TKSTL -> TKDESTL, ...),
# so link whichever of them this installation provides.
set(BUILD123D_OCCT_TOOLKITS
    TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool
    TKShHealing TKFillet TKOffset TKFeat TKMesh TKHLR TKService TKV3d
    TKCDF TKLCAF TKCAF TKVCAF TKXCAF TKBinXCAF TKXmlXCAF
    TKDE TKXSBase TKDESTEP TKSTEP TKSTEPBase TKSTEPAttr TKSTEP209 TKXDESTEP
    TKDESTL TKSTL TKRWMesh TKDEGLTF)
set(BUILD123D_OCCT_LIBS)
foreach(tk IN LISTS BUILD123D_OCCT_TOOLKITS)
  if(TARGET ${tk})
    list(APPEND BUILD123D_OCCT_LIBS ${tk})
  endif()
endforeach()

# lib3mf (3MF and STL meshes, Mesher): its C++ binding headers and the shared library. Found
# directly, as lib3mf's own CMake config only covers some installations and prints banners.
find_path(LIB3MF_INCLUDE_DIR lib3mf_implicit.hpp
          PATH_SUFFIXES Bindings/Cpp include/Bindings/Cpp lib3mf/Bindings/Cpp include/lib3mf/Bindings/Cpp)
find_library(LIB3MF_LIBRARY NAMES 3mf lib3mf)
if(NOT LIB3MF_INCLUDE_DIR OR NOT LIB3MF_LIBRARY)
  message(FATAL_ERROR "lib3mf not found (scripts/setup-toolchain.sh installs it); set LIB3MF_INCLUDE_DIR "
                      "to the directory of lib3mf_implicit.hpp and LIB3MF_LIBRARY to the library")
endif()
message(STATUS "lib3mf: ${LIB3MF_LIBRARY}")

file(GLOB BUILD123D_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(build123d ${BUILD123D_SOURCES})
add_library(build123d::build123d ALIAS build123d)
target_compile_features(build123d PUBLIC cxx_std_17)
target_include_directories(build123d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # The OCCT imported targets carry their include directory; this covers older configs.
    $<BUILD_INTERFACE:${OpenCASCADE_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(build123d PUBLIC ${BUILD123D_OCCT_LIBS})

# libcurl uploads to PCBWay (export_to_pcbway); without it that function throws.
find_package(CURL QUIET)
if(CURL_FOUND)
  target_link_libraries(build123d PRIVATE CURL::libcurl)
  target_compile_definitions(build123d PRIVATE BUILD123D_HAVE_CURL)
else()
  message(WARNING "libcurl not found: export_to_pcbway will throw std::runtime_error")
endif()

target_include_directories(build123d SYSTEM PRIVATE ${LIB3MF_INCLUDE_DIR})
target_link_libraries(build123d PRIVATE ${LIB3MF_LIBRARY})
if(MSVC)
  target_compile_options(build123d PRIVATE /W3 /bigobj /permissive-)
  target_compile_definitions(build123d PUBLIC _USE_MATH_DEFINES NOMINMAX)
else()
  target_compile_options(build123d PRIVATE -Wall -Wextra -Wno-unused-parameter
                                           -Wno-deprecated-declarations)
endif()

if(BUILD123D_BUILD_EXAMPLES)
  file(GLOB BUILD123D_EXAMPLES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp)
  foreach(example_src IN LISTS BUILD123D_EXAMPLES)
    get_filename_component(example_name ${example_src} NAME_WE)
    add_executable(${example_name} ${example_src})
    target_link_libraries(${example_name} PRIVATE build123d)
    set_target_properties(${example_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                     ${CMAKE_BINARY_DIR}/examples)
  endforeach()
endif()

if(BUILD123D_BUILD_TESTS)
  enable_testing()
  file(GLOB BUILD123D_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
  add_executable(build123d_tests ${BUILD123D_TEST_SOURCES})
  target_link_libraries(build123d_tests PRIVATE build123d)
  target_include_directories(build123d_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  add_test(NAME build123d_tests COMMAND build123d_tests
           WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  # Prints metrics of models also built with Python build123d (tests/compare/compare.py).
  file(GLOB BUILD123D_COMPARE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/compare/*.cpp)
  add_executable(compare_cases ${BUILD123D_COMPARE_SOURCES})
  target_link_libraries(compare_cases PRIVATE build123d)
  target_include_directories(compare_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples
                                                   ${CMAKE_CURRENT_SOURCE_DIR}/tests/compare)
endif()

include(GNUInstallDirs)
install(TARGETS build123d EXPORT build123dTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/build123d DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT build123dTargets NAMESPACE build123d::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/build123d)

# The fonts that come with build123d (its data/fonts: the single-line font Relief SingleLine CAD,
# SIL Open Font License). The library looks for them at run time in $BUILD123D_FONT_DIR,
# ../share/build123d/fonts next to itself, the install prefix and this source tree.
install(DIRECTORY data/fonts DESTINATION ${CMAKE_INSTALL_DATADIR}/build123d)
target_compile_definitions(build123d PRIVATE
  BUILD123D_INSTALL_FONT_DIR="${CMAKE_INSTALL_FULL_DATADIR}/build123d/fonts"
  BUILD123D_SOURCE_FONT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/fonts")
target_link_libraries(build123d PRIVATE ${CMAKE_DL_LIBS})
