cmake_minimum_required(VERSION 3.20)

project(sailroute VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(SAILROUTE_BUILD_TESTS "Build sailroute tests" ON)
option(SAILROUTE_BUILD_BENCHMARKS "Build sailroute benchmarks" OFF)
option(SAILROUTE_ENABLE_LTO "Enable link-time optimization when supported" OFF)

find_package(Threads REQUIRED)
find_package(eccodes CONFIG QUIET)

if(NOT TARGET eccodes AND NOT TARGET eccodes_shared AND NOT TARGET ECCODES::eccodes)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ECCODES REQUIRED IMPORTED_TARGET eccodes)
    add_library(ECCODES::eccodes ALIAS PkgConfig::ECCODES)
elseif(TARGET eccodes)
    add_library(ECCODES::eccodes ALIAS eccodes)
elseif(TARGET eccodes_shared)
    add_library(ECCODES::eccodes ALIAS eccodes_shared)
endif()

add_library(sailroute STATIC
    src/error.cpp
    src/time.cpp
    src/weather/weather_dataset.cpp
    src/polar/vessel_polar.cpp
    src/routing/geodesy.cpp
    src/routing/router.cpp
    src/serialization/json.cpp
    src/serialization/gpx.cpp
    src/serialization/isochrones.cpp
)
add_library(sailroute::sailroute ALIAS sailroute)

target_compile_features(sailroute PUBLIC cxx_std_20)
target_include_directories(sailroute
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(sailroute PRIVATE ECCODES::eccodes Threads::Threads)

if(MSVC)
    target_compile_options(sailroute PRIVATE /W4 /permissive-)
else()
    target_compile_options(sailroute PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

if(SAILROUTE_ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT sailroute_ipo_supported)
    if(sailroute_ipo_supported)
        set_property(TARGET sailroute PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
endif()

add_executable(sailroute_cli apps/sailroute.cpp)
set_target_properties(sailroute_cli PROPERTIES OUTPUT_NAME sailroute)
target_link_libraries(sailroute_cli PRIVATE sailroute::sailroute)

if(SAILROUTE_BUILD_TESTS)
    enable_testing()
    add_executable(sailroute_tests
        tests/test_main.cpp
        tests/test_polar.cpp
        tests/test_routing.cpp
        tests/test_serialization.cpp
        tests/test_time.cpp
        tests/test_weather.cpp
    )
    target_link_libraries(sailroute_tests PRIVATE sailroute::sailroute ECCODES::eccodes)
    target_compile_definitions(
        sailroute_tests
        PRIVATE SAILROUTE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME sailroute_tests COMMAND sailroute_tests)
    add_test(NAME sailroute_cli_help COMMAND sailroute_cli --help)
    set_tests_properties(
        sailroute_cli_help
        PROPERTIES PASS_REGULAR_EXPRESSION "--isochrones-json PATH")
endif()

if(SAILROUTE_BUILD_BENCHMARKS)
    add_executable(sailroute_benchmarks benchmarks/benchmark_main.cpp)
    target_link_libraries(sailroute_benchmarks PRIVATE sailroute::sailroute)
endif()

install(TARGETS sailroute sailroute_cli
    EXPORT sailrouteTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT sailrouteTargets
    FILE sailrouteTargets.cmake
    NAMESPACE sailroute::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sailroute
)

configure_package_config_file(
    cmake/sailrouteConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/sailrouteConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sailroute
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/sailrouteConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/sailrouteConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/sailrouteConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sailroute
)
