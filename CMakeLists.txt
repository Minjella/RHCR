project(lifelong)

cmake_minimum_required (VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")

# Boost: header-only로만 사용
execute_process(
  COMMAND brew --prefix
  OUTPUT_VARIABLE HOMEBREW_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(Boost CONFIG REQUIRED COMPONENTS filesystem program_options)

add_executable(lifelong ${SOURCES} )

execute_process(COMMAND brew --prefix OUTPUT_VARIABLE HOMEBREW_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)
include_directories(${HOMEBREW_PREFIX}/include)

target_link_libraries(lifelong PRIVATE
  Boost::filesystem
  Boost::program_options
)

add_custom_command(TARGET lifelong POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:lifelong>
          ${CMAKE_SOURCE_DIR}/lifelong
  COMMENT "Copy lifelong to project root"
)
