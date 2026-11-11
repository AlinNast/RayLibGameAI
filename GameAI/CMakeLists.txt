# CMakeList.txt : CMake project for GameAI, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.14)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("GameAI")

include(FetchContent)
FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.zip
)
FetchContent_MakeAvailable(raylib)

add_executable (GameAI "GameAI.cpp" "GameAI.h" "Agent.cpp" "Agent.h" "Steering.cpp" "Steering.h" "SteeringBehavior.h" "World.h" "Behaviors.h" "CompositeSteeringBehaviors.h" "Collision.h" "Path.h" "Formation.h")
target_link_libraries(GameAI raylib)

set_property(TARGET GameAI PROPERTY CXX_STANDARD 20)