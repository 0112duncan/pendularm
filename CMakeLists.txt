cmake_minimum_required(VERSION 3.16)
project(pendularm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(autorob_node src/middleware/node.cpp)
target_include_directories(autorob_node PUBLIC include)
target_link_libraries(autorob_node PUBLIC Threads::Threads)

add_executable(autorob_hub
    src/hub.cpp
    src/rosbridge_server.cpp
    src/middleware/bus.cpp
)
target_include_directories(autorob_hub PRIVATE include)
target_link_libraries(autorob_hub PRIVATE Threads::Threads)

add_executable(integrator_step src/nodes/integrator_step.cpp)
target_link_libraries(integrator_step PRIVATE autorob_node)

function(add_autorob_node target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} PRIVATE autorob_node)
endfunction()

add_autorob_node(arm_sim src/nodes/arm_sim.cpp)
add_autorob_node(ik src/nodes/ik.cpp)
add_autorob_node(ik_action src/nodes/ik_action.cpp)
add_autorob_node(ik_trial src/nodes/ik_trial.cpp)

add_executable(integration_step_test src/nodes/integration_step_test.cpp)
target_link_libraries(integration_step_test PRIVATE autorob_node)

add_executable(demo_node src/examples/demo_node.cpp)
target_link_libraries(demo_node PRIVATE autorob_node)
