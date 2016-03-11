macro(add_affordance_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    hpp-affordance
    ${Boost_LIBRARIES}
    )
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_affordance_test)

message ("executable output path: ${EXECUTABLE_OUTPUT_PATH}")
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})

message ("current binary dir:, ${CMAKE_CURRENT_BINARY_DIR}")
message ("boos include dir: ${Boost_INCLUDE_DIRS}")

#add_affordance_test(test-basic test-main.cc)
#add_affordance_test(test-create-environment test-main2.cc)
add_affordance_test(test-create-affordance test-main3.cc)
