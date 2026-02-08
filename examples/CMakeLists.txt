# examples/CMakeLists.txt

# Helper function to create example targets
function(add_are_example EXAMPLE_NAME)
    add_executable(${EXAMPLE_NAME} ${ARGN})
    target_link_libraries(${EXAMPLE_NAME} PRIVATE are)
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/examples
    )
endfunction()

# Add subdirectories for each example
add_subdirectory(01_hello_triangle)
add_subdirectory(02_cornell_box)
add_subdirectory(03_material_showcase)
add_subdirectory(04_performance_test)
