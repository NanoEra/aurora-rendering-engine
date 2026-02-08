# Phase 3 verification example

add_are_example(phase3_test
    main.cpp
)

# Copy to bin directory for easy execution
set_target_properties(phase3_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Copy shaders to build directory
add_custom_command(TARGET phase3_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/shaders
    ${CMAKE_BINARY_DIR}/bin/shaders
    COMMENT "Copying shaders to build directory"
)
