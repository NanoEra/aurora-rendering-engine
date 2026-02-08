# Phase 2 verification example

add_are_example(phase2_test
    main.cpp
)

# Copy to bin directory for easy execution
set_target_properties(phase2_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
