# Phase 4 verification example

add_are_example(phase4_test
    main.cpp
)

# Copy to bin directory for easy execution
set_target_properties(phase4_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
