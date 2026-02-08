# Phase 2 visual verification example

add_are_example(visual_test
    main.cpp
)

# Copy to bin directory
set_target_properties(visual_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
