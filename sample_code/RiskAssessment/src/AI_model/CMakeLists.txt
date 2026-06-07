cmake_minimum_required(VERSION 3.10)

# This directory's sources are built into the RiskAssessment binary by the
# parent CMake. Do not create a standalone executable by default.
option(AI_MODEL_STANDALONE "Build AI_model standalone executable" OFF)

if(AI_MODEL_STANDALONE)
    project(risk_main)

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    # Legacy offline test runner (kept for reference).
    add_executable(risk_main
        ../../tools/ai_offline/legacy_cpp/main.cpp
    )

    target_include_directories(risk_main PRIVATE
        /opt/onnxruntime/include
    )

    target_link_libraries(risk_main PRIVATE
        /opt/onnxruntime/lib/libonnxruntime.so
    )

    set_target_properties(risk_main PROPERTIES
        BUILD_RPATH "/opt/onnxruntime/lib"
        INSTALL_RPATH "/opt/onnxruntime/lib"
    )
else()
    message(STATUS "AI_model standalone build disabled; sources are built by RiskAssessment.")
endif()
