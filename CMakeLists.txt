cmake_minimum_required(VERSION 3.20)

project(HeroLob LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)

enable_testing()

add_library(hero_lob_core
    core/src/pipeline.cpp
    core/src/capture.cpp
    core/src/identifier.cpp
    core/src/identifier_geometry.cpp
    core/src/tracker.cpp
    core/src/reference_frame_selector.cpp
    core/src/image_registrator.cpp
    core/src/background_remover.cpp
    core/src/tracker_processor.cpp
    core/src/image_synthesis.cpp
)

target_include_directories(hero_lob_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_core
    PUBLIC
        ${OpenCV_LIBS}
)

add_executable(hero_lob
    core/src/main.cpp
)

target_link_libraries(hero_lob
    PRIVATE
        hero_lob_core
)

add_executable(hero_lob_color_picker
    core/tools/color_picker_main.cpp
)

target_include_directories(hero_lob_color_picker
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_color_picker
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_brighten
    core/tools/brighten_main.cpp
)

target_include_directories(hero_lob_brighten
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_brighten
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_identifier_debug
    core/tools/identifier_debug_main.cpp
)

target_link_libraries(hero_lob_identifier_debug
    PRIVATE
        hero_lob_core
)

add_executable(hero_lob_trajectory_debug
    core/tools/trajectory_debug_main.cpp
)

target_link_libraries(hero_lob_trajectory_debug
    PRIVATE
        hero_lob_core
)

add_executable(identifier_geometry_test
    core/tests/identifier_geometry_test.cpp
)

target_link_libraries(identifier_geometry_test
    PRIVATE
        hero_lob_core
)

add_test(NAME identifier_geometry_test COMMAND identifier_geometry_test)

add_executable(identifier_synthetic_test
    core/tests/identifier_synthetic_test.cpp
)

target_link_libraries(identifier_synthetic_test
    PRIVATE
        hero_lob_core
)

add_test(NAME identifier_synthetic_test COMMAND identifier_synthetic_test)

add_executable(background_remover_test
    core/tests/background_remover_test.cpp
)

target_link_libraries(background_remover_test
    PRIVATE
        hero_lob_core
)

add_test(NAME background_remover_test COMMAND background_remover_test)

add_executable(tracker_processor_test
    core/tests/tracker_processor_test.cpp
)

target_link_libraries(tracker_processor_test
    PRIVATE
        hero_lob_core
)

add_test(NAME tracker_processor_test COMMAND tracker_processor_test)

add_executable(pipeline_smoke_test
    core/tests/pipeline_smoke_test.cpp
)

target_compile_definitions(pipeline_smoke_test
    PRIVATE
        HERO_LOB_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(pipeline_smoke_test
    PRIVATE
        hero_lob_core
)

add_test(NAME pipeline_smoke_test COMMAND pipeline_smoke_test)
