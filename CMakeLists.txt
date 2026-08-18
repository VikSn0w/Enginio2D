cmake_minimum_required(VERSION 3.24)
project(Enginio2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The physics core runs at audio rate: an unoptimised build will not keep up.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()
if(MSVC)
    add_compile_options(/EHsc $<$<CONFIG:Debug>:/O2>)
endif()

# SFML's package config only exports Debug and Release. Without these maps,
# RelWithDebInfo/MinSizeRel silently fall back to the Debug import, which links
# a /MD executable against /MDd SFML - the mismatch shows up as a debug-CRT
# assertion inside the first SFML call that crosses the DLL boundary.
set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release "")
set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL Release "")

find_package(SFML 3.1 REQUIRED COMPONENTS Graphics Audio Window System HINTS "deps/SFML/lib/cmake/SFML")

add_executable(Enginio2D
        src/app/main.cpp
        src/sim/Engine.cpp
        src/sim/EngineDesign.cpp
        src/sim/Drivetrain.cpp
        src/sim/Dyno.cpp
        src/audio/EngineSound.cpp
        src/ui/Widgets.cpp
        src/ui/Editor.cpp
)
target_include_directories(Enginio2D PRIVATE src)
target_link_libraries(Enginio2D PRIVATE SFML::Graphics SFML::Audio SFML::Window SFML::System)

# SFML ships as DLLs: put them next to the executable.
add_custom_command(TARGET Enginio2D POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_RUNTIME_DLLS:Enginio2D> $<TARGET_FILE_DIR:Enginio2D>
        COMMAND_EXPAND_LISTS)
