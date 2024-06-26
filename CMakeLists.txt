cmake_minimum_required(VERSION 3.25)
project(mate)

set(CMAKE_CXX_STANDARD 17)

option(BUILD_CLIENT "Build the client" ON)
option(BUILD_DOCS "Build the docs" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (BUILD_CLIENT)
    if (APPLE)
        enable_language(Swift)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

        include_directories(${GSTREAMER_INCLUDE_DIRS})
        link_directories(${GSTREAMER_LIBRARY_DIRS})


        # glfw
        set(GLFW_BUILD_DOCS OFF)
        set(GLFW_BUILD_EXAMPLES OFF)
        set(GLFW_BUILD_TESTS OFF)
        add_subdirectory(lib/glfw)
        find_package(OpenGL REQUIRED)
        include_directories(lib/glfw/include)

        # imgui
        add_library(imgui STATIC
                lib/imgui/imgui.cpp
                lib/imgui/imgui_demo.cpp
                lib/imgui/imgui_draw.cpp
                lib/imgui/imgui_widgets.cpp
                lib/imgui/imgui_tables.cpp
                lib/imgui/backends/imgui_impl_glfw.cpp
                lib/imgui/backends/imgui_impl_opengl3.cpp
        )
        include_directories(lib/imgui lib/imgui/backends)

        # implot
        file(GLOB_RECURSE IMPLOT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/lib/implot/*.cpp)
        add_library(implot STATIC ${IMPLOT_SOURCES})
        include_directories(lib/implot)

        # pigpio
        file(GLOB_RECURSE PIGPIO_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/lib/pigpio/*.c)
        add_library(pigpio STATIC ${PIGPIO_SOURCES})
        include_directories(lib/pigpio)

        # nfd
        add_subdirectory(lib/nativefiledialog-extended EXCLUDE_FROM_ALL)
        include_directories(lib/nativefiledialog-extended/src/include)

        # photogrammetry
        add_custom_command(
                OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/photogrammetry-swift.h
                COMMAND ${CMAKE_Swift_COMPILER} -frontend -typecheck
                ${CMAKE_CURRENT_SOURCE_DIR}/src/photogrammetry.swift
                -sdk ${CMAKE_OSX_SYSROOT}
                -module-name PhotogrammetrySwift
                -cxx-interoperability-mode=default
                -emit-clang-header-path ${CMAKE_CURRENT_BINARY_DIR}/include/photogrammetry-swift.h
                DEPENDS src/photogrammetry.swift
                COMMENT "Generating Swift header"
        )
        add_custom_target(photogrammetry_header DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/include/photogrammetry-swift.h)

        add_library(photogrammetry STATIC src/photogrammetry.swift)
        add_dependencies(photogrammetry photogrammetry_header)
        set_target_properties(photogrammetry PROPERTIES Swift_MODULE_NAME PhotogrammetrySwift)

        add_executable(client
                src/main.cpp
                src/camera_stream.cpp
                src/controls.cpp
                src/photogrammetry.cpp
                src/pi.cpp
                src/depth_sensor.cpp
                src/orientation_sensor.cpp
                src/pid.cpp
        )
        target_include_directories(client PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/include)
        target_link_libraries(client PUBLIC OpenGL::GL ${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES} glfw imgui implot nfd pigpio photogrammetry)
    else()
        message("Client is only supported on macOS")
    endif(APPLE)
endif(BUILD_CLIENT)

if (BUILD_DOCS)
    find_package(Doxygen)
    if (DOXYGEN_FOUND)

        set(DOXYGEN_GENERATE_HTML YES)
        set(DOXYGEN_HTML_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/docs)

        set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${CMAKE_CURRENT_SOURCE_DIR}/README.md)

        set(DOXYGEN_GENERATE_TREEVIEW YES)
        set(DOXYGEN_DISABLE_INDEX NO)
        set(DOXYGEN_FULL_SIDEBAR NO)
        set(DOXYGEN_HTML_EXTRA_STYLESHEET
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/doxygen-awesome-css/doxygen-awesome.css
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/doxygen-awesome-css/doxygen-awesome-sidebar-only.css)
        set(DOXYGEN_HTML_COLORSTYLE DARK)

        set(DOXYGEN_EXTRACT_ALL YES)

        set(DOXYGEN_PROJECT_NAME "Blue Lobster")
        set(DOXYGEN_PROJECT_BRIEF "Code for the MATE 2024 ROV Competition")
        set(DOXYGEN_PROJECT_LOGO "${CMAKE_CURRENT_SOURCE_DIR}/doc/logo.png")

        set(DOXYGEN_DOT_IMAGE_FORMAT svg)
        set(DOXYGEN_INTERACTIVE_SVG YES)
        set(DOXYGEN_DOT_TRANSPARENT YES)
        

        doxygen_add_docs(
            docs
            ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/doc ${CMAKE_CURRENT_SOURCE_DIR}/README.md
            COMMENT "Generating with Doxygen"
        )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(BUILD_DOCS)
