##########---------- Include additional files  ----------##########
include(cmake/Macro.cmake)
include_modules()

cmake_minimum_required(VERSION 3.10)
project(CalibrationToolbox)

cmake_policy(SET CMP0071 NEW)

##########----------  Manage Library Properties  ----------##########
get_version(PROJECT_VERSION vers_major vers_minor vers_patch)
message( STATUS "Software Version ${vers_major}.${vers_minor}.${vers_patch}" )

# Save information on config.h
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in
   	            ${CMAKE_CURRENT_SOURCE_DIR}/include/config.h)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /utf-8")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /utf-8")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MD")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE HEADERS RELATIVE "${CMAKE_SOURCE_DIR}" include/*.h)
file(GLOB_RECURSE SOURCES RELATIVE "${CMAKE_SOURCE_DIR}" src/*.cpp)
file(GLOB_RECURSE RESOURCES RELATIVE "${CMAKE_SOURCE_DIR}" resources/*)

#set(QT_VERSION "5.12.0")
set(QT_VERSION "5.9")
find_package(Qt5 ${QT_VERSION} COMPONENTS Gui Widgets Svg Xml Concurrent PrintSupport LinguistTools REQUIRED) 
if(Qt5_FOUND)
	message(STATUS "Qt5Gui found version: " ${Qt5Gui_VERSION})
	message(STATUS "Qt5Widgets found version: " ${Qt5Widgets_VERSION})
	message(STATUS "Qt5Svg found version: " ${Qt5Svg_VERSION})
	message(STATUS "Qt5Xml found version: " ${Qt5Xml_VERSION})
	message(STATUS "Qt5Concurrent found version: " ${Qt5Concurrent_VERSION})
	message(STATUS "Qt5PrintSupport found version: " ${Qt5PrintSupport_VERSION})
	message(STATUS "Qt5LinguistTools found version: " ${Qt5LinguistTools_VERSION})
endif()

# Try to link to prebuilt libraries
find_package(OpenCV COMPONENTS world)
# Otherwise use specific components
if(NOT OpenCV_FOUND)
	find_package(OpenCV)
endif()

qt5_wrap_ui(UI_GENERATED_HEADERS ${UI_SOURCES})
message(STATUS "Generated UI files: ")
foreach(ui IN LISTS UI_GENERATED_HEADERS)
	message(STATUS ${ui})
endforeach()

qt5_create_translation(QM_FILES ${HEADERS} ${SOURCES} ${UI_SOURCES} ${TS_FILES})
message(STATUS "Generated QM files: ")
foreach(qm IN LISTS QM_FILES)
	message(STATUS ${qm})
endforeach()

install(FILES ${UI_GENERATED_HEADERS} DESTINATION /include)

include_directories(	
						${CMAKE_CURRENT_BINARY_DIR} 
						${CMAKE_SOURCE_DIR}
						${CMAKE_SOURCE_DIR}/include
						${CMAKE_SOURCE_DIR}/src
						${OpenCV_INCLUDE_DIRS}
					)

message(STATUS "Found HEADERS: ")
foreach(header IN LISTS HEADERS)
	message(STATUS ${header})
endforeach()

message(STATUS "Found SOURCES: ")
foreach(source IN LISTS SOURCES)
	message(STATUS ${source})
endforeach()

add_executable(CalibrationToolbox ${SOURCES} ${HEADERS} ${RCC_SOURCES} ${UI_GENERATED_HEADERS} ${QM_FILES}) 

foreach(source IN LISTS SOURCES)
    get_filename_component(source_path "${source}" PATH)
    string(REPLACE "/" "\\" source_path_msvc "${source_path}")
    source_group("${source_path_msvc}" FILES "${source}")
endforeach()

foreach(header IN LISTS HEADERS)
    get_filename_component(header_path "${header}" PATH)
    string(REPLACE "/" "\\" header_path_msvc "${header_path}")
    source_group("${header_path_msvc}" FILES "${header}")
endforeach()

target_link_libraries(CalibrationToolbox Qt5::Widgets Qt5::Concurrent Qt5::Xml Qt5::Svg Qt5::Gui Qt5::PrintSupport ${OpenCV_LIBS})