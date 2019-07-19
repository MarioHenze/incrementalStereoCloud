#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Link Boost as dependency
find_package(Boost REQUIRED)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

cgv_find_package(point_cloud)
cgv_find_package(cg_ext)
cgv_find_package(crg_grid)
cgv_find_package(crg_light)

set(PLUGINS
	cg_fltk
	cg_icons
	cg_ext
	cmi_io
	crg_grid
	crg_light
	crg_stereo_view
)

# Source files
file(GLOB_RECURSE SOURCES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.cxx")
# Header files
file(GLOB_RECURSE HEADERS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.h")
# Shader files
file(GLOB_RECURSE SHADER_SOURCES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.gl*")

include_directories(
	${cgv_INCLUDE_DIRS} 
	${cgv_INCLUDE_DIRS}/libs 
	${cgv_gl_INCLUDE_DIRS}
)

cgv_add_module(${PROJECT_NAME} ${SOURCES} ${IMG_SOURCES} ${HEADERS} ${SHADER_SOURCES} ${IMAGES})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${cgv_LIBRARIES} point_cloud)
cgv_test_shaders(${PROJECT_NAME} ${SHADER_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "${FOLDER_NAME_APPLICATION_PLUGINS}")

set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS "plugin:cg_fltk type(shader_config):shader_path='${CGV_DIR}/libs/cgv_gl/glsl;${CGV_DIR}/libs/point_cloud/glsl;${CMAKE_CURRENT_SOURCE_DIR}' plugin:cg_icons plugin:cg_ext plugin:cmi_io plugin:crg_grid plugin:crg_light plugin:crg_stereo_view plugin:${PROJECT_NAME} config:${CMAKE_CURRENT_SOURCE_DIR}/config.def")
set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_COMMAND $<IF:$<CONFIG:Debug>,${VIEWER_DEBUG_EXE},${VIEWER_EXE}>)
set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

cgv_write_find_file(${PROJECT_NAME})
