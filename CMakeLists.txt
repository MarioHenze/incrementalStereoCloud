cmake_minimum_required(VERSION 2.6)
set(PROJECT_NAME "incremental_pc_view")
project(${PROJECT_NAME})


# Add CGV framework
# - configure
option(BUILD_EXAMPLE_PLUGIN "Build the example plugin" FALSE)
mark_as_advanced(FORCE BUILD_EXAMPLE_PLUGIN)
set(LIBS)
set(cgv_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cgv)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cgv ext_build/cgv)
include_directories(
	${cgv_INCLUDE_DIRS} 
	../cgv/libs 
	${cgv_gl_INCLUDE_DIRS}
)
# - prepare for use by the project
find_package(cgv 
	COMPONENTS render gl gui base type reflect utils signal data media
	HINTS ${cgv_DIR}
	)
find_package(OpenGL)
find_package(shader_test)
find_package(cgv_viewer)
cgv_find_package(GLEW)
cgv_find_package(cg_fltk)
cgv_find_package(cg_icons)
cgv_find_package(cmi_io)
cgv_find_package(cgv_gl)
cgv_find_package(crg_stereo_view)
cgv_get_viewer_locations(VIEWER_EXE VIEWER_DEBUG_EXE)
get_filename_component(FINAL_BUILD_BIN_DIR ${VIEWER_EXE} DIRECTORY)

# Add project
add_subdirectory(src)

# Launch configurations for various IDEs that are not Visual Studio
# - VS Code
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/.vscode)
configure_file(${CMAKE_SOURCE_DIR}/CMake/Templates/launch.json.in ${CMAKE_SOURCE_DIR}/.vscode/launch.json @ONLY)
# - pure Unix
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tmp)
set(EXTRA_PLUGINS "plugin:cg_ext plugin:crg_grid plugin:crg_light")
set(EXTRA_PLUGINS_DEBUG "plugin:cg_ext_d plugin:crg_grid_d plugin:crg_light_d")
set(EXTRA_ARGS "")
configure_file(${CMAKE_SOURCE_DIR}/CMake/Templates/run.sh.in ${CMAKE_BINARY_DIR}/tmp/run.sh @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/CMake/Templates/debug.sh.in ${CMAKE_BINARY_DIR}/tmp/debug.sh @ONLY)
file(COPY
	${CMAKE_BINARY_DIR}/tmp/run.sh
	DESTINATION ${CMAKE_BINARY_DIR}
	FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
file(COPY
	${CMAKE_BINARY_DIR}/tmp/debug.sh
	DESTINATION ${CMAKE_BINARY_DIR}
	FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/tmp)
