set(BUILD_TESTS OFF CACHE BOOL "Enable BUILD_TESTS")
set(BUILD_EXAMPLES OFF CACHE BOOL "Enable BUILD_EXAMPLES")

add_subdirectory(ros EXCLUDE_FROM_ALL)
add_subdirectory(Sophus EXCLUDE_FROM_ALL)

#set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Enable testing")
#set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Enable parse tools")
#set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "Enable contrib stuff in library")
#set(YAML_CPP_INSTALL OFF CACHE BOOL "Enable generation of install target")
#set(YAML_BUILD_SHARED_LIBS ON CACHE BOOL "Build Shared Libraries")
#add_subdirectory(yaml-cpp)