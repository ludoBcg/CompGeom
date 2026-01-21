# check CMAKE version
cmake_minimum_required(VERSION 3.0)

# create project
project(CompGeom)

# use C++ 20
set(CMAKE_CXX_STANDARD 20)

# add files
set(SRCS
	src/main.cpp
	src/vkcontext.cpp
	src/vkapp.cpp
	src/mesh.cpp
	src/dynamicmesh.cpp
	src/surfacemesh.cpp
	src/image.cpp
	src/point.cpp
	src/spring.cpp
	src/massspringsystem.cpp
	src/numericalintegration.cpp
	src/arap.cpp
	src/fem.cpp
    )
    
set(HEADERS
	src/vkutils.h
	src/vkcontext.h
	src/vkapp.h
	src/mesh.h
	src/dynamicmesh.h
	src/surfacemesh.h
	src/image.h
	src/point.h
	src/spring.h
	src/massspringsystem.h
	src/numericalintegration.h
	src/arap.h
	src/fem.h
    )

	
# Add include directories
include_directories(SYSTEM "${CMAKE_CURRENT_SOURCE_DIR}/src" )
# Dependencies folder
set(LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../libs")
include_directories(SYSTEM "${LIBS_DIR}")


############################### EXTERNAL DEPENDENCIES ################

# OpenMP
find_package(OpenMP)


# Vulkan
include_directories("C:/VulkanSDK/1.3.250.1/Include")
link_directories("C:/VulkanSDK/1.3.250.1/Lib")
SET(VULKAN_LIBS vulkan-1.lib)
# flag for conditional compilation
add_compile_definitions(USE_VULKAN)


# GLFW (to compile before)
set(GLFW_DIR "${LIBS_DIR}/third_party/glfw-3.4")
include_directories(${GLFW_DIR}/include)
link_directories(${GLFW_DIR}/build/src/Release)
SET(GLFW_LIBS glfw3.lib)


# GLM (Header only)
include_directories(SYSTEM "${LIBS_DIR}/third_party/glm-1.0.1")


# Eigen (Header only)
include_directories(SYSTEM "${LIBS_DIR}/third_party/eigen-3.4.0")


################################# BUILD PROJECT ######################

# Add executable for project
add_executable(${PROJECT_NAME} ${SRCS} ${HEADERS})

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX ${GLFW_LIBS} ${VULKAN_LIBS})

# Install executable
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

