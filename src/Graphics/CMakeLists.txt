cmake_minimum_required(VERSION 3.0.0)
project(OpenGL_LearningIMGUI)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find packages
find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(Bullet REQUIRED)
find_package(GLM REQUIRED)

# Include directories
include_directories(
    ${OPENGL_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${SDL2_IMAGE_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
)

# Add your source files
set(SOURCES
    Graphics/Main.cpp   
    Graphics/Screen.cpp
    Graphics/Input.cpp
    Graphics/glad.c
    Graphics/Shader.cpp
    Graphics/Quad.cpp
    Graphics/Buffer.cpp
    Graphics/Camera.cpp
    Graphics/Texture.cpp
    Graphics/Light.cpp
    Graphics/Cube.cpp
    Graphics/Utility.cpp
    Graphics/Model.cpp
    Graphics/Material.cpp
    Graphics/Grid.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link against libraries
target_link_libraries(${PROJECT_NAME}
    ${OPENGL_LIBRARIES}
    ${SDL2_LIBRARIES}
    /opt/homebrew/Cellar/glew/2.2.0_1/lib/libGLEW.2.2.0.dylib
    /opt/homebrew/Cellar/sdl2_image/2.6.3_1/lib/libSDL2_image-2.0.0.dylib
    ${GLM_LIBRARIES}
)

# Add a custom target to run the binary
add_custom_target(run
    COMMAND ${PROJECT_NAME}
    DEPENDS ${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running ${PROJECT_NAME}"
)

# set(CMAKE_CXX_FLAGS
#     "-framework OpenGL 
# 	-framework Cocoa 
# 	-framework IOKit 
# 	-framework CoreVideo 
# 	-framework CoreFoundation 
# 	-framework GLUT "
# )


