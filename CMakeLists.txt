cmake_minimum_required(VERSION 3.12)
project(MyApp)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Add the source files and include directories
file(GLOB_RECURSE SOURCES src/*.cpp src/Physics/*.cpp)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(/opt/homebrew/include)
include_directories(/opt/homebrew/Cellar/sdl2_image/2.6.3_1/include)
include_directories(/opt/homebrew/Cellar/sdl2/2.28.2/include)

# Specify the executable target
add_executable(app ${SOURCES})

# Link libraries and specify library directories
target_link_libraries(app 
/opt/homebrew/Cellar/sdl2_image/2.6.3_1/lib/libSDL2_image-2.0.0.dylib
 -lm 
 -lSDL2 
 -lSDL2_image 
 -lSDL2_gfx)

link_directories(/opt/homebrew/lib)  # Specify the library directory
link_directories(/opt/homebrew/Cellar/sdl2_image/2.6.3_1/lib)
link_directories(/opt/homebrew/Cellar/sdl2/2.28.2/lib)

# Define a custom target for 'run'
add_custom_target(run
    COMMAND ./app
    DEPENDS app
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Define a custom target for 'clean'
add_custom_target(cleanapp
    COMMAND rm -f app
)
