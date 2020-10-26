cmake_minimum_required(VERSION 3.4.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(/home/sam/Downloads/OpenCV-android-sdk/sdk/native/jni/include)
include_directories(src/main/cpp/OpenCL)
link_directories(src/main/cpp/OpenCL/lib64)
add_library(lib_opencv SHARED IMPORTED)
set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION /home/sam/Downloads/OpenCV-android-sdk/sdk/native/libs/${ANDROID_ABI}/libopencv_java3.so)

add_library(hello-jnicallback SHARED
        src/main/cpp/hello-jnicallback.cpp)

add_library( openclexample1
        SHARED
        src/main/cpp/OpenCLexample1.cpp
        src/main/cpp/openCLNR.cpp
        src/main/cpp/refNR.cpp
        )

find_library( log-lib
              log )
# Include libraries needed for hello-jnicallback lib
target_link_libraries(hello-jnicallback
        android
        lib_opencv
        log)


target_link_libraries( openclexample1
        ${log-lib}
        android
        OpenCL
        -ljnigraphics)
