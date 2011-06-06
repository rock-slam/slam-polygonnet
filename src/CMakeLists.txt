rock_library(polygonnet
    SOURCES dummy.cpp)

rock_executable(polygonnet_bin main.cpp
    DEPS polygonnet)

