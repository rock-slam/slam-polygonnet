# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)


find_package(Qt4 REQUIRED QtGui QtCore QtOpenGL)
set(QT_USE_QTOPENGL TRUE)
add_definitions(${QT_DEFINITIONS})
include(${QT_USE_FILE})

list(APPEND CUSTOM_CFLAGS ${QT_INCLUDE_DIR}
         ${QT_QTCORE_INCLUDE_DIR}
         ${QT_QTOPENGL_INCLUDE_DIR}
         ${QT_QTGUI_INCLUDE_DIR}
         ${QT_QTXML_INCLUDE_DIR}
)
string(REGEX REPLACE ";" " -I" ADD_CFLAGS "${CUSTOM_CFLAGS}")
set(ADD_CFLAGS "-I${ADD_CFLAGS}")
message("Add cflags: ${ADD_CFLAGS}")

rock_init(polygonnet 0.1)
rock_standard_layout()
