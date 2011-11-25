include(FindPkgConfig)

pkg_check_modules(POLYGONNET "polygonnet")

qt4_wrap_ui(ui_cpp Design.ui)
qt4_wrap_cpp(moc_cpp window.h glwidget.h)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/test)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(SOURCES camera.cpp glwidget.cpp window.cpp tools.cpp main.cpp)
rock_executable(test_${PROJECT_NAME}_bin ${SOURCES} ${moc_cpp} ${ui_cpp}
                             DEPS_PKGCONFIG base-lib glu QtOpenGL)
target_link_libraries(test_${PROJECT_NAME}_bin ${EXTERNAL_LIBS} ${POLYGONNET_LIBRARIES})

