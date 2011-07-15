include_directories(${PROJECT_SOURCE_DIR}/external/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(GSL_LIBRARY "/usr/lib/libgsl.so;/usr/lib/libgslcblas.so")
set(GL_LIBRARY "/usr/lib/libglut.so")
set(OPENCV_LIBRARIES "/usr/lib/libcv.so;/usr/lib/libhighgui.so;/usr/lib/libcxcore.so")
### Update external libraries that are available
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${Boost_LIBRARIES} ${BASE_LIBRARIES} ${OPENCV_LIBRARIES} ${QT_LIBRARIES} ${GSL_LIBRARY} ${GL_LIBRARY})

# Add your .ui files ot the following line (and uncomment it)
#QT4_WRAP_UI(ui_cpp myproject.ui)
# Add the .h files that define QObject subclasses (that have a Q_OBJECT
# statement)
QT4_WRAP_CPP(moc_cpp RegularTriangularMesh.h fitting.h)

set(SOURCES addPointsThread.cpp face.cpp point.cpp RegularTriangularMesh.cpp anchor.cpp fitting.cpp drawingRoutines.cpp)

rock_library(${PROJECT_NAME} SHARED ${SOURCES} ${moc_cpp} ${ui_cpp}
			     DEPS_CMAKE OpenCV
		 	     DEPS_PKGCONFIG base-lib)
target_link_libraries(${PROJECT_NAME} ${EXTERNAL_LIBS})

#rock_executable(test_bin main.cpp DEPS test)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/external/include/cml DESTINATION include/${PROJECT_NAME})
