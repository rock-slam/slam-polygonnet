include_directories(${PROJECT_SOURCE_DIR}/external/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set(GSL_LIBRARY "/usr/lib/libgsl.so;/usr/lib/libgslcblas.so")
#set(GL_LIBRARY "/usr/lib/libglut.so")
#set(OPENCV_LIBRARIES "/usr/lib/libcv.so;/usr/lib/libhighgui.so;/usr/lib/libcxcore.so")

### Update external libraries that are available
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${Boost_LIBRARIES} ${QT_LIBRARIES})

# Add your .ui files ot the following line (and uncomment it)
#QT4_WRAP_UI(ui_cpp myproject.ui)
# Add the .h files that define QObject subclasses (that have a Q_OBJECT
# statement)
QT4_WRAP_CPP(moc_cpp RegularTriangularMesh.h Fitting.h)

set(SOURCES 
	AddPointsThread.cpp
	Face.cpp
	Point.cpp
	RegularTriangularMesh.cpp
	Anchor.cpp
	Fitting.cpp)

set (HEADERS 
	Anchor.h
	DataTypes.h
	Face.h
	Fitting.h
	Point.h
	RegularTriangularMesh.h
)

rock_library(${PROJECT_NAME} SOURCES ${SOURCES} ${moc_cpp} ${ui_cpp}
                             HEADERS ${HEADERS}
			     DEPS_CMAKE OpenCV
		 	     DEPS_PKGCONFIG base-lib gsl gl opencv libpng12)
target_link_libraries(${PROJECT_NAME} ${EXTERNAL_LIBS})

#rock_executable(test_bin main.cpp DEPS test)

