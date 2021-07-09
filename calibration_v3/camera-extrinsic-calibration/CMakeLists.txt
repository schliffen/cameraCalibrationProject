cmake_minimum_required(VERSION 2.8)
project( CalibrateCamera )
find_package( OpenCV REQUIRED )


add_library( autoCalibration
        src/loadParameters.cpp
        src/loadParameters.h
        src/extrinsic_calibration.cpp
        src/extrinsic_calibration.h
        src/testSpeed.cpp src/testSpeed.h)


include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable( ${PROJECT_NAME} main.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries( ${PROJECT_NAME} autoCalibration)