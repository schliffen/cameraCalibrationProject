#include <iostream>
#include <time.h>
#include <stdio.h>

#include "src/loadParameters.h"
#include "src/extrinsic_calibration.h"
#include "src/testSpeed.h"


#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif



static void help()
{
    std::cout <<  "This is a camera calibration sample." << std::endl
         <<  "Usage: calibration configurationFile"  << std::endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it." << std::endl;
}



bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat&  cameraMatrix, cv::Mat& distCoeffs,
                           std::vector<std::vector<cv::Point2f> > imagePoints );

int main(int argc, char* argv[])
{
    help();
    Settings s;
    //
    // read intrinsic settings
    const std::string inputSettingsFile = argc > 1 ? argv[1] : "../configs/initial_settings.xml";
    // read initial settings

    // ----------------------------< TRAINING >----------------------
    s.read_initSettings( inputSettingsFile );
    s.read_radar_json();
    //assuming we have parameters at hand
    vehicle_detector detector( s );
    detector.collect_target_position(s); // temp inactive
    // reading vehicle positions
    s.read_vehicle_positions();
    extrinsic_calibration autocalibrate( s );
    // auto calibration here
    autocalibrate.process();


    // --------------------< TESTING >--------------------------

    s.testflag = 1;
    s.read_radar_json();
    //detector.collect_target_position(s);
    s.read_vehicle_positions();
    // reading the calibration results
    s.read_projection_mat();
    // creating speed computation part
    testSpeed get_speed( s );
    //
    get_speed.readVehicTrack(s);
    get_speed.computeSpeed();


























    return 0;
}

