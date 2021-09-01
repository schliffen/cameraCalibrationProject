//
// Created by ai on 6/22/21.
//

#ifndef CALIBRATECAMERA_LOADPARAMETERS_H
#define CALIBRATECAMERA_LOADPARAMETERS_H

#include <fstream>
#include <sstream>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <iterator>
#include <iomanip>
#include <math.h>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>



//using namespace cv;
//using namespace std;

class Settings
{
public:
    Settings() {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};
    void read_radar_json();
    void read_projection_mat();
    void read_homography_matrix();
    // in case that there is an intrinsic parameters at hand
// for later development
    //    void read_intrinsic()                          //Read serialization for this class
//    {
//
//        cv::FileStorage fs(camera_intirinsic_path, cv::FileStorage::READ); // Read the settings
//        if (!fs.isOpened())
//        {
//            std::cout << "Could not open the configuration file: \"" << camera_intirinsic_path << "\"" << std::endl;
//        }
//        fs["Camera_Matrix"] >> camera_params;
//        fs["Distortion_Coefficients"] >> distortion;
//        fs.release();
//   }

   void read_initSettings(std::string inputSettingsFile ){
       //
       cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
       if (!fs.isOpened())
       {
           std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
       }
       fs["Settings"]["video_setting_path"] >> video_settings_path;
       fs["Settings"]["video_file_path"] >> video_file_path;
       fs["Settings"]["radar_trigger_frame_path"] >> radar_trigger_frame_path; // this is to coordinate radar detection results
       fs["Settings"]["test_video_setting_path"] >> test_video_settings_path;
       fs["Settings"]["test_video_file_path"] >> test_video_file_path;

       fs["Settings"]["homography_matrix_path"] >> homography_matrix_path;
       //

       fs["Settings"]["camera_intirinsic_path"] >> camera_intirinsic_path;
       //
       fs["Settings"]["homography_matrix_path"] >> homography_matrix_path;
       fs["Settings"]["net_cfg_path"] >> net_cfg_path;
       fs["Settings"]["net_weights_path"] >> net_weights_path;
       fs["Settings"]["net_class_names_path"] >> net_class_names_path;
       fs["Settings"]["test_video_setting_path"] >> test_video_settings_path;
       fs["Settings"]["test_video_file_path"] >> test_video_file_path;
       fs["Settings"]["train_target_position_path"] >> train_target_position_path;
       fs["Settings"]["test_target_position_path"] >> test_target_position_path;
       //
       fs["Settings"]["estimated_projection_path"] >> estimated_projection_path;


    }

    void read_vehicle_positions(){
        // read positions from txt as follows
        // num_frame top_left_x top_left_y width height
       std::ifstream vehicle_positions;
       if (!testflag)
           vehicle_positions.open( train_target_position_path );
       else
           vehicle_positions.open( test_target_position_path );
       //
       vehicle_centers.clear();
       std::string positiondata;
       std::string delimiter = " ";
       int counter=1;
       cv::Rect_<float> tmp_position;
       if (vehicle_positions.is_open()) {

           vehicle_positions >> positiondata;
           fps = std::stof(positiondata);
           vehicle_positions >> positiondata;
           video_width = std::stoi(positiondata);
           vehicle_positions >> positiondata;
           video_height = std::stoi(positiondata);


           while (!vehicle_positions.eof()) {

               vehicle_positions >> positiondata;
               if (counter % 5 == 1)
                   tmp_position.height = std::stof(positiondata);
               else if (counter % 5 == 2)
                   tmp_position.x = std::stof(positiondata);
               else if (counter % 5 == 3)
                   tmp_position.y = std::stof(positiondata);
               else if (counter % 5 == 4)
                   tmp_position.width = std::stof(positiondata);
               else if (counter % 5 == 0)
                   vehicle_centers.push_back( tmp_position );

               counter++;
           }

       }else{
           std::cerr<< " cannot open vehicle position file \n";
       }

    }

    // to control the inputs


public:
    cv::Point2f radar_min_position;          //
    cv::Point2f radar_max_position;
    float radar_speed;
    float radar_trigger; // radar trigger time in seconds

    float video_height, radar_height;
    float video_width, radar_width;

    std::string video_settings_path, video_file_path, camera_intirinsic_path, net_cfg_path, net_weights_path,
    net_class_names_path, train_target_position_path, test_target_position_path, test_video_settings_path,
    test_video_file_path, estimated_projection_path, radar_trigger_frame_path, homography_matrix_path;

    std::vector<cv::Rect_<float>> vehicle_centers;
    int cameraID;
    //cv::Mat mulHomography, homography_matrix;
    //std::vector<cv::String> imgList;
    //std::vector<std::string> imageList;
    //int atImageList;
    //cv::VideoCapture inputCapture;
    //InputType inputType;
    //bool goodInput;
    int testflag = 0;
    float fps;
    float afProjMat[12];

    cv::Mat camera_params, homography_matrix, mulHomography, distortion;


private:
    std::string patternToUse;


};



// colors for bounding boxes
const cv::Scalar colors[] = {
        {0, 255, 255},
        {255, 255, 0},
        {0, 255, 0},
        {255, 0, 0}
};

class vehicle_detector {
public:
//    std::string _net_cfg_path, _net_weights_path, _net_class_names_path, _video_path;
    float _net_min_confidence = 0.6;
    float NMS_THRESHOLD = 0.4;
    int _network_width = 512;
    int _network_height = 512;
    int _probability_index = 5;
    int NUM_COLORS = sizeof(colors)/sizeof(colors[0]);
    std::map<int, std::string> _class_names_vec;
    cv::dnn::Net _net;
    cv::Mat _resized;
    cv::Rect _net_bbox;
    int NUM_CLASSES = 3; // will consider up to the 8th class in coco names (car truck bus)
    std::vector<cv::Mat>  _detectionMat;
    std::vector<int> indices[3];
    std::vector<float> scores[3];
    std::vector<cv::Rect> boxes[3];
    int init_frame_num;
    float max_iot=0;
    cv::Rect_<float> init_candid_vehicle;

    std::ostringstream _ss;

public:
    vehicle_detector( Settings s);
    int detect_vehicle(cv::Mat );
    int collect_target_position( Settings & );

    int select_candid_vehicle( cv::Mat,  cv::Rect_<float> & scaled_radar, cv::Rect_<float> & candid_vehicle, float & iou );
    int select_correct_frame(  Settings &s );



};



#endif //CALIBRATECAMERA_LOADPARAMETERS_H
