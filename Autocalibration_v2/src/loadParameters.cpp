//
// Created by ai on 6/22/21.
//

#include "loadParameters.h"



//void Settings::read_radar_json() {
//    //
//    cv::FileStorage fs;
//    if (!testflag)
//        fs.open( video_settings_path, 0);
//    else
//        fs.open( test_video_settings_path, 0);
//
//    cv::FileNode root = fs[ "object" ];
//    root[ "snapshot" ]["cameraArea"]["rectangle"]["upperLeft"]["x"] >> radar_min_position.x;
//    root[ "snapshot" ]["cameraArea"]["rectangle"]["upperLeft"]["y"] >> radar_min_position.y;
//    //
//    root[ "snapshot" ]["cameraArea"]["rectangle"]["lowerRight"]["x"] >> radar_max_position.x;
//    root[ "snapshot" ]["cameraArea"]["rectangle"]["lowerRight"]["y"] >> radar_max_position.y;
//
//    // reading radar m/s speed
//    root[ "snapshot" ]["speed"] >> radar_speed;
//
//    // height-width of the video for scaling back the position
//    root[ "snapshot" ]["imageWidth"] >> video_width;
//    root[ "snapshot" ]["imageHeight"] >> video_height;
//    root["preEventTimeForVideo"] >> radar_trigger;
//
//
//}

void Settings::read_radar_json() {
    //
    cv::FileStorage fs;
    if (!testflag)
        fs.open( video_settings_path, 0);

    else
        fs.open( test_video_settings_path, 0);

    cv::FileNode root = fs["initialSnapshot"];

    root["cameraArea"]["rectangle"]["upperLeft"]["x"] >> radar_min_position.x;
    root["cameraArea"]["rectangle"]["upperLeft"]["y"] >> radar_min_position.y;

    //
    root["cameraArea"]["rectangle"]["lowerRight"]["x"] >> radar_max_position.x;
    root["cameraArea"]["rectangle"]["lowerRight"]["y"] >> radar_max_position.y;

    root["imageWidth"] >> radar_width;
    root["imageHeight"] >> radar_height;

    // reading radar m/s speed
    root["speed"] >> radar_speed;

    radar_min_position.x = std::abs(radar_min_position.x) * radar_width;
    radar_max_position.x = std::abs(radar_max_position.x) * radar_width;
    radar_min_position.y = std::abs(radar_min_position.y) * radar_height;
    radar_max_position.y = std::abs(radar_max_position.y) * radar_height;
    // height-width of the video for scaling back the position

    fs["preEventTimeForVideo"] >> radar_trigger;
    std::vector<cv::Point2f> A(2);
    A[0] = radar_min_position;
    A[1] = radar_max_position;
    cv::perspectiveTransform(A, mulHomography, homography_matrix);
    radar_min_position.x = mulHomography.at<float>(0, 0);
    radar_min_position.y = mulHomography.at<float>(0, 1);
    radar_max_position.x = mulHomography.at<float>(0, 2);
    radar_max_position.y = mulHomography.at<float>(0, 3);

}

void Settings::read_homography_matrix()
{
    //fs;
    //if (isFileExist(homography_matrix_path))
    //{
        cv::FileStorage fs(homography_matrix_path, cv::FileStorage::READ);
        fs["mat"] >> homography_matrix;
    //}
    //else
    //{
    //    std::fputs("Error: homography matrix not exist or can not open\n", stderr); exit(1);
    //}
}

void Settings::read_projection_mat() {
    //
    FILE * poCamParamFl = std::fopen( estimated_projection_path.c_str(), "r");
    if ( poCamParamFl == NULL ) { std::fputs("Error: camera parameters not loaded\n", stderr); exit(1); }
    std::ifstream ifsCamParam;
    ifsCamParam.open( estimated_projection_path.c_str() );
    int nLnCnt = 0;
    while (!ifsCamParam.eof())
    {
        char acBuf[256] = { 0 };
        ifsCamParam.getline(acBuf, 256);
        std::sscanf(acBuf, "%f %f %f %f %f %f %f %f %f %f %f %f",
                        &afProjMat[0],  &afProjMat[1], &afProjMat[2], &afProjMat[3], &afProjMat[4],
                        &afProjMat[5],  &afProjMat[6], &afProjMat[7], &afProjMat[8], &afProjMat[9],
                        &afProjMat[10], &afProjMat[11]);

        nLnCnt++;
    }
    ifsCamParam.close();

}


vehicle_detector::vehicle_detector(Settings s)
{
    // std::string net_cfg_path, std::string net_weights_path, std::string net_class_names_path, std::string video_path
    //_net_cfg_path = net_cfg_path;
    //_net_weights_path = net_weights_path;
    //_net_class_names_path = net_class_names_path;
    //_video_path = video_path;
    // check if video path exists
    //    std::ifstream f( s.video_file_path.c_str());
    //    if (! f.good() )
    //    {
    //        std::cerr << "File does not exist" << std::endl;
    //    }

    std::ifstream class_file( s.net_class_names_path );
    if (!class_file)
    {
        std::cerr << "failed to open _net_class_names_path \n";
    }
    std::string line;
    std::vector<std::string> _net_class_names_vec;
    while (std::getline(class_file, line))
        _net_class_names_vec.push_back(line);

    for (int i=0; i<NUM_CLASSES; i++)
        _class_names_vec.insert(std::pair<int, std::string>(std::stoi(_net_class_names_vec.at(2*i)), _net_class_names_vec.at(2*i +1 ) ) );

    // load the network
    _net = cv::dnn::readNetFromDarknet((cv::String) s.net_cfg_path, (cv::String) s.net_weights_path);
    if(_net.empty())
    {
        std::cerr << "Can't load network by using the following files: " << std::endl;
        std::cerr << "cfg-file:     " << s.net_cfg_path << std::endl;
        std::cerr << "weights-file: " << s.net_weights_path << std::endl;
        std::cerr << "Models can be downloaded here:" << std::endl;
        std::cerr << "https://pjreddie.com/darknet/yolo/" << std::endl;
        exit(-1);
    }

}



/// Initialize network
int vehicle_detector::detect_vehicle(cv::Mat frame)
{
    indices->clear();
    scores->clear();
    boxes->clear();
    //
    //cv::namedWindow("Resized", cv::WINDOW_NORMAL);
    cv::resize(frame, _resized, cv::Size(_network_width, _network_height));
    cv::Mat inputBlob = cv::dnn::blobFromImage(_resized, 1 / 255.F); // Convert Mat to batch of images
    _net.setInput(inputBlob, "data");                                // set the network input
    /// Make forward pass
    auto outnames = _net.getUnconnectedOutLayersNames();
    _net.forward( _detectionMat, outnames);
    // compute output
    std::map<int, std::string>::iterator it = _class_names_vec.begin();
    for (auto& output : _detectionMat)
    {
        const auto num_boxes = output.rows;
        for (int i = 0; i < num_boxes; i++)
        {
            auto x = output.at<float>(i, 0) * frame.cols;
            auto y = output.at<float>(i, 1) * frame.rows;
            auto width = output.at<float>(i, 2) * frame.cols;
            auto height = output.at<float>(i, 3) * frame.rows;
            cv::Rect rect(x - width/2, y - height/2, width, height);
            // just filling the desired classes
            //for (int c = 0; c < NUM_CLASSES; c++)
            int c=0;
            while (it != _class_names_vec.end())
            {
                auto confidence = *output.ptr<float>(i, 5 + it->first);
                if ( (confidence >= _net_min_confidence) )
                {
                    boxes[c].push_back(rect);
                    scores[c].push_back(confidence);
                }
                it++;
                c++;
            }
            it = _class_names_vec.begin();
        }
    }
    for (int c = 0; c < NUM_CLASSES; c++)
        cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);
    return 0;
}

int vehicle_detector::select_candid_vehicle( cv::Mat frame, cv::Rect_<float> & scaled_radar, cv::Rect_<float> & candid_vehicle, float & iou){
    // calculating maximum IOU with the radar
    // get width and height
    cv::resize(frame, frame, cv::Size(_network_width, _network_height));

    // comparing with vehicles frame
    detect_vehicle( frame );

    float iou_tmp;


    for (int c= 0; c < NUM_CLASSES; c++)
    {
        for (size_t i = 0; i < indices[c].size(); ++i)
        {

            auto idx = indices[c][i];
            const auto& rect = boxes[c][idx];
            iou_tmp = 0;
            // clculating the iou with the target vehicle from radar
            int iou_x_min = std::max((int) ((scaled_radar.x - scaled_radar.width/2  )* _network_width) , rect.x );
            int iou_y_min = std::max((int) ((scaled_radar.y - scaled_radar.height/2)* _network_height), rect.y );
            int iou_x_max = std::min( (int) ((scaled_radar.x + scaled_radar.width/2)*_network_width), rect.x + rect.width);
            int iou_y_max = std::min( (int) ((scaled_radar.y + scaled_radar.height/2)*_network_height), rect.y + rect.height);

            if ( (iou_x_max - iou_x_min)<0 ||  (iou_y_max - iou_y_min)<0 ) continue;

            iou_tmp = (iou_x_max - iou_x_min) * (iou_y_max - iou_y_min) /(float) (scaled_radar.width * scaled_radar.height * _network_height * _network_width + rect.width * rect.height);

            //if (iou_tmp > iou) {
                iou = iou_tmp;
                candid_vehicle.x = (rect.x + rect.width/2)  / (float)_network_width;
                candid_vehicle.y = (rect.y + rect.height/2) / (float)_network_height;
                candid_vehicle.width = rect.width   / (float)_network_width;
                candid_vehicle.height = rect.height / (float)_network_height;
            //}

        }
    }
}


bool cmp(std::pair<float, std::pair <cv::Rect, int> >& a,
         std::pair<float, std::pair <cv::Rect, int> >& b)
{
    return a.first < b.first;
}


int vehicle_detector::select_correct_frame( Settings &s ){
    //
    cv::VideoCapture cap;
    if (!s.testflag)
        cap.open( s.video_file_path );
    else
        cap.open( s.test_video_file_path );
    // calculate start frame number

    s.fps = cap.get(cv::CAP_PROP_FPS);

    int count = cap.get(cv::CAP_PROP_FRAME_COUNT);

    int start_search_frame = ( s.radar_trigger - 2 )*s.fps; // todo: change these values in final release
    int end_search_frame = ( s.radar_trigger + 2 )*s.fps;  // todo: change these values in final release
    //cap.set(CV_CAP_PROP_POS_FRAMES, start_frame_number);
    cv::namedWindow("vehicle", cv::WINDOW_NORMAL);
    //
    cv::Rect_<float> scaled_radar;
    cv::Rect_<float> candid_vehicle;
    float iou=-1;
    //

    //cv::namedWindow("radarf", cv::WINDOW_FREERATIO);
    //cv::Mat rfrm = cv::imread(s.radar_trigger_frame_path);


    s.video_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    s.video_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    scaled_radar.x = ((std::abs(s.radar_min_position.x) + std::abs(s.radar_max_position.x)) * .5) / s.video_width;
    scaled_radar.width = (std::abs(s.radar_max_position.x) - std::abs(s.radar_min_position.x)) / s.video_width;
    scaled_radar.height = scaled_radar.width;
    scaled_radar.y = ((std::abs(s.radar_min_position.y) + std::abs(s.radar_max_position.y)) * 0.5) / s.video_height;



    //cv::Point p1  (std::abs(s.radar_min_position.x) , std::abs(s.radar_min_position.y) );
    //cv::Point p2  (std::abs(s.radar_max_position.x) , std::abs(s.radar_max_position.y) );
    //cv::rectangle(rfrm, p1, p2, cv::Scalar(200,1,1), 4 );

//    cv::resize( rfrm, rfrm, cv::Size(600,600));
//    cv::imshow( "radarf", rfrm);
//    cv::waitKey(0);

    count = 1;
    //std::map<float, std::pair <cv::Rect, int> > candid_frames;
    const auto color = colors[100 % NUM_COLORS];

    cv::Mat frame, max_frame;
    int framenum = 0;
    while (cap.read(frame)) {
        //cap >> frame; // get a new frame from camera/video or read image
        framenum++;
        if (frame.empty()) {
            cv::waitKey();
            break;
        } else if (count < 0 || framenum < start_search_frame)
            continue;
        else if (framenum <= end_search_frame) {
            select_candid_vehicle( frame,  scaled_radar,  candid_vehicle,  iou );
            if (iou >= max_iot){
                init_frame_num = framenum;
                init_candid_vehicle = candid_vehicle;
                max_iot = iou;

                frame.copyTo(max_frame); // for visualization
                cv::resize(max_frame, max_frame, cv::Size(_network_width, _network_height));
                int x1 = (init_candid_vehicle.x -init_candid_vehicle.width/2)* _network_width;
                int y1 = (init_candid_vehicle.y - init_candid_vehicle.height/2) * _network_height;
                int w1 = x1 + init_candid_vehicle.width * _network_width;
                int h1 = y1 + init_candid_vehicle.height * _network_height;
                //
                int x2 = (scaled_radar.x - scaled_radar.width/2)  * _network_width;
                int w2 = x2 +  scaled_radar.width * _network_width;
                int y2 = (scaled_radar.y -  scaled_radar.height/2 ) * _network_height;
                int h2 = y2 +   scaled_radar.height * _network_height;
                //
                // detected
                cv::rectangle(max_frame, cv::Point(x1 , y1), cv::Point(w1, h1), cv::Scalar( 200, 0, 0), 3);
                cv::rectangle(max_frame, cv::Point(x2 ,y2 ),  cv::Point(w2, h2) , cv::Scalar(0, 0, 200), 3);
                cv::circle(max_frame, cv::Point( (x2 + w2)/2, (y2+ h2)/2) , 3, cv::Scalar(0, 200, 0), 2);
                //


                //std::ostringstream label_ss;
                cv::imwrite( "target_vehicle.png", max_frame);
                //cv::imshow("vehicle", max_frame);
                //cv::waitKey(0);


            }
        }
    }


    cap.release();
    // plotting the results
    // comparing the boxes and the radars
    std::cout << " target car detected! \n";
}

int vehicle_detector::collect_target_position( Settings &s){
    //
    cv::VideoCapture cap;
    if (!s.testflag){
        select_correct_frame( s );
        cap.open( s.video_file_path );}
    else {
        select_correct_frame( s );
        cap.open(s.test_video_file_path);
    }

    // calculate start frame number

    int count = cap.get(cv::CAP_PROP_FRAME_COUNT);
    cv::namedWindow("vehicle", cv::WINDOW_NORMAL);

    count = 1;
    cv::Mat frame, max_frame;
    int frame_limit, framenum = 0;
    float cont_dist;
    std::vector<cv::Rect_<float>> target_vehicle;
    std::vector<int> frame_index;
    target_vehicle.push_back(init_candid_vehicle);
    if (!s.testflag){

        frame_limit = 25;}
    else
        frame_limit = 10;

    int prev_frame = 0;

    while (cap.read(frame)){
        //cap >> frame; // get a new frame from camera/video or read image
        framenum++;
        if(frame.empty())
        {
            cv::waitKey();
            break;
        }else if (count<0 || framenum <= init_frame_num)
            continue;
        else {
            // resizing the frame
            // collecting target vehicle positions
            cv::Rect_<float>  candid_vehicle;
            float iou=0.1;
            //cv::imshow("frame",  frame); cv::waitKey(0);
            select_candid_vehicle( frame, target_vehicle.at(prev_frame), candid_vehicle, iou);
            // todo: check if it is acceptable detection -> check if box does not cross the image boundary
            // calculate the distance of two consequtive framse for controlling
            cont_dist = std::sqrt( std::pow(target_vehicle.at(prev_frame).x - candid_vehicle.x,2) +  std::pow(target_vehicle.at(prev_frame).y - candid_vehicle.y,2));


            if (candid_vehicle.width ==0 || candid_vehicle.height ==0 )
                continue;
            // check if the bbox touched the boundary
            if (target_vehicle.size()>frame_limit || cont_dist < 0.000002 || candid_vehicle.x>0.9 || candid_vehicle.y>0.9)
                break;

            target_vehicle.push_back(candid_vehicle);
            frame_index.push_back(framenum);
            prev_frame++;

            frame.copyTo(max_frame); // for visualization
            cv::resize(max_frame, max_frame, cv::Size(_network_width, _network_height));
            int x1 = (target_vehicle.at(prev_frame ).x -target_vehicle.at(prev_frame ).width/2 )* _network_width;
            int y1 = (target_vehicle.at(prev_frame ).y -target_vehicle.at(prev_frame ).height/2 )* _network_height;
            int w1 = x1 + target_vehicle.at(prev_frame ).width * _network_width;
            int h1 = y1 + target_vehicle.at(prev_frame ).height * _network_height;

            //int x2 = (target_vehicle.at(framenum - init_frame_num).x -target_vehicle.at(framenum - init_frame_num).width/2 )* _network_width;
            //int y2 = (target_vehicle.at(framenum - init_frame_num).y -target_vehicle.at(framenum - init_frame_num).height/2 )* _network_height;;
            //int w2 = x2 + target_vehicle.at(framenum - init_frame_num ).width * _network_width;
            //int h2 = y2 + target_vehicle.at(framenum - init_frame_num ).height * _network_height;

            // detected
            cv::rectangle(max_frame, cv::Point(x1 , y1), cv::Point(w1, h1), cv::Scalar( 200, 0, 0), 2);
            //cv::rectangle(max_frame, cv::Point(x2 ,y2 ),  cv::Point(w2, h2) , cv::Scalar(0, 0, 200), 1);
            //cv::circle(max_frame, cv::Point( (x2 + w2)/2, (y2+ h2)/2) , 3, cv::Scalar(0, 200, 0), 2);
            //std::ostringstream label_ss;
            cv::imshow("tracking", max_frame);
            cv::waitKey(0);

            // collecting only 25 samples is enough
            //std::cout<< "distance: " << cont_dist << std::endl;

        }

    }
    cap.release();

    // write collected positions into a txt file for later use
    std::ofstream Vehicle_position;
    if (!s.testflag)
        Vehicle_position.open( s.train_target_position_path ); //
    else
        Vehicle_position.open(s.test_target_position_path ); //

    if (Vehicle_position.is_open())
    {
        // write the frame per second
        Vehicle_position << s.fps << "\n";

        Vehicle_position << s.video_width << "\n";
        Vehicle_position << s.video_height << "\n";


        for(int i=0; i < frame_index.size(); i++) // auto it = std::begin(target_vehicle); it != std::end(target_vehicle); ++it
            Vehicle_position << frame_index[i] << " "<< target_vehicle[i+1].x << " " << target_vehicle[i+1].y << " " << target_vehicle[i+1].width << " " << target_vehicle[i+1].height << "\n";
            Vehicle_position.close();
    }

    return 0;

}



/*
 *
     for(int i = 0; i < _detectionMat.rows; i++)
    {

        const int probability_size = _detectionMat.cols - _probability_index;
        float *prob_array_ptr = &_detectionMat.at<float>(i, _probability_index);

        size_t objectClass = std::max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
        float confidence = _detectionMat.at<float>(i, (int)objectClass + _probability_index);

        if(confidence > _net_min_confidence)
        {
            float x = _detectionMat.at<float>(i, 0);
            float y = _detectionMat.at<float>(i, 1);
            float width = _detectionMat.at<float>(i, 2);
            float height = _detectionMat.at<float>(i, 3);
            int xLeftBottom = static_cast<int>((x - width / 2) * frame.cols);
            int yLeftBottom = static_cast<int>((y - height / 2) * frame.rows);
            int xRightTop = static_cast<int>((x + width / 2) * frame.cols);
            int yRightTop = static_cast<int>((y + height / 2) * frame.rows);

            cv::Rect object(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);

            //
            //_core->drawRect(object);

            std::cout << "Class: " << objectClass << std::endl; // TODO:
            std::cout << "Confidence: " << confidence << std::endl;
            std::cout << " " << xLeftBottom << " " << yLeftBottom << " " << xRightTop << " " << yRightTop
                      << std::endl;

            if(objectClass < _net_class_names_vec.size())
            {
                _ss.str("");
                _ss << confidence;
                cv::String conf(_ss.str());

                cv::String label = cv::String(_net_class_names_vec[objectClass]) + ": " + conf;
                int baseLine = 0;

                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::rectangle(frame, cv::Rect(cv::Point(xLeftBottom, yLeftBottom - labelSize.height),
                                              cv::Size(labelSize.width, labelSize.height + baseLine)),
                              cv::Scalar(255, 255, 255), CV_FILLED);
                cv::putText(frame, label, cv::Point(xLeftBottom, yLeftBottom), cv::FONT_HERSHEY_SIMPLEX,
                            0.5, cv::Scalar(0, 0, 0));


                cv::imshow("frame", frame);
                cv::waitKey(0);

            }
            else
            {
            }
        }

    }

 *
 *
 */