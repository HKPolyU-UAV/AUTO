#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>


#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <math.h>
#include <stdio.h>
#include <numeric>
#include <chrono>
#include <iomanip>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>


using namespace std;
typedef struct objectinfo
{
    float confidence = 0;
    string classnameofdetection;
    cv::Rect boundingbox;
    cv::Mat frame;
    double depth;
}objectinfo;

class run_yolo
{
    cv::String cfg_file;
    cv::String weights_file;
    cv::String obj_file;
    cv::Mat depthdata;

    float set_confidence;
    vector<std::string> classnames;

    void findboundingboxes(cv::Mat &frame);
    void findwhichboundingboxrocks(vector<cv::Mat> &netOut, cv::Mat &frame);
    void getclassname(vector<std::string> &classnames);
    chrono::time_point <chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;
    float total_fps;
    objectinfo obj;
    cv::dnn::Net mydnn;

public:
    run_yolo(const cv::String cfgfile, const cv::String weightfile, const cv::String objfile, const float confidence);
    ~run_yolo();

    void rundarknet(cv::Mat &frame);
    void display(cv::Mat frame);
    void getdepthdata(cv::Mat depthdata);
    float appro_fps;
    vector<objectinfo> obj_vector;


};
