#include <iostream>
#include <fstream>
#include <istream>

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



#include <numeric>

#include <chrono>
#include <iomanip>

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

public:
    run_yolo(const cv::String cfgfile, const cv::String weightfile, const cv::String objfile, const float confidence);
    ~run_yolo();

    void rundarknet(cv::Mat &frame);
    void display(cv::Mat frame);
    void getdepthdata(cv::Mat depthdata);
    float appro_fps;
    vector<objectinfo> obj_vector;


};
