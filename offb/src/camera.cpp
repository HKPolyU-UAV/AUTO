#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"


#include "include/run_yolo.h"
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace std;
static cv::Mat frame;

static cv::String weightpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/yt608.weights";
static cv::String cfgpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/yt608.cfg";
static cv::String classnamepath = "/home/patrick/catkin_ws/src/offb/src/include/yolo/obj.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.25));



void callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
{

    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image_dep = depth_ptr->image;
    Yolonet.getdepthdata(image_dep);

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}


int main(int argc, char** argv)
{

    cv::VideoWriter video("/home/patrick/catkin_ws/src/offb/src/include/yolo/tracking.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(640,360));
    cv::VideoWriter videoyolo("/home/patrick/catkin_ws/src/offb/src/include/yolo/yolo.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(640,360));

    // >>>> Kalman Filter
    int stateSize = 8;
    int measSize = 5;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,w,h] need z as well
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x, E_y, E_z, E_v_x, E_v_y, E_v_z,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 0 dT  0  0  0  0]
    // [ 0 1 0  0 dT  0  0  0]
    // [ 0 0 1  0  0 dT  0  0]
    // [ 0 0 0  1  0  0  0  0]
    // [ 0 0 0  0  1  0  0  0]
    // [ 0 0 0  0  0  1  0  0]
    // [ 0 0 0  0  0  0  1  0]
    // [ 0 0 0  0  0  0  0  1]


    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 0 0]
    // [ 0 1 0 0 0 0 0 0]
    // [ 0 0 1 0 0 0 0 0]
    // [ 0 0 0 0 0 0 1 0]
    // [ 0 0 0 0 0 0 0 1]


    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(9) = 1.0f;
    kf.measurementMatrix.at<float>(18) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0    0     0    0     0   0   0  ]
    // [ 0    Ey   0     0    0     0   0   0  ]
    // [ 0    0   Ez     0    0     0   0   0  ]
    // [ 0    0    0   Ev_x   0     0   0   0  ]
    // [ 0    0    0     0   Ev_y   0   0   0  ]
    // [ 0    0    0     0    0   Ev_z  0   0  ]
    // [ 0    0    0     0    0     0   Ew  0  ]
    // [ 0    0    0     0    0     0   0   Eh ]


    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(9) = 1e-2;
    kf.processNoiseCov.at<float>(18) = 1e-2;
    kf.processNoiseCov.at<float>(27) = 5.0f;
    kf.processNoiseCov.at<float>(36) = 5.0f;
    kf.processNoiseCov.at<float>(45) = 5.0f;
    kf.processNoiseCov.at<float>(54) = 1e-2;
    kf.processNoiseCov.at<float>(63) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    // Camera Index
    int idx = 0;

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 10);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Publisher publish_obj_w = nh.advertise<geometry_msgs::Point>("/pose_w",10);
    ros::Publisher publish_obj_c = nh.advertise<geometry_msgs::PointStamped>("/pose_camera",10);
    ros::Publisher publish_found = nh.advertise<std_msgs::Bool>("/obj_found",10);



    bool found = false;
    bool measured =false;
    int notFoundCount = 0;
    double dT;
    double tpf=0;
    int w = 200,h = 200;
    double ticks = 0;
    int i=0;

    cv::Point center_true;
    cv::Point center_pred;
    double depth = 0, depth_ = 0;
    double camera_z;
    double fps;
    double fps_average = 0;
    vector<double> fpss;
    double time_start, time_end;

    while(ros::ok())
    {
        time_start = ros::Time::now().toSec();
        if(!frame.empty())
        {
            cv::Mat res;
            frame.copyTo( res );
            double precTick = ticks;
            ticks = (double) cv::getTickCount();

            dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
            if (found)
            {
                // >>>> Matrix A
                kf.transitionMatrix.at<float>(3) = dT;
                kf.transitionMatrix.at<float>(12) = dT;
                kf.transitionMatrix.at<float>(21) = dT;

                // <<<< Matrix A
                //            cout << "dT:" << endl << dT << endl;
                //            cout << "State post:" << endl << state << endl;

                state = kf.predict();

                cv::Point center;
                center.x = state.at<float>(0);
                center.y = state.at<float>(1);
                double z_c_temp = state.at<float>(2);

                cv::Rect predRect;
                predRect.width = state.at<float>(6);
                predRect.height = state.at<float>(7);
                predRect.x = state.at<float>(0) - predRect.width / 2;
                predRect.y = state.at<float>(1) - predRect.height / 2;
                //                cv::Scalar color(rand()&255, rand()&255, rand()&255);
                cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
                center_pred = cv::Point(predRect.x+predRect.width/2, predRect.y+predRect.height/2);

                char temp_depth[40];
                sprintf(temp_depth, "%.2f", z_c_temp);
                string d = "Predicted z_C: ";
                string textoutputonframe =d+temp_depth + " m";
                cv::Point placetext = cv::Point((predRect.x-10),(predRect.y+predRect.height+24));
                cv::putText(res, textoutputonframe, placetext,cv::FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(255,0,0));
                depth_ = z_c_temp;
            }

            double starttime = ros::Time::now().toSec();
            Yolonet.rundarknet(frame);
            double endtime = ros::Time::now().toSec();
            double deltatime = endtime - starttime;
            //cout<<deltatime<<endl;
            fps = 1/deltatime;
            cout<<"fps: "<<fps<<endl;
            if(fpss.size()>2000)
                fpss.clear();
            fpss.push_back(fps);
            fps_average = accumulate(fpss.begin(), fpss.end(),0.0)/fpss.size();
            cout<<"fps_avg: "<<fps_average<<endl;

            vector<objectinfo> temp = Yolonet.obj_vector;
            cout<<"temp size: "<<temp.size()<<endl;

            cv::Rect interested;
            vector<objectinfo> potential;
            vector<float> potential_c;
            vector<float> potential_c_area;
            bool got=false;

            cv::Mat tempp;
            if(temp.size()!=0)
            {
                for(auto stuff:temp)
                {
                    cout<<stuff.classnameofdetection<<endl;
                    if(stuff.classnameofdetection=="Pooh")
                    {
                        potential.push_back(stuff);
                        potential_c.push_back(stuff.confidence);
                        potential_c_area.push_back(stuff.boundingbox.area());
                    }
                }
                cout<<"potential size: "<<potential.size()<<endl;

                if(potential.size()!=0)
                {
                    //                int maxElementIndex = max_element(potential_c.begin(),potential_c.end()) - potential_c.begin();
                    int maxElementIndex = max_element(potential_c_area.begin(),potential_c_area.end()) - potential_c_area.begin();
                    interested = potential[maxElementIndex].boundingbox;
                    got = true;

                    center_true=cv::Point(interested.x+interested.width/2, interested.y+interested.height/2);

                    cv::rectangle(res, interested, CV_RGB(255,255,0), 2);
                    tpf = Yolonet.appro_fps;
                    w=interested.width;
                    h=interested.height;
                    tempp = potential[maxElementIndex].frame;
                    depth = camera_z = potential[maxElementIndex].depth;

                    char temp_depth[40];
                    sprintf(temp_depth, "%.2f", depth);
                    string d = "z_C: ";
                    string textoutputonframe =d+temp_depth + " m";
                    cv::Point placetext = cv::Point((interested.x-10),(interested.y-24));
                    cv::putText(res, textoutputonframe, placetext,cv::FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(255,255,0));
                }
            }
            time_end=ros::Time::now().toSec();
            cout<<"ms: "<<time_end-time_start<<endl;
            if(!got)
            {
                notFoundCount++;
                measured = false;
                cout << "notFoundCount:" << notFoundCount << endl;
                if(notFoundCount>100)
                {
                    found = false;
                }
            }
            else
            {
                //            cout<<"hey"<<endl;
                measured = true;
                notFoundCount = 0;
                meas.at<float>(0) = interested.x + interested.width /  2;
                meas.at<float>(1) = interested.y + interested.height / 2;
                meas.at<float>(2) = camera_z;
                meas.at<float>(3) = interested.width;
                meas.at<float>(4) = interested.height;
                if (!found) // First detection!
                {
                    // >>>> Initialization
                    kf.errorCovPre.at<float>(0) = 1; // px
                    kf.errorCovPre.at<float>(9) = 1; // px
                    kf.errorCovPre.at<float>(18) = 1;
                    kf.errorCovPre.at<float>(27) = 1;
                    kf.errorCovPre.at<float>(36) = 1; // px
                    kf.errorCovPre.at<float>(45) = 1; // px
                    kf.errorCovPre.at<float>(54) = 1; // px
                    kf.errorCovPre.at<float>(63) = 1; // px

                    state.at<float>(0) = meas.at<float>(0);
                    state.at<float>(1) = meas.at<float>(1);
                    state.at<float>(2) = meas.at<float>(2);
                    state.at<float>(3) = 0;
                    state.at<float>(4) = 0;
                    state.at<float>(5) = 0;
                    state.at<float>(6) = meas.at<float>(3);
                    state.at<float>(7) = meas.at<float>(4);
                    // <<<< Initialization

                    kf.statePost = state;
                    found = true;
                }
                else
                    kf.correct(meas); // Kalman Correction
            }
            cv::Mat display;

            geometry_msgs::PointStamped send;
            if(measured)
            {
                cout<<"show measure: "<<endl;
                send.point.x = center_true.x;
                send.point.y = center_true.y;
                send.point.z = depth;
            }
            else
            {
                cout<<"show predict"<<endl;
                send.point.x = center_pred.x;
                send.point.y = center_pred.y;
                send.point.z = depth_;
            }            
            publish_obj_c.publish(send);

            std_msgs::Bool obj_found;
            if(found)
                obj_found.data = true;
            else
                obj_found.data = false;
            publish_found.publish(obj_found);

//            cv::hconcat(tempp, res, display);
            cv::line( res, cv::Point(320,240), center_pred, CV_RGB(100,0,255), 1, cv::LINE_AA);
            cv::Point text = cv::Point((320+center_pred.x)/2,(240+center_pred.y)/2);
            char temp_depth[40];
            sprintf(temp_depth, "%.2f", depth);
            string d = "distance: ";
            string textoutputonframe =d+temp_depth + " m";
            cv::putText(res, textoutputonframe, text,cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(180,140,120));
            cv::imshow("Yolo", frame);
            cv::imshow("Tracking...", res);
            cv::waitKey(20);
            video.write(res);
            videoyolo.write(frame);
        }
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

void testtrack()
{

}




