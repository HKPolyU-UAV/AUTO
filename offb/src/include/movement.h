#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <fstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <numeric>

#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "visualization_msgs/Marker.h"

#include <vector>
#include <stdio.h>
#include <ros/console.h>

using namespace std;

typedef struct UAVpose
{
    double x;
    double y;
    double z;
    double ow;
    double ox;
    double oy;
    double oz;
}UAVpose;

typedef struct waypts
{
    double x;
    double y;
    double z;
}waypts;


class movement
{
    waypts wp_i={0,0,0};
    waypts wp_h={0,0,2};
    waypts wp_1={0,9,2};
    waypts wp_2={-8,5,2};
    waypts wp_3={-8,0,2};


    //~~~
    waypts wp_f={0,0,2};

    waypts local_destination;
    waypts local_startpoint;
    waypts local_trajectory;

    vector<waypts> waypoints;
    vector<waypts> waypoints_alternative;

    double dv = 0.5, x_v, y_v, z_v, dw = 0.25;
    double distance;
    double dt;
    double start_time;

    double distancex, distancey, distancez, distanceall, current_yaw, d_yaw;;


    double calculateorientation(waypts start, waypts end);
    void switchflymode(ros::Time now, UAVpose currentinfo);
    void depthcal(cv::Point whichpixel, cv::Mat depth);

    int indicator = 0;
    int wp_indicator = 0;

//MSTG
    double _qp_cost;
    Eigen::MatrixXd _Q, _M, _Ct;
    Eigen::VectorXd _Px, _Py, _Pz;

    Eigen::MatrixXd getQ(const int p_num1d, const int order, const Eigen::VectorXd &Time, const int seg_index);

    Eigen::MatrixXd getM(const int p_num1d, const int order, const Eigen::VectorXd &Time, const int seg_index);

    Eigen::MatrixXd getCt(const int seg_num, const int d_order);

    Eigen::VectorXd closedFormCalCoeff1D(
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &M,
        const Eigen::MatrixXd &Ct,
        const Eigen::VectorXd &WayPoints1D,
        const Eigen::VectorXd &StartState1D,
        const Eigen::VectorXd &EndState1D,
        const int seg_num,
        const int d_order);

public:
    movement(vector<waypts>);
    ~movement();

    double Q2rpy(UAVpose currentinfo);
    void rpy2Q(geometry_msgs::PoseStamped &pose, double yaw);
    void go(UAVpose currentinfo, geometry_msgs::PoseStamped &pose, ros::Time &last_request, ros::Time instane_time);
    void justmove(UAVpose currentinfo, geometry_msgs::PoseStamped &pose, double &last_request, ros::Time instant_time, waypts start,waypts end);
    void setorientation(UAVpose currentinfo, geometry_msgs::PoseStamped &pose, double &last_request, ros::Time instant_time,waypts start, waypts end);
    void go_circle(UAVpose currentinfo, waypts set_destination, geometry_msgs::PoseStamped &pose, ros::Time &last_request, ros::Time instane_time);
    void sway(UAVpose currentinfo, geometry_msgs::PoseStamped &pose);
    void hover(geometry_msgs::PoseStamped &pose, waypts hoverlocation);
    bool checkcollision(UAVpose currentinfo, double last_request, cv::Mat frame, cv::Mat depth);
    Eigen::Matrix <double, 4, 1> futurepoint(waypts start, waypts end);//the 4th future time step state

    bool switchflymode_;
    double total_time;
    void set_waypoints();
    void resetindicator(bool change)
    {
        if (change)
            indicator = 0;

    }

    //MSTG

    Eigen::MatrixXd PolyQPGeneration(
        const int order,
        const Eigen::MatrixXd &Path,
        const Eigen::MatrixXd &Vel,
        const Eigen::MatrixXd &Acc,
        const Eigen::VectorXd &Time);

    int Factorial(int x);
    double desired_yaw;

};
