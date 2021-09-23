#include <iostream>
#include <ros/ros.h>
#include "include/movement.h"
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Bool.h>


static UAVpose uavinfo;
static double obj_x_c, obj_y_c, obj_z_c;
static double x_prev = 0, y_prev = 0, z_prev = 0;



void pooh_gt_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ofstream save("/home/patrick/track_ws/src/offb/src/log/pooh_gt.txt",ios::app);
    save<<pose->pose.position.x<<endl;
    save<<pose->pose.position.y<<endl;
    save<<pose->pose.position.z<<endl;
    save<<pose->pose.orientation.w<<endl;
    save<<pose->pose.orientation.x<<endl;
    save<<pose->pose.orientation.y<<endl;
    save<<pose->pose.orientation.z<<endl;
    save.close();

    uavinfo.x = pose->pose.position.x;
    uavinfo.y = pose->pose.position.y;
    uavinfo.z = pose->pose.position.z;
    uavinfo.ow = pose->pose.orientation.w;
    uavinfo.ox = pose->pose.orientation.x;
    uavinfo.oy = pose->pose.orientation.y;
    uavinfo.oz = pose->pose.orientation.z;
}

void position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ofstream save("/home/patrick/track_ws/src/offb/src/log/mav_pos.txt",ios::app);
    save<<pose->pose.position.x<<endl;
    save<<pose->pose.position.y<<endl;
    save<<pose->pose.position.z<<endl;
    save.close();
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& velocity)
{
    ofstream save("/home/patrick/track_ws/src/offb/src/log/mav_vel.txt",ios::app);
    save<<velocity->twist.linear.x<<endl;
    save<<velocity->twist.linear.y<<endl;
    save<<velocity->twist.linear.z<<endl;
    save<<velocity->twist.angular.x<<endl;
    save<<velocity->twist.angular.y<<endl;
    save<<velocity->twist.angular.z<<endl;
    save.close();
}

void obj_info_cb(const geometry_msgs::PointStampedConstPtr& msg)
{
    obj_x_c = msg->point.x;
    obj_y_c = msg->point.y;
    obj_z_c = msg->point.z;
    if(obj_x_c==0)
        obj_x_c = x_prev;
    if(obj_y_c==0)
        obj_y_c = y_prev;
    if(obj_z_c==0)
        obj_z_c = z_prev;
    if(obj_x_c!=0)
        x_prev = obj_x_c ;
    if(obj_y_c!=0)
        y_prev = obj_y_c;
    if(obj_z_c!=0)
        z_prev = obj_z_c;

}

static double fx, fy, cx, cy; //focal length and principal point

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
}

static bool meaornot;

void mea_cb(const std_msgs::Bool::ConstPtr& msg)
{
    meaornot = msg->data;
}

waypts c2w(Eigen::Matrix<double, 3, 1> camera_pt)
{
    double z = camera_pt(2),
           x = camera_pt(0),
           y = camera_pt(1);

    Eigen::Matrix<double, 4, 1> cam (x,y,z,1), body, world, offset (0.14, -0.02,0,0);
    Eigen::Matrix<double, 4, 4> cam_to_body;
    cam_to_body << 0,0,1,0,
        -1,0,0,0,
        0,-1,0,0,
        0,0,0,1;

    Eigen::Matrix<double, 3, 3> matrix_for_q;
    Eigen::Quaterniond q2r_matrix(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);
    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world <<
        matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), uavinfo.x,
        matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), uavinfo.y,
        matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), uavinfo.z,
        0,0,0,1;

    world = body_to_world * cam_to_body * cam;
    waypts world_pt = {world(0), world(1), world(2)};
//    cout<<uavinfo.x<<endl;
//    cout<<uavinfo.y<<endl;
//    cout<<uavinfo.z<<endl;
//    cout<<world_pt.x<<endl;
//    cout<<world_pt.y<<endl;
//    cout<<world_pt.z<<endl<<endl;;


    return world_pt;
}

void pooh_percept()
{
    double z = obj_z_c; // the distance between center of the object is surface depth + object size
    double x = z * (obj_x_c - cx)/fx;  //pixel coordinate u,v -> camera coordinate x,y
    double y = z * (obj_y_c - cy)/fy;
    Eigen::Matrix<double,3,1> fromcameranode;
    fromcameranode(0) = x;
    fromcameranode(1) = y;
    fromcameranode(2) = z;
    waypts locationofpooh = c2w(fromcameranode);
    ofstream save("/home/patrick/track_ws/src/offb/src/log/serachpooh.txt",ios::app);
    save<<meaornot<<endl;
    save<<locationofpooh.x<<endl;
    save<<locationofpooh.y<<endl;
    save<<locationofpooh.z<<endl;
    save.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inforecorder");
    ros::NodeHandle nh;

    ros::Subscriber pooh_info_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                   ("/vrpn_client_node/pooh/pose", 1, pooh_gt_cb);
    ros::Subscriber mav_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                   ("/mavros/local_position/pose", 1, position_callback);
    ros::Subscriber mav_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                   ("/mavros/local_position/velocity_local", 1, velocity_callback);
    ros::Subscriber sub_obj = nh.subscribe<geometry_msgs::PointStamped>
                              ("/pose_camera", 1, obj_info_cb);
    ros::Subscriber sub_mea= nh.subscribe<std_msgs::Bool>
                              ("/obj_found", 1, mea_cb);
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);



    ros::Rate rate(20.0);
    while(ros::ok())
    {
        pooh_percept();
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return 0;




}



