#include "include/movement.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include "include/run_yolo.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"
#include "offb/obj.h"


using namespace std;

enum flying_step
{
    IDLE,
    TO2Hover,
    Hover,
    Prep,
    Starto,
    Move2wp,
    Change,
    Avoid,
    HoverandSway,
    Land,
    Move,
    Follow,
    END,
};

static flying_step  fly = IDLE;
static UAVpose uavinfo;
static mavros_msgs::State current_state;
static int indicator = 0;
static visualization_msgs::Marker UAVtrajectory;
static vector<waypts> waypoints;
static vector<waypts> referpts;
static vector<waypts> futurepts;
static cv::Mat frame;
static cv::Mat image_dep;
static visualization_msgs::Marker edge_points;
static cv::Rect whichframe_fp;
static double v;
waypts instantiation1 = {0,0,0};
waypts instantiation2 = {0,0,0};
vector<waypts> temp;
movement tool(temp);


void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


void position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo.x = pose->pose.position.x;
    uavinfo.y = pose->pose.position.y;
    uavinfo.z = pose->pose.position.z;
    uavinfo.ow = pose->pose.orientation.w;
    uavinfo.ox = pose->pose.orientation.x;
    uavinfo.oy = pose->pose.orientation.y;
    uavinfo.oz = pose->pose.orientation.z;

}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& velocity)
{
    double vx = velocity->twist.linear.x;
    double vy = velocity->twist.linear.y;
    double vz = velocity->twist.linear.z;
    v = sqrt(vx*vx+vy*vy+vz*vz);
}


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
    image_dep = depth_ptr->image;

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

static double fx, fy, cx, cy; //focal length and principal point

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
}

static double obj_x_c, obj_y_c, obj_z_c;
static double x_prev = 0, y_prev = 0, z_prev = 0;

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

static bool foundornot;

void found_cb(const std_msgs::Bool::ConstPtr& msg)
{
    foundornot = msg->data;
}

static double obj_d_v;
void obj_d_v_cb(const offb::obj::ConstPtr& msg)
{
    obj_d_v = msg->Z_c;
}


void xymove(int which, geometry_msgs::PoseStamped &pose);
int locateonframe();
waypts c2w(Eigen::Matrix<double, 3, 1> camera_pt);
int judge_d();
void zmove(int which, geometry_msgs::PoseStamped &pose);
waypts b2w(Eigen::Matrix<double, 3, 1> body_pt);

int main(int argc, char **argv)
{
    cout<<"tracking..."<<endl;
    ros::init(argc, argv, "tracking");
    ros::NodeHandle nh;

    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 1, state_callback);
    ros::Subscriber sub_uabposition = nh.subscribe<geometry_msgs::PoseStamped>
                                      ("/mavros/local_position/pose", 1, position_callback);
    ros::Subscriber sub_velocity = nh.subscribe<geometry_msgs::TwistStamped>
                                   ("/mavros/local_position/velocity_local", 1, velocity_callback);

    ros::Subscriber sub_obj = nh.subscribe<geometry_msgs::PointStamped>
                              ("/pose_camera", 1, obj_info_cb);
    ros::Subscriber sub_found= nh.subscribe<std_msgs::Bool>
                              ("/obj_found", 1, found_cb);
    ros::Subscriber sub_obj_vel = nh.subscribe<offb::obj>("/obj_v", 1, obj_d_v_cb);




    ros::Publisher pub_traj_pts = nh.advertise<geometry_msgs::PoseStamped>
                                  ("mavros/setpoint_position/local", 1);
    ros::Publisher rviz_visual = nh.advertise <visualization_msgs::Marker>("gt_points",1);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);

    geometry_msgs::PoseStamped pose;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    //    for(int i = 100; ros::ok() && i > 0; --i){
    //        pub_traj_pts.publish(pose);
    //        ros::spinOnce();
    //        rate.sleep();
    //    }

    //mode setting
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    double last_request = ros::Time::now().toSec();
    double last_request_= ros::Time::now().toSec();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    int i=0,j=0,k=0,l=0,m=0,n=0;
    double end_x=0, end_y=0, end_z=0;
    double yaw = M_PI/2;

    waypts hoverpoint1={0,0,0}, hoverpointTO={0,0,2.0};
    waypts localstartpt, localendpt;

    waypts wp1={0,0,1.4}, wp2={-2,3,1.4}, wp3={-6,-3,1.4},wp4={-11,2,1.4};
    vector<waypts> waypoints;

    movement move(waypoints);
    bool findwayptsuccess;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(ros::ok())
    {
        //preparation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == IDLE)
        {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
            {
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");

                }
                last_request = ros::Time::now().toSec();
            }
            else
            {
                if( !current_state.armed && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
                {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                        fly=TO2Hover;
                        localstartpt = hoverpoint1;
                        localendpt = wp1;
                    }
                    last_request = ros::Time::now().toSec();
                }
            }
        }

        //move~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == TO2Hover && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), localstartpt, localendpt);
            if(move.switchflymode_==true)
            {
                last_request = ros::Time::now().toSec();
                fly = HoverandSway;
            }
        }
        if(fly == HoverandSway)
        {
            move.sway(uavinfo,pose);
            cout<<"Sway"<<endl;
            if(foundornot == true)
            {
                fly = Follow;
            }
        }
        if(fly == Follow)
        {
            cout<<"Follow"<<endl;
            int which_xy = locateonframe();
            int which_zz = judge_d();
            xymove(which_xy, pose);
            zmove(which_zz, pose);

            if(foundornot == false)
            {
                fly = Land;
            }
        }

        if(fly == Land)
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), wp1, hoverpoint1);
            if(move.switchflymode_)
                fly = END;
        }


        if(fly == END)
        {
            pose.pose.position.z = pose.pose.position.z - 0.02;

            if(pose.pose.position.z < 0.05)
            {
                arm_cmd.request.value = false;
                if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success )
                {
                    cout << "UAV about to touch ground" << endl;
                    cout << "Touched and end...."<< endl;
                    return 0;//break the control UAV will land automatically
                }
             }
        }
        if(!frame.empty())
        {
            cv::line(frame, cv::Point(1,120),cv::Point(640,120),CV_RGB(200,200,1),5);
            cv::line(frame, cv::Point(1,240),cv::Point(640,240),CV_RGB(200,200,1),5);
            cv::line(frame, cv::Point(214,1),cv::Point(214,360),CV_RGB(200,200,1),5);
            cv::line(frame, cv::Point(428,1),cv::Point(428,360),CV_RGB(200,200,1),5);
            cv::circle(frame,cv::Point(obj_x_c,obj_y_c),10.0,CV_RGB(10,186,286),-1);
            cv::imshow("tracker!", frame);
            cv::waitKey(20);
        }


        pub_traj_pts.publish(pose);
//        rviz_visual.publish(vis_traj);
//        rviz_visual.publish(vis_pooh);

        ros::spinOnce();
        rate.sleep();
        last_request_=ros::Time::now().toSec();
    }
    return 0;
}

int locateonframe()
{
    cv::Point pt_frame = cv::Point(obj_x_c, obj_y_c);
    int tempx = pt_frame.x;
    int tempy = pt_frame.y;
    //1
    if(tempx>=1 && tempx<=214 && tempy>=1 && tempy<=160)
        return 1;
    //2
    if(tempx>=215 && tempx<=428 && tempy>=1 && tempy<=160)
        return 2;
    //3
    if(tempx>=429 && tempx<=640 && tempy>=1 && tempy<=160)
        return 3;
    //4
    if(tempx>=1 && tempx<=214 && tempy>=161 && tempy<=320)
        return 4;
    //5
    if(tempx>=215 && tempx<=428 && tempy>=161 && tempy<=320)
        return 5;
    //6
    if(tempx>=429 && tempx<=640 && tempy>=161 && tempy<=320)
        return 6;
    //7
    if(tempx>=1 && tempx<=214 && tempy>=321 && tempy<=480)
        return 7;
    //8
    if(tempx>=215 && tempx<=428 && tempy>=321 && tempy<=480)
        return 8;
    //9
    if(tempx>=429 && tempx<=640 && tempy>=321 && tempy<=480)
        return 9;
    //0
    else return 0;
}

void xymove(int which, geometry_msgs::PoseStamped &pose)
{
    double yaw = tool.Q2rpy(uavinfo);

    if(which==1)
    {
        tool.rpy2Q(pose,yaw+0.08);
        pose.pose.position.z = pose.pose.position.z + 0.005;
    }
    if(which==2)
    {
        pose.pose.position.z = pose.pose.position.z + 0.005;
    }
    if(which==3)
    {
        tool.rpy2Q(pose,yaw-0.08);
        pose.pose.position.z = pose.pose.position.z + 0.005;
    }

    if(which==4)
    {
        tool.rpy2Q(pose,yaw+0.08);
    }
    if(which==5)
    {

    }
    if(which==6)
    {
        tool.rpy2Q(pose,yaw-0.08);
    }

    if(which==7)
    {
        tool.rpy2Q(pose,yaw+0.08);
        pose.pose.position.z = pose.pose.position.z - 0.005;
    }
    if(which==8)
    {
        pose.pose.position.z = pose.pose.position.z - 0.005;
    }
    if(which==9)
    {
        tool.rpy2Q(pose,yaw-0.08);
        pose.pose.position.z = pose.pose.position.z - 0.005;
    }
}

waypts c2w(Eigen::Matrix<double, 3, 1> camera_pt)
{
    double depth = camera_pt(2);
    double z = camera_pt(2),
           x = camera_pt(0),
           y = camera_pt(1);

    Eigen::Matrix<double, 4, 1> cam (x,y,z,1), body, world;
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
    cout<<"from here to here"<<endl;
    cout<<uavinfo.x<<endl;
    cout<<uavinfo.y<<endl;
    cout<<uavinfo.z<<endl;
    cout<<world_pt.x<<endl;
    cout<<world_pt.y<<endl;
    cout<<world_pt.z<<endl<<endl;;


    return world_pt;
}

int judge_d()
{
    if(obj_z_c<=2.25 || obj_z_c>=3.25)
        return 1;
    else
        return 0;
}

void zmove(int which, geometry_msgs::PoseStamped &pose)
{
    double move2z_c = 0;
//    cout<<which<<endl;
    if(which == 1)
    {
        cout<<"need move"<<endl;

        Eigen::Matrix<double, 3, 1> pt_c;

        move2z_c = obj_z_c - 2.5;

        if(move2z_c > 0)
        {
            if(abs(obj_d_v) > 0.5)
                pt_c(0) = 0.5 * 1 / 20;
            else
                pt_c(0) = abs(obj_d_v * 1 / 20);
        }
        else
        {
            if(abs(obj_d_v) > 0.5)
                pt_c(0) = -0.5 * 1 / 20;
            else
                pt_c(0) = -abs(obj_d_v * 1 / 20);

        }
        cout<<pt_c(0)<<endl;
        pt_c(1)=0;
        pt_c(2)=0;
        waypts destination = b2w(pt_c);
        pose.pose.position.x = destination.x;
        pose.pose.position.y = destination.y;
        pose.pose.position.z = destination.z;

    }
    if(which == 0)
    {
        //do nothing
    }
}

waypts b2w(Eigen::Matrix<double, 3, 1> body_pt)
{
    Eigen::Matrix<double, 4, 1> body, world;
    body(0) = body_pt(0);
    body(1) = body_pt(1);
    body(2) = body_pt(2);
    body(3) = 1;
    Eigen::Matrix<double, 3, 3> matrix_for_q;
    Eigen::Quaterniond q2r_matrix(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);
    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world <<
        matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), uavinfo.x,
        matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), uavinfo.y,
        matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), uavinfo.z,
        0,0,0,1;

    world = body_to_world * body;
    waypts result = {world(0),world(1), world(2)};
    return result;

}

