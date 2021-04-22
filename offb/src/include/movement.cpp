#include "movement.h"
#include <algorithm>

movement::movement(vector<waypts> pts)
{

}

movement::~movement()
{

}
void movement::go(UAVpose currentinfo, geometry_msgs::PoseStamped &pose, ros::Time &last_request, ros::Time instane_time)
{
    if(indicator == 0 && wp_indicator<=waypoints.size())
    {
        cout<<"reset...next waypoint"<<endl;
        distancex = waypoints[wp_indicator+1].x - waypoints[wp_indicator].x;
        distancey = waypoints[wp_indicator+1].y - waypoints[wp_indicator].y;
        distancez = waypoints[wp_indicator+1].z - waypoints[wp_indicator].z;
        distance = sqrt(pow(distancex, 2) + pow(distancey, 2) + pow(distancez, 2));
        local_startpoint = waypoints[wp_indicator];
        local_destination = waypoints[wp_indicator+1];
        local_trajectory = local_startpoint;
        total_time = distance / dv;
        x_v = distancex / total_time;
        y_v = distancey / total_time;
        z_v = distancez / total_time;

        start_time = instane_time.now().toSec();
        cout<<x_v<<endl;
        cout<<y_v<<endl;
        cout<<z_v<<endl;
        wp_indicator++;
    }
    indicator++;

    switchflymode(instane_time, currentinfo);

    dt = instane_time.now().toSec() - start_time;

    if(!switchflymode_)
    {
//        setorientation(currentinfo, local_destination, pose); //2D
        pose.pose.position.x = local_startpoint.x + dt * x_v;
        pose.pose.position.y = local_startpoint.y + dt * y_v;
        pose.pose.position.z = local_startpoint.z + dt * z_v;

    }
    if(switchflymode_)
    {
        indicator = 0;
    }
}

void movement::justmove(UAVpose currentinfo, geometry_msgs::PoseStamped &pose, double &last_request, ros::Time instant_time,waypts start, waypts end)
{
    if(indicator==0)
    {
        distancex = end.x - start.x;
        distancey = end.y - start.y;
        distancez = end.z - start.z;
        distance  = sqrt(pow(distancex,2) + pow(distancey,2) + pow(distancez,2));

        total_time = distance / dv;
        x_v = distancex / total_time;
        y_v = distancey / total_time;
        z_v = distancez / total_time;

        start_time = instant_time.now().toSec();
        cout<<x_v<<endl;
        cout<<y_v<<endl;
        cout<<z_v<<endl;
        cout<<distance<<endl;
        cout<<"hi"<<endl;
        indicator = 1;
    }
    switchflymode(instant_time, currentinfo);

    dt = instant_time.now().toSec() - start_time;

    if(!switchflymode_)
    {
//        setorientation(currentinfo, local_destination, pose); //2D
        pose.pose.position.x = start.x + dt * x_v;
        pose.pose.position.y = start.y + dt * y_v;
        pose.pose.position.z = start.z + dt * z_v;
//        cout<<pose.pose.position.x<<endl;
//        cout<<pose.pose.position.y<<endl;
//        cout<<pose.pose.position.z<<endl;

    }
    if(switchflymode_)
    {
        indicator = 0;
    }
}

void movement::setorientation(UAVpose currentinfo, geometry_msgs::PoseStamped &pose, double &last_request, ros::Time instant_time,waypts start, waypts end)
{

    if(indicator==0)
    {
        switchflymode_ = false;
        desired_yaw = calculateorientation(start, end);
        current_yaw = Q2rpy(currentinfo);

        if (current_yaw>=M_PI)  current_yaw-=2*M_PI;
        if (current_yaw<=-M_PI) current_yaw+=2*M_PI;
        if (desired_yaw>=M_PI)    desired_yaw-=2*M_PI;
        if (desired_yaw<=-M_PI)   desired_yaw+=2*M_PI;
        d_yaw = desired_yaw - current_yaw;
        indicator = 1;
        total_time = abs(d_yaw)/dw;
        start_time = instant_time.now().toSec();
        cout<<current_yaw<<endl;
        cout<<desired_yaw<<endl;
        cout<<d_yaw<<endl;

    }
    switchflymode(instant_time, currentinfo);
    dt = instant_time.now().toSec() - start_time;
    cout<<switchflymode_<<endl;
    if(!switchflymode_)
    {
        double temp;
        if(d_yaw>0)
            temp=current_yaw+dt*dw;
        else if(d_yaw<0)
            temp=current_yaw-dt*dw;
        cout<<"temp: "<<temp<<endl;
        rpy2Q(pose, temp);

    }
    if(switchflymode_)
    {
        indicator = 0;
    }

//    if(current_yaw - desired_yaw > 0.02 || current_yaw - desired_yaw < -0.02)
//    {
//        if(desired_yaw > current_yaw)
//        {
//            current_yaw += 0.1;
//        }
//        else if (desired_yaw < current_yaw)
//        {
//            current_yaw -= 0.1;
//        }
//        else
//        {
//            //do nothing
//        }
//    }
//    cout<<desired_yaw<<endl<<endl;
//    cout<<desired_yaw<<endl;
}

void movement::switchflymode(ros::Time now, UAVpose currentinfo)
{
    double deltax = currentinfo.x-local_destination.x;
    double deltay = currentinfo.y-local_destination.y;
    double deltaz = currentinfo.z-local_destination.z;
    double delta  =sqrt(pow(deltax,2)+pow(deltay,2)+pow(deltaz,2));
    if(now.toSec() - start_time > total_time /*&& delta < 0.2*/)
    {
        switchflymode_ = true;
    }
    else
    {
        switchflymode_ = false;
    }
//    cout<<currentinfo.x-local_destination.x<<endl;
//    cout<<currentinfo.y-local_destination.y<<endl;
//    cout<<currentinfo.z-local_destination.z<<endl<<endl;
//    double deltax = currentinfo.x-local_destination.x;
//    double deltay = currentinfo.y-local_destination.y;
//    double deltaz = currentinfo.z-local_destination.z;
//    double delta  =sqrt(pow(deltax,2)+pow(deltay,2)+pow(deltaz,2));
//    cout<<delta<<endl<<endl;
//    if(delta<0.5)
//    {
//        switchflymode_ = true;
//    }
//    else
//    {
//        switchflymode_ = false;
//    }
}

double movement::calculateorientation(waypts start, waypts end)
{
    double sine, cosine;
    double vectorx = end.x - start.x;
    double vectory = end.y - start.y;
    sine = vectory/ sqrt(pow(vectorx,2)+pow(vectory,2));
    cosine = vectorx/ sqrt(pow(vectorx,2)+pow(vectory,2));
    return atan2(sine, cosine);
}

double movement::Q2rpy(UAVpose currentinfo)
{
    double sin, cos;
    sin = 2 * (currentinfo.ow * currentinfo.oz + currentinfo.ox * currentinfo.oy);
    cos = 1 - 2 * (currentinfo.oy * currentinfo.oy + currentinfo.oz * currentinfo.oz);
    double yaw = atan2(sin, cos);
    return yaw;
}

void movement::rpy2Q(geometry_msgs::PoseStamped &pose, double yaw)
{
    pose.pose.orientation.w = cos(0) * cos (0) * cos (yaw/2) + sin (0) * sin (0) * sin (yaw/2);
    pose.pose.orientation.x = sin(0) * cos (0) * cos (yaw/2) - cos (0) * sin (0) * sin (yaw/2);
    pose.pose.orientation.y = cos(0) * sin (0) * cos (yaw/2) + sin (0) * cos (0) * sin (yaw/2);
    pose.pose.orientation.z = cos(0) * cos (0) * sin (yaw/2) - sin (0) * sin (0) * cos (yaw/2);
}

void movement::sway(UAVpose currentinfo, geometry_msgs::PoseStamped &pose)
{
    double yaw_now = Q2rpy(currentinfo);
    rpy2Q(pose, yaw_now+0.16);
}

void movement::hover(geometry_msgs::PoseStamped &pose, waypts hoverlocation)
{
    pose.pose.position.x = hoverlocation.x;
    pose.pose.position.y = hoverlocation.y;
    pose.pose.position.z = hoverlocation.z;
}

bool movement::checkcollision(UAVpose currentinfo, double last_request, cv::Mat frame, cv::Mat depth)
{

}

void movement::depthcal(cv::Point whichpixel, cv::Mat loadeddepth)
{

}

Eigen::Matrix <double,4,1> movement::futurepoint(waypts start, waypts end)
{
    double nowx, nowy, nowz;
    nowx=start.x+(dt)*x_v;
    nowy=start.y+(dt)*y_v;
    nowz=start.z+(dt)*z_v;

    double x,y,z;
    x=start.x+(dt+1.6)*x_v;
    y=start.y+(dt+1.6)*y_v;
    z=start.z+(dt+1.6)*z_v;

    double temp_now_to_predict = sqrt(pow(nowx-x,2) + pow(nowy-y,2) + pow(nowz-z,2));

//    double nowx, nowy, nowz;
//    nowx=start.x+(dt)*x_v;
//    nowy=start.y+(dt)*y_v;
//    nowz=start.z+(dt)*z_v;
    double temp_now_to_end = sqrt(pow(end.x-nowx,2) + pow(end.y-nowy,2) + pow(end.z-nowy,2));

    Eigen::Matrix<double,4,1> returnvalue;

    if(temp_now_to_predict> temp_now_to_end)
    {
        returnvalue(0) = end.x;
        returnvalue(1) = end.y;
        returnvalue(2) = end.z;
        returnvalue(3) = 1;
    }
    else
    {
        returnvalue(0) = x;
        returnvalue(1) = y;
        returnvalue(2) = z;
        returnvalue(3) = 1;
    }

    cout<<"nowla:"<<endl;
    cout<<nowx<<endl;
    cout<<nowy<<endl;
    cout<<nowz<<endl;

    cout<<x<<endl;
    cout<<y<<endl;
    cout<<z<<endl<<endl;

    return returnvalue;
}

//define factorial function, input i, output i!
int movement::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd movement::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    cout<<"hi"<<endl;
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment
    int n_seg = Time.size();
    cout<<p_order<<endl;
    cout<<p_num1d<<endl;
    cout<<n_seg<<endl;;
    // the number of segments
    Eigen::MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(n_seg, 3 * p_num1d);// position(x,y,z), so we need (3 * p_num1d) coefficients

    Eigen::VectorXd Px(p_num1d * n_seg);
    Eigen::VectorXd Py(p_num1d * n_seg);
    Eigen::VectorXd Pz(p_num1d * n_seg);

    Eigen::MatrixXd startpt(d_order,3);
    Eigen::MatrixXd endpt(d_order,3);

    startpt.row(0)=Path.row(0);
    startpt.row(1)=Vel.row(0);
    startpt.row(2)=Acc.row(0);
    endpt.row(0)=Path.row(Path.rows()-1);
    endpt.row(1)=Vel.row(1);
    endpt.row(2)=Acc.row(1);
    cout<<"hi"<<endl<<endl;;

    if(d_order == 4)//use minimum snap
    {
        cout<<"hi in if"<<endl;

        startpt.row(3) = Eigen::VectorXd::Zero(3); //j0 = 0
        endpt.row(3) = Eigen::VectorXd::Zero(3);//jf = 0
    }
//instantiate Q M & Ct


    _Q = Eigen::MatrixXd::Zero(p_num1d * n_seg, p_num1d * n_seg);
    _M = Eigen::MatrixXd::Zero(p_num1d * n_seg, p_num1d * n_seg);
    _Ct = Eigen::MatrixXd::Zero(2 * d_order * n_seg, d_order * (n_seg + 1));
    cout<<1<<endl;
    for(int seg_index = 0; seg_index < n_seg; seg_index++)
    {
        _Q.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getQ(p_num1d, d_order, Time, seg_index);
        _M.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getM(p_num1d, d_order, Time, seg_index);
    }
    cout<<1<<endl;

    _Ct = getCt(n_seg, d_order);
    Px = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(0), startpt.col(0), endpt.col(0), n_seg, d_order);
    Py = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(1), startpt.col(1), endpt.col(1), n_seg, d_order);
    Pz = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(2), startpt.col(2), endpt.col(2), n_seg, d_order);

    for(int i = 0; i < n_seg; i++)
    {
        PolyCoeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(2*p_num1d, p_num1d) = Pz.segment(p_num1d*i, p_num1d); //segment refers to starting from that index, and extend for howmany
    }


    return PolyCoeff;
}

Eigen::MatrixXd movement::getQ(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
    Eigen::VectorXd time = Eigen::VectorXd::Zero(p_num1d);
    for(int i = 0; i < p_num1d; i++)
    {
        time(i) = pow(Time(seg_index),i);
    }
    if(p_num1d == 6)        // minimum jerk
    {
        Q_k << 0,     0     ,     0     ,      0     ,       0     ,       0     ,
            0,     0     ,     0     ,      0     ,       0     ,       0     ,
            0,     0     ,     0     ,      0     ,       0     ,       0     ,
            0,     0     ,     0     ,  36*time(1),   72*time(2),  120*time(3),
            0,     0     ,     0     ,  72*time(2),  192*time(3),  360*time(4),
            0,     0     ,     0     , 120*time(3),  360*time(4),  720*time(5);
    }
    else if(p_num1d == 8)   // minimum snap
    {
        Q_k << 0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
            0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
            0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
            0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
            0,     0    ,      0     ,      0     ,  576*time(1),  1440*time(2),  2880*time(3),   5040*time(4),
            0,     0    ,      0     ,      0     , 1440*time(2),  4800*time(3), 10800*time(4),  20160*time(5),
            0,     0    ,      0     ,      0     , 2880*time(3), 10800*time(4), 25920*time(5),  50400*time(6),
            0,     0    ,      0     ,      0     , 5040*time(4), 20160*time(5), 50400*time(6), 100800*time(7);
    }
    // cout << " Q_k = " << endl;
    // cout <<  Q_k << endl;

    return Q_k;
}

Eigen::MatrixXd movement::getM(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    Eigen::MatrixXd M_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
    Eigen::VectorXd time = Eigen::VectorXd::Zero(p_num1d);
    for(int i = 0; i < p_num1d; i++)
    {
        time(i) = pow(Time(seg_index),i);
    }
    if(p_num1d == 6)        // minimum jerk
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,
            0,     1   ,     0     ,     0     ,      0     ,      0     ,
            0,     0   ,     2     ,     0     ,      0     ,      0     ,
            1,  time(1),    time(2),    time(3),     time(4),     time(5),
            0,     1   ,  2*time(1),  3*time(2),   4*time(3),   5*time(4),
            0,     0   ,     2     ,  6*time(1),  12*time(2),  20*time(3);
    }
    else if(p_num1d == 8)   // minimum snap
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
            0,     1   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
            0,     0   ,     2     ,     0     ,      0     ,      0     ,      0     ,      0     ,
            0,     0   ,     0     ,     6     ,      0     ,      0     ,      0     ,      0     ,
            1,  time(1),    time(2),    time(3),     time(4),     time(5),     time(6),     time(7),
            0,     1   ,  2*time(1),  3*time(2),   4*time(3),   5*time(4),   6*time(5),   7*time(6),
            0,     0   ,     2     ,  6*time(1),  12*time(2),  20*time(3),  30*time(4),  42*time(5),
            0,     0   ,     0     ,     6     ,  24*time(1),  60*time(2), 120*time(3), 210*time(4);
    }
    // cout << "M_k = " << endl;
    // cout << M_k << endl;

    return M_k;
}

Eigen::MatrixXd movement::getCt(const int n_seg, const int d_order)
{
    int d_num = 2 * d_order * n_seg;
    int dF_dP_num = (n_seg + 1) * d_order;

    Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(d_num, dF_dP_num);

    Eigen::MatrixXd C_temp = Eigen::MatrixXd::Zero(2 * d_order, dF_dP_num);

    Ct.block(0, 0, d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);

    if(d_order == 3)//use minimum jerk
    {
        for(int k = 0; k < n_seg - 1; k++)
        {
            C_temp(0, d_order + k) = 1;
            C_temp(1, d_order + d_order + (n_seg - 1) + 2*k) = 1;
            C_temp(2, d_order + d_order + (n_seg - 1) + 2*k + 1) = 1;

            C_temp(3, d_order + k) = 1;
            C_temp(4, d_order + d_order + (n_seg - 1) + 2*k) = 1;
            C_temp(5, d_order + d_order + (n_seg - 1) + 2*k + 1) = 1;

            Ct.block(d_order + 2 * d_order * k, 0, 2 * d_order, dF_dP_num) = C_temp;
            C_temp = Eigen::MatrixXd::Zero(2 * d_order, dF_dP_num);
        }
    }

    if(d_order == 4)//use minimum snap
    {
        for(int k = 0; k < n_seg - 1; k++)
        {
            C_temp(0, d_order + k) = 1;
            C_temp(1, d_order + d_order + (n_seg - 1) + 3*k) = 1;
            C_temp(2, d_order + d_order + (n_seg - 1) + 3*k + 1) = 1;
            C_temp(3, d_order + d_order + (n_seg - 1) + 3*k + 2) = 1;

            C_temp(4, d_order + k) = 1;
            C_temp(5, d_order + d_order + (n_seg - 1) + 3*k) = 1;
            C_temp(6, d_order + d_order + (n_seg - 1) + 3*k + 1) = 1;
            C_temp(7, d_order + d_order + (n_seg - 1) + 3*k + 2) = 1;

            Ct.block(d_order + 2 * d_order * k, 0, 2 * d_order, dF_dP_num) = C_temp;
            C_temp = Eigen::MatrixXd::Zero(2 * d_order, dF_dP_num);
        }
    }
    Ct.block((n_seg - 1) * 2 * d_order + d_order , d_order + (n_seg - 1) , d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);

    // cout << "Ct = " << endl;
    // cout << Ct << endl;
    return Ct;
}

Eigen::VectorXd movement::closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                                                  const Eigen::MatrixXd &M,
                                                                  const Eigen::MatrixXd &Ct,
                                                                  const Eigen::VectorXd &WayPoints1D,
                                                                  const Eigen::VectorXd &startPointState1D,
                                                                  const Eigen::VectorXd &endPointState1D,
                                                                  const int n_seg,
                                                                  const int d_order)
{
    int dF_dP_num = d_order * (n_seg + 1);

    int df_num = 2 * d_order + (n_seg - 1);
    int dp_num = (d_order - 1) * (n_seg - 1);

    Eigen::MatrixXd C = Ct.transpose();
    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::MatrixXd R = C * M_inv_tran * Q * M_inv * Ct;
    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);


    Eigen::VectorXd dF(df_num);
    dF.head(d_order) = startPointState1D;//start state:p0,v0,a0,j0
    dF.segment(d_order, (n_seg - 1)) = WayPoints1D.segment(1,WayPoints1D.rows()-2);
    dF.tail(d_order) = endPointState1D;//end state:pf,vf,af,jf
    // cout << "dF = " << endl;
    // cout << dF << endl;

    Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;//closed form



    Eigen::VectorXd dF_and_dP(dF_dP_num);
    dF_and_dP << dF, dP;

    Eigen::VectorXd PolyCoeff1D = M_inv * Ct * dF_and_dP;//on same direction

    return PolyCoeff1D;
}

