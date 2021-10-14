#include <stdio.h>
#include "run_yolo.h"
#include <math.h>
#include <ros/ros.h>

using namespace std;

run_yolo::run_yolo(const cv::String cfgfile, const cv::String weightfile, const cv::String objfile, const float confidence)
{
    this->cfg_file = cfgfile;
    this->weights_file = weightfile;
    this->obj_file = objfile;
    this->set_confidence = confidence;
    //the above are all usable in this class
    this->mydnn = cv::dnn::readNetFromDarknet(cfg_file, weights_file);
    
    this->mydnn.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    this->mydnn.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    //uncomment the below if CUDA available
    //this->mydnn.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    //this->mydnn.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

}
run_yolo::~run_yolo()
{

}



void run_yolo::rundarknet(cv::Mat &frame)
{
    obj_vector.clear();
    this->total_start = std::chrono::steady_clock::now();
    findboundingboxes(frame);
    this->total_end = std::chrono::steady_clock::now();
    total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    this->appro_fps = total_fps;
}

void run_yolo::display(cv::Mat frame)
{
    cv::imshow("Yolo-ed", frame);
    cv::waitKey(20);

}

void run_yolo::getdepthdata(cv::Mat depthdata)
{
    this->depthdata = depthdata;
}

void run_yolo::findboundingboxes(cv::Mat &frame)
{
    cv::Mat blob;
    blob = cv::dnn::blobFromImage(frame, 0.00392, cv::Size(608, 608), cv::Scalar(), true, false);
    //    as the dnn::net function does not accept image from image, it only receive blob hence the above function, refer to teams/article/blob

    //instatiate cv::dnn::Net object
//    cv::dnn::Net mydnnnet = cv::dnn::readNetFromDarknet(cfg_file, weights_file);
//    mydnnnet.setPreferableTarget(cv::dnn::DNN_BACKEND_DEFAULT);
//    mydnnnet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    mydnn.setInput(blob);
    //feed 4-D blob to darknet dnn

    vector<cv::String> net_outputNames;//names of output layer of yolo, should be 3
    net_outputNames = mydnn.getUnconnectedOutLayersNames();

    vector<cv::Mat> netOutput;
    //coppy the result to this object


    double starttime = ros::Time::now().toSec();
    mydnn.forward(netOutput, net_outputNames);
    //    cout<<netOut[0].size()<<endl;
    //    cout<<netOut[1].size()<<endl;
    //    cout<<netOut[2].size()<<endl<<endl;
    double endtime = ros::Time::now().toSec();
    double deltatime = endtime - starttime;
    cout<<"time:"<<deltatime<<endl;
    cout<<"fps: "<<1/deltatime<<endl;

    findwhichboundingboxrocks(netOutput, frame);
}

void run_yolo::findwhichboundingboxrocks(vector<cv::Mat> &netOutput, cv::Mat &frame)
{
    vector<float> confidenceperbbox;
    vector<int> indices;
    vector<cv::Rect> bboxes;
    vector<string> classnames;
    vector<int> classids;

    getclassname(classnames);

    int indicator =0;
    for(auto &output: netOutput)//read every layer's output, the auto variable of "output" indicates 3 different layer, as per the architecture of yolo
    {
        for(int i=0;i<output.rows;i++)//now, for every layer's output, there will be 17328*(5+class number), 4332*(5+class number), 1083*(5+class number) numbers, it holds the very info of every predicted bounding boxes
        {
            auto isthereanobjectconfidence = output.at<float> (i,4);//save the confidence of every bounding box
            if(isthereanobjectconfidence>set_confidence)//this does: assess whether there is an object in this bounding box
            //if there is, further extract the data
            {
                auto x =output.at<float>(i,0) * frame.cols;
                auto y =output.at<float>(i,1) * frame.rows;
                auto w =output.at<float>(i,2) * frame.cols;
                auto h =output.at<float>(i,3) * frame.rows;
                //                auto c_max =output.at<float>(i,4);
                //                auto c_no1 =output.at<float>(i,5);
                //                auto c_no2 =output.at<float>(i,6);
                //                auto c_no3 =output.at<float>(i,7);
                //                auto c_no4 =output.at<float>(i,8);
                //                auto c_no5 =output.at<float>(i,9);
                auto x_ = int(x - w/2);
                auto y_ = int(y - h/2);
                auto w_ = int(w);
                auto h_ = int(h);
                cv::Rect Rect_temp(x_,y_,w_,h_);

                //                cout<<"here: "<<c_max<<" "<<c_no1<<" "<<c_no2<<" "<<c_no3<<" "<<c_no4<<" "<<c_no5<<" "<<endl<<endl;
                for(int class_i=0;class_i<classnames.size();class_i++)//as for this step, this for loop take the probabilities of every class
                {
                    auto confidence_each_class = output.at<float>(i, 5+class_i); //6th element will be the 1st class confidence, class id=0
                        //7th element will be the 2nd class confidence, class id=1, etc
                    if(confidence_each_class>set_confidence)
                    {
                        bboxes.push_back(Rect_temp);
                        confidenceperbbox.push_back(confidence_each_class);
                        classids.push_back(class_i);
                    }
                }
            }
        }
    }

    cv::dnn::NMSBoxes(bboxes,confidenceperbbox,0.1,0.1,indices);
    //Basically, the indicies return the index of the bboxes, i.e, show which bounding box is the most suitable one
    for(int i =0 ; i < indices.size();i++)
    {
        int index = indices[i];

        int final_x, final_y, final_w, final_h;
        final_x = bboxes[index].x;
        final_y = bboxes[index].y;
        final_w = bboxes[index].width;
        final_h = bboxes[index].height;
        cv::Scalar color;

        cv::Point center = cv::Point(final_x+final_w/2, final_y+final_h/2);
        int depthbox_w = final_w*0.25;
        int depthbox_h = final_h*0.25;

        cv::Point depthbox_vertice1 = cv::Point(center.x - depthbox_w/2, center.y - depthbox_h/2);
        cv::Point depthbox_vertice2 = cv::Point(center.x + depthbox_w/2, center.y + depthbox_h/2);
        cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);

        cv::Mat ROI(depthdata, letsgetdepth);
        cv::Mat ROIframe;
        ROI.copyTo(ROIframe);
        vector<cv::Point> nonzeros;



        cv::findNonZero(ROIframe, nonzeros);
        vector<double> nonzerosvalue;
        for(auto temp : nonzeros)
        {
            double depth = ROIframe.at<ushort>(temp);
            nonzerosvalue.push_back(depth);
        }

        double depth_average;
        if(nonzerosvalue.size()!=0)
            depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(),0.0)/nonzerosvalue.size();

        cv::Point getdepth(final_x+final_w/2, final_y+final_h/2);
        double depthofboundingbox = 0.001 * depth_average;
        int temp_iy = 0;


        string detectedclass = classnames[classids[index]];
        float detectedconfidence = confidenceperbbox[index]*100;

        char temp_depth[40];
        sprintf(temp_depth, "%.2f", depthofboundingbox);
        char temp_confidence[40];
        sprintf(temp_confidence, "%.2f", detectedconfidence);     


        string textoutputonframe = detectedclass + ": " + temp_confidence + "%, "+ temp_depth + "m";


        cv::Scalar colorforbox(rand()&255, rand()&255, rand()&255);

        cv::rectangle(frame, cv::Point(final_x, final_y), cv::Point(final_x+final_w, final_y+final_h), colorforbox,2);
        cv::putText(frame, textoutputonframe, cv::Point(final_x,final_y-10),cv::FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(255,255,0));
        obj.confidence = detectedconfidence;
        obj.classnameofdetection = detectedclass;
        obj.boundingbox = cv::Rect(cv::Point(final_x, final_y), cv::Point(final_x+final_w, final_y+final_h));
        obj.depth = depthofboundingbox;
        obj.frame = frame;
        obj_vector.push_back(obj);
    }

}

void run_yolo::getclassname(vector<std::string> &classnames)
{
    ifstream class_file(obj_file);
    if (!class_file)
    {
        cerr << "failed to open classes.txt\n";
    }

    string line;
    while (getline(class_file, line))
    {
        classnames.push_back(line);
    }
}


