/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <sstream>
#include "../include/template_gui_package/qnode.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

#define PI 3.141592
#define D2R PI/180
#define R2D 180/PI


extern Motor motor_data;
extern Battery battery_data;
extern Joystick RtoC_joystick_data;
extern Joystick CtoR_joystick_data;
extern Movie movie_data;
extern Getgas getgas_data;
extern Getcapture getcapture_data;

Motor motor_data;
Battery battery_data;
Joystick RtoC_joystick_data;
Joystick CtoR_joystick_data;
Movie movie_data;
Getgas getgas_data;
Getcapture getcapture_data;

namespace template_gui_package {
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}


bool QNode::init() {
    ros::init(init_argc,init_argv,"kmu_gui");
    if ( ! ros::master::check() ) {
        return false;

        std::cout<<"no rosmaster"<<std::endl;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    std::cout<<"sub start"<<std::endl;
    rosmode_publisher = n.advertise<std_msgs::UInt16>("rosmode", 1);
    S_Camera_movie = n.subscribe("/usb_cam/image_raw/",      1, &QNode::Camera_Callback, this);
    test_camera = n.subscribe("usb_cam/image_raw/",1,&QNode::test_camera_Callback,this);//test1-2

    //S_top_movie    = n.subscribe("/usb_cam/image_raw/", 1, &QNode::Rviz_Top_Callback, this);
    S_top_movie    = n.subscribe("/topCamera/image/compressed/", 1, &QNode::Rviz_Top_Callback, this);
    S_tpf_movie    = n.subscribe("/tpfCamera/image/compressed/", 1, &QNode::Rviz_Tpf_Callback, this);
    S_Motor_data = n.subscribe("/RtoC_M_State", 100, &QNode::RtoC_M_Callback, this);
    S_Battery_data = n.subscribe("/RtoC_B_State", 100, &QNode::RtoC_B_Callback, this);
    S_RtoC_Joystick_data = n.subscribe("/RtoC_J_State", 100, &QNode::RtoC_J_Callback, this);
    S_CtoR_Joystick_data = n.subscribe("/joy", 100, &QNode::CtoR_J_Callback, this);

    S_RtoC_getgas_data = n.subscribe("/get_gas", 100, &QNode::RtoC_getgas_Callback, this);

    S_RtoC_getgas_data = n.subscribe("/get_capture", 100, &QNode::RtoC_getcapture_Callback, this);


    start();
    lastImgMsgTime=ros::Time::now();
    return true;
}

/*
bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;

    ros::init(remappings,"kmu_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // Add your ros communications here.
//    rosmode_publisher = n.advertise<std_msgs::UInt16>("rosmode", 1);

//    //S_Camera_movie = n.subscribe("/movie/image_raw",      1, &QNode::Camera_Callback, this);
//    S_Camera_movie = n.subscribe("/usb_cam/image_raw/compressed",      1, &QNode::Camera_Callback, this);
//    S_top_movie    = n.subscribe("/movie/top_image_raw/", 1, &QNode::Rviz_Top_Callback, this);
//    S_tpf_movie    = n.subscribe("/movie/tpf_image_raw/", 1, &QNode::Rviz_Tpf_Callback, this);
//    S_Motor_data = n.subscribe("/RtoC_M_State", 100, &QNode::RtoC_M_Callback, this);
//    S_Battery_data = n.subscribe("/RtoC_B_State", 100, &QNode::RtoC_B_Callback, this);
//    S_RtoC_Joystick_data = n.subscribe("/RtoC_J_State", 100, &QNode::RtoC_J_Callback, this);
//    S_CtoR_Joystick_data = n.subscribe("/joy", 100, &QNode::CtoR_J_Callback, this);

    start();

    lastImgMsgTime=ros::Time::now();
    return true;
}
*/

void QNode::run() {
    ros::Rate loop_rate(100); // not affect to callback subscriber to gui signal
//    int count = 0;
    ros::NodeHandle n;

    std::cout<<"qnode run";
    while ( ros::ok() ) {

        ros::Duration diff = ros::Time::now()-lastImgMsgTime;
        double rosTimeDiff_byImg= diff.toSec();


        Q_EMIT mainWindowForceUpdate(rosTimeDiff_byImg);
        ros::spinOnce();
        loop_rate.sleep();
//        ++count;
        //std::cout<<"ros spins\n";
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}

void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    }

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);

    //Q_EMIT loggingUpdated(); // used to readjust the scrollbar

}

void QNode::RtoC_M_Callback(const template_gui_package::motor& motor_msg){
    motor_data.HEADER_SEC = motor_msg.HEADER.stamp.sec;
    motor_data.HEADER_NSEC = motor_msg.HEADER.stamp.nsec;
    motor_data.NAME = motor_msg.NAME;
    motor_data.LMC_S = motor_msg.LMC_S;
    motor_data.RMC_S = motor_msg.RMC_S;
    motor_data.FMC_S = motor_msg.FMC_S;

    // LM_GAIN
//    motor_data.LMC_S = motor_msg.LMC_S;
//    motor_data.RMC_S = motor_msg.RMC_S;
//    motor_data.FMC_S = motor_msg.FMC_S;

    motor_data.LM_SPD = motor_msg.LM_SPD;
    motor_data.LW_SPD = motor_msg.LW_SPD;
    motor_data.RM_SPD = motor_msg.RM_SPD;
    motor_data.RW_SPD = motor_msg.RW_SPD;
    motor_data.FM_SPD = motor_msg.FM_SPD;
    motor_data.FW_SPD = motor_msg.FW_SPD;
    motor_data.FM_AGL = motor_msg.FM_AGL;
    motor_data.FW_AGL = motor_msg.FW_AGL;

    Q_EMIT motorflagUpdated();

}

void QNode::RtoC_B_Callback(const template_gui_package::battery& battery_msg){

    battery_data.HEADER_SEC = battery_msg.HEADER.stamp.sec;
    battery_data.HEADER_NSEC = battery_msg.HEADER.stamp.nsec;
    battery_data.NAME = battery_msg.NAME;
    battery_data.V = battery_msg.V;
    battery_data.A = battery_msg.A;
    battery_data.SOC = battery_msg.SOC;
    battery_data.TEMP = battery_msg.TEMP;

    Q_EMIT batteryflagUpdated();

    std::cout<<"battery calb"<<std::endl;
}

void QNode::RtoC_J_Callback(const template_gui_package::joystick& joystick_msg){
    RtoC_joystick_data.HEADER_SEC = joystick_msg.HEADER.stamp.sec;
    RtoC_joystick_data.HEADER_NSEC = joystick_msg.HEADER.stamp.nsec;
    RtoC_joystick_data.NAME = joystick_msg.NAME;
    RtoC_joystick_data.LS_X = joystick_msg.LS_X;
    RtoC_joystick_data.LS_Y = joystick_msg.LS_Y;
    RtoC_joystick_data.L_T = joystick_msg.L_T;
    RtoC_joystick_data.L1_BTN = joystick_msg.L1_BTN;
    RtoC_joystick_data.L2_BTN = joystick_msg.L2_BTN;
    RtoC_joystick_data.L3_BTN = joystick_msg.L3_BTN;
    RtoC_joystick_data.RS_X = joystick_msg.RS_X;
    RtoC_joystick_data.RS_Y = joystick_msg.RS_Y;
    RtoC_joystick_data.R_T = joystick_msg.R_T;
    RtoC_joystick_data.R1_BTN = joystick_msg.R1_BTN;
    RtoC_joystick_data.R2_BTN = joystick_msg.R2_BTN;
    RtoC_joystick_data.R3_BTN = joystick_msg.R3_BTN;
    RtoC_joystick_data.D_BTN_X = joystick_msg.D_BTN_X;
    RtoC_joystick_data.D_BTN_Y = joystick_msg.D_BTN_Y;
    RtoC_joystick_data.BTN0 = joystick_msg.BTN[0];
    RtoC_joystick_data.BTN1 = joystick_msg.BTN[1];
    RtoC_joystick_data.BTN2 = joystick_msg.BTN[2];
    RtoC_joystick_data.BTN3 = joystick_msg.BTN[3];
    RtoC_joystick_data.S_BTN = joystick_msg.S_BTN;
    RtoC_joystick_data.O_BTN = joystick_msg.O_BTN;
    RtoC_joystick_data.PS_BTN = joystick_msg.PS_BTN;
    RtoC_joystick_data.DMC_BTN = joystick_msg.DMC_BTN;
    RtoC_joystick_data.FMC_BTN = joystick_msg.FMC_BTN;

    Q_EMIT RtoC_joystickflagUpdated();
}

void QNode::CtoR_J_Callback(const sensor_msgs::Joy::ConstPtr& msg){
    CtoR_joystick_data.HEADER_SEC =  msg->header.stamp.sec;
    CtoR_joystick_data.HEADER_NSEC = msg->header.stamp.nsec;
    CtoR_joystick_data.NAME = msg->header.frame_id;
    CtoR_joystick_data.LS_X = msg->axes[0];
    CtoR_joystick_data.LS_Y = msg->axes[1];
    CtoR_joystick_data.L_T = msg->buttons[11];
    CtoR_joystick_data.L1_BTN = msg->buttons[4];
    CtoR_joystick_data.L2_BTN = msg->buttons[6];
    CtoR_joystick_data.L3_BTN = msg->buttons[11];
    CtoR_joystick_data.RS_X = msg->axes[3];
    CtoR_joystick_data.RS_Y = msg->axes[4];
    CtoR_joystick_data.R_T = msg->buttons[12];
    CtoR_joystick_data.R1_BTN = msg->buttons[5];
    CtoR_joystick_data.R2_BTN = msg->buttons[7];
    CtoR_joystick_data.R3_BTN = msg->buttons[12];
    CtoR_joystick_data.D_BTN_X = msg->axes[6]; //left +
    CtoR_joystick_data.D_BTN_Y = msg->axes[7]; //up +
    CtoR_joystick_data.BTN0 = msg->buttons[0]; // X
    CtoR_joystick_data.BTN1 = msg->buttons[1]; // O
    CtoR_joystick_data.BTN2 = msg->buttons[2]; // triangle
    CtoR_joystick_data.BTN3 = msg->buttons[3]; // square
    CtoR_joystick_data.S_BTN = msg->buttons[8];
    CtoR_joystick_data.O_BTN = msg->buttons[9];
    CtoR_joystick_data.PS_BTN = msg->buttons[10];
//    CtoR_joystick_data.DMC_BTN = msg->axes[0];
//    CtoR_joystick_data.FMC_BTN = msg->axes[0];

    Q_EMIT CtoR_joystickflagUpdated();
}



void QNode::RtoC_getgas_Callback(const template_gui_package::get_gas& getgas_msg){

    double buffer[32];
    for(int i=0; i<32; i++)
    {
      getgas_data.adc_data[i]=getgas_msg.ADC_DATA[i];
      //buffer[i]=getgas_msg.ADC_DATA[i];
    }
    Q_EMIT RtoC_getgasflagUpdated();
}

void QNode::RtoC_getcapture_Callback(const template_gui_package::get_capture& getcapture_msg){

    getcapture_data.GET_FLOW[0]=getcapture_msg.GET_FLOW[0];

    Q_EMIT RtoC_getcaptureflagUpdated();
}




void QNode::Camera_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  //std::cout<<"camera_callbacked"<<std::endl;
  QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);

  //temp = temp.rgbSwapped();
  QImage image = temp.copy();

  if(!image.isNull())   {
    Q_EMIT CameraUpdated(image);

    lastImgMsgTime = msg->header.stamp;
    //std::cout<<"["<<lastImgMsgTime<<"]"<<std::endl;

  }
//   Use image ...
}

void QNode::test_camera_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  //std::cout<<"camera_callbacked"<<std::endl;
  QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);

  //temp = temp.rgbSwapped();
  QImage image = temp.copy();

  if(!image.isNull())   {
    Q_EMIT testCameraUpdated(image);

    lastImgMsgTime = msg->header.stamp;
    //std::cout<<"["<<lastImgMsgTime<<"]"<<std::endl;

  }
//   Use image ...
} //test1-4

void QNode::Rviz_Top_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
  QImage image = temp.copy();

  if(!image.isNull())   {
    Q_EMIT Rviz_Top_Updated(image);

  }
//   Use image ...
}

void QNode::Rviz_Tpf_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
  QImage image = temp.copy();

  if(!image.isNull())   {
    Q_EMIT Rviz_Tpf_Updated(image);
  }
//   Use image ...
}

}  // namespace kmu_gui
