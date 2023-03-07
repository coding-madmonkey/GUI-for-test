/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef template_gui_package_QNODE_HPP_
#define template_gui_package_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QImage>
#include <QLabel>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include "template_gui_package/motor.h"
#include "template_gui_package/battery.h"
#include "template_gui_package/command.h"
#include "template_gui_package/joystick.h"
#include "template_gui_package/get_gas.h"

#include "template_gui_package/set_gas.h"
#include "template_gui_package/get_capture.h"
#include "template_gui_package/set_capture.h"




struct Motor{
    int HEADER_SEC;
    int HEADER_NSEC;
    std::string NAME;

    std::string CM;
    std::string RP;

    bool LMC_S;
    bool RMC_S;
    bool FMC_S;

    int LM_SPD;
    float LW_SPD;

    int RM_SPD;
    float RW_SPD;

    int FM_SPD;
    float FW_SPD;

    int FM_AGL;
    float FW_AGL;
};

struct Battery{
    int HEADER_SEC;
    int HEADER_NSEC;
    std::string NAME;

    float V;
    float A;

    unsigned int SOC;
    unsigned int TEMP;
};

struct Joystick{
    int HEADER_SEC;
    int HEADER_NSEC;
    std::string NAME;

    double LS_X;
    double LS_Y;
    int L_T;

    bool L1_BTN;
    bool L2_BTN;
    bool L3_BTN;

    double RS_X;
    double RS_Y;
    int R_T;

    bool R1_BTN;
    bool R2_BTN;
    bool R3_BTN;

    double D_BTN_X;
    double D_BTN_Y;

    bool BTN0, BTN1, BTN2, BTN3;

    bool S_BTN;
    bool O_BTN;
    bool PS_BTN;

    bool DMC_BTN;
    bool FMC_BTN;
};

struct Movie{
    int cam_width;
    int cam_height;

    int rviz_width;
    int rviz_height;
};


struct Getgas{

    int HEADER_SEC;
    int HEADER_NSEC;

    std::string NAME;
    std::string NODE_ID;
    std::string DATE;
    std::string TIME;

    double adc_data[32];
};

struct Getcapture{

  int HEADER_SEC;
  int HEADER_NSEC;

  uint8_t GET_STATE;

  int16_t N_SAVED_DATA;
  int16_t I_SAVED_DATA;

  int16_t GET_LINE[100];
  //std::string GET_MSMT_TIME[100];
  int16_t GET_FLOW[100];
  int16_t GET_CAP_TIME[100];
  int16_t GET_INTEG_FLOW[100];

};

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace template_gui_package {

  /*****************************************************************************
  ** Class
  *****************************************************************************/

    class QNode : public QThread {
      Q_OBJECT
    public:
//      QNode();
      QNode(int argc, char** argv );
      virtual ~QNode();
      bool init();
      //bool init(const std::string &master_url, const std::string &host_url);
      void run();
      void Camera_Callback(const sensor_msgs::ImageConstPtr &msg);
      void Rviz_Top_Callback(const sensor_msgs::ImageConstPtr &msg);
      void Rviz_Tpf_Callback(const sensor_msgs::ImageConstPtr &msg);
      void RtoC_M_Callback(const template_gui_package::motor& motor_msg);
      void RtoC_B_Callback(const template_gui_package::battery& battery_msg);
      void RtoC_J_Callback(const template_gui_package::joystick& joystick_msg);
      void CtoR_J_Callback(const sensor_msgs::Joy::ConstPtr& msg);

      void RtoC_getgas_Callback(const template_gui_package::get_gas& getgas_msg);
      void RtoC_getcapture_Callback(const template_gui_package::get_capture& getcapture_msg);
      void test_camera_Callback(const sensor_msgs::ImageConstPtr &msg);//test1-3
      /*********************
      ** Logging
      **********************/
      enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
      };
      QImage view_image;
      QStringListModel* loggingModel() { return &logging_model; }
      void log( const LogLevel &level, const std::string &msg);

      Q_SIGNALS:
      void loggingUpdated();

      void motorflagUpdated();
      void batteryflagUpdated();
      void RtoC_joystickflagUpdated();
      void CtoR_joystickflagUpdated();

      void RtoC_getgasflagUpdated();

      void RtoC_getcaptureflagUpdated();


      void rosShutdown();
      void CameraUpdated(const QImage &);
      void Rviz_Top_Updated(const QImage &);
      void Rviz_Tpf_Updated(const QImage &);

      void mainWindowForceUpdate(const double);

      void testCameraUpdated(const QImage &); //test1-5
    private:
      int init_argc;
      char** init_argv;
      ros::Publisher rosmode_publisher;

      ros::Subscriber S_Camera_movie;
      ros::Subscriber S_top_movie;
      ros::Subscriber S_tpf_movie;
      ros::Subscriber S_Motor_data;
      ros::Subscriber S_Battery_data;
      ros::Subscriber S_RtoC_Joystick_data;
      ros::Subscriber S_CtoR_Joystick_data;

      ros::Subscriber S_RtoC_getgas_data;
      ros::Subscriber test_camera; //test1-1
      ros::Time lastImgMsgTime;

        QStringListModel logging_model;
    };



  }  // namespace kmu_gui

#endif /* kmu_gui_QNODE_HPP_ */
