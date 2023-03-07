#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include "qnode.h"
#include <QOpenGLWindow>
#include <QSurfaceFormat>
#include <QOpenGLFunctions>
#include <QtOpenGL/QtOpenGL>
#include <GL/glu.h>

#include <QApplication>
#include <QIcon>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
  ~MainWindow();
  void chatterCallback(const std_msgs::String::ConstPtr& msg);

public slots:
  void spinOnce();
  void mainUIupdate(const double rosTimeDiff_byImg);
  void updateMainCameraView(const QImage &);
  void updateSub1CameraView(const QImage &);
  void updateSub2CameraView(const QImage &);
  void update32GasView();
  void update100CaptureView();
  void CtoR_updateJDataView();
  void updatetestCameraView(const QImage &); //test1-8
  void testswapimage(); //test2-6
private slots:
  //void on_hi_button_clicked();

  void on_imageSwapButton_clicked();

  void on_imagechangebutton_clicked();

private:
  Ui::MainWindow *ui;
  //Ui::mainwindow *ui;
  QTimer *ros_timer;
  template_gui_package::QNode qnode;

  ros::NodeHandlePtr nh_;
  ros::Subscriber chatter_sub_;
  ros::Publisher  hello_pub_;

  QOpenGLContext *context;
  QOpenGLFunctions *openGLFunctions;
  QPixmap image_greenlight;
  QPixmap image_redlight;

  QPixmap image_hedgehog;
  QPixmap image_deer;
  QPixmap image_rabbit; //2-5
  int swapNum;
  int testchangenum; //test2-3
};

#endif // MAINWINDOW_H
