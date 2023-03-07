#include "mainwindow.h"
#include "ui_mainwindow.h"


extern Getgas getgas_data;
extern Joystick CtoR_joystick_data;
extern Getcapture getcapture_data;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  qnode(argc,argv),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  nh_.reset(new ros::NodeHandle("~"));

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

  //std::cout<<"ros init?"<<std::endl;
  qnode.init();


  QObject::connect(&qnode, SIGNAL(mainWindowForceUpdate(const double)), this, SLOT(mainUIupdate(const double)));

  QObject::connect(&qnode, SIGNAL(CameraUpdated(const QImage &)),    this, SLOT(updateMainCameraView(const QImage &)));
  QObject::connect(&qnode, SIGNAL(Rviz_Top_Updated(const QImage &)),    this, SLOT(updateSub1CameraView(const QImage &)));
  QObject::connect(&qnode, SIGNAL(Rviz_Tpf_Updated(const QImage &)),    this, SLOT(updateSub2CameraView(const QImage &)));

  QObject::connect(&qnode, SIGNAL(CtoR_joystickflagUpdated()), this, SLOT(CtoR_updateJDataView()));
  QObject::connect(&qnode, SIGNAL(RtoC_getgasflagUpdated()),    this, SLOT(update32GasView()));

  QObject::connect(&qnode, SIGNAL(RtoC_getcaptureflagUpdated()),    this, SLOT(update100CaptureView()));



  image_redlight.load(":/rcs/Red.png");
  image_greenlight.load(":/rcs/Green.png");

  ui->sensor_light_00->setPixmap(image_redlight);
  ui->sensor_light_01->setPixmap(image_redlight);
  ui->sensor_light_02->setPixmap(image_redlight);
  ui->sensor_light_03->setPixmap(image_redlight);
  ui->sensor_light_04->setPixmap(image_redlight);
  ui->sensor_light_05->setPixmap(image_redlight);
  ui->sensor_light_06->setPixmap(image_redlight);
  ui->sensor_light_07->setPixmap(image_redlight);
  ui->sensor_light_08->setPixmap(image_redlight);
  ui->sensor_light_09->setPixmap(image_redlight);
  ui->sensor_light_10->setPixmap(image_redlight);
  ui->sensor_light_11->setPixmap(image_redlight);
  ui->sensor_light_12->setPixmap(image_redlight);
  ui->sensor_light_13->setPixmap(image_redlight);
  ui->sensor_light_14->setPixmap(image_redlight);
  ui->sensor_light_15->setPixmap(image_redlight);

  //ui->label->setPixmap(QPixmap(":/karim/test.png"));
  ui->joystick_base->setPixmap(QPixmap(":/rcs/alljoy.png").scaled(ui->joystick_base->width(),ui->joystick_base->height(),Qt::KeepAspectRatio, Qt::FastTransformation));

  ui->joy_btn_up->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_down->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_left->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_right->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_x->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_circle->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_x->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_rectangle->setAttribute(Qt::WA_NoSystemBackground);
  ui->joy_btn_triangle->setAttribute(Qt::WA_NoSystemBackground);

  swapNum=0;

  if (!image_redlight.load( ":/rcs/Red.png" )) {
      qWarning("Failed to load images/Red.png");
  }

  // publish a message on the channel specified by ~/hello_topic param
  std::string hello_topic;
  nh_->param<std::string>("hello_topic",hello_topic,"chatter");
  hello_pub_ = nh_->advertise<std_msgs::String>(hello_topic,1);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete ros_timer;
}
void MainWindow::mainUIupdate(const double rosTimeDiff_byImg){
    ui->lcdNumber->display(rosTimeDiff_byImg);
    //sstd::cout<<rosTimeDiff_byImg<<std::endl;
}

void MainWindow::updateMainCameraView(const QImage &msg) {

  if(swapNum==0)
    ui->main_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->main_camera->width(),ui->main_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
  else if (swapNum==1)
    ui->sub1_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->sub1_camera->width(),ui->sub1_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
  else
    ui->sub2_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->sub2_camera->width(),ui->sub2_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));

    //                QPixmap p = QPixmap::fromImage(msg);
}
void MainWindow::updateSub1CameraView(const QImage &msg) {
    //QPixmap p = QPixmap::fromImage(msg);

  if(swapNum==0)
    ui->sub1_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->sub1_camera->width(),ui->sub1_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
  else if (swapNum==1)
    ui->sub2_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->sub2_camera->width(),ui->sub2_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
  else
    ui->main_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->main_camera->width(),ui->main_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
}
void MainWindow::updateSub2CameraView(const QImage &msg) {

  if(swapNum==0)
    ui->sub2_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->sub2_camera->width(),ui->sub2_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
  else if (swapNum==1)
    ui->main_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->main_camera->width(),ui->main_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
  else
    ui->sub1_camera->setPixmap(QPixmap::fromImage(msg).scaled(ui->sub1_camera->width(),ui->sub1_camera->height(),Qt::KeepAspectRatio, Qt::FastTransformation));

    //                QPixmap p = QPixmap::fromImage(msg);
}

void MainWindow::update32GasView() {

  if(getgas_data.adc_data[0]<4.0)
    ui->sensor_light_00->setPixmap(image_greenlight);
  else
    ui->sensor_light_00->setPixmap(image_redlight);
  if(getgas_data.adc_data[1]<4.0)
    ui->sensor_light_01->setPixmap(image_greenlight);
  else
    ui->sensor_light_01->setPixmap(image_redlight);
  if(getgas_data.adc_data[2]<4.0)
    ui->sensor_light_02->setPixmap(image_greenlight);
  else
    ui->sensor_light_02->setPixmap(image_redlight);
  if(getgas_data.adc_data[3]<4.0)
    ui->sensor_light_03->setPixmap(image_greenlight);
  else
    ui->sensor_light_03->setPixmap(image_redlight);
  if(getgas_data.adc_data[4]<4.0)
    ui->sensor_light_04->setPixmap(image_greenlight);
  else
    ui->sensor_light_04->setPixmap(image_redlight);
  if(getgas_data.adc_data[5]<4.0)
    ui->sensor_light_05->setPixmap(image_greenlight);
  else
    ui->sensor_light_05->setPixmap(image_redlight);
  if(getgas_data.adc_data[6]<4.0)
    ui->sensor_light_06->setPixmap(image_greenlight);
  else
    ui->sensor_light_06->setPixmap(image_redlight);
  if(getgas_data.adc_data[7]<4.0)
    ui->sensor_light_07->setPixmap(image_greenlight);
  else
    ui->sensor_light_07->setPixmap(image_redlight);
  if(getgas_data.adc_data[8]<4.0)
    ui->sensor_light_08->setPixmap(image_greenlight);
  else
    ui->sensor_light_08->setPixmap(image_redlight);
  if(getgas_data.adc_data[9]<4.0)
    ui->sensor_light_09->setPixmap(image_greenlight);
  else
    ui->sensor_light_09->setPixmap(image_redlight);
  if(getgas_data.adc_data[10]<4.0)
    ui->sensor_light_10->setPixmap(image_greenlight);
  else
    ui->sensor_light_10->setPixmap(image_redlight);
  if(getgas_data.adc_data[11]<4.0)
    ui->sensor_light_11->setPixmap(image_greenlight);
  else
    ui->sensor_light_11->setPixmap(image_redlight);
  if(getgas_data.adc_data[12]<4.0)
    ui->sensor_light_12->setPixmap(image_greenlight);
  else
    ui->sensor_light_12->setPixmap(image_redlight);
  if(getgas_data.adc_data[13]<4.0)
    ui->sensor_light_13->setPixmap(image_greenlight);
  else
    ui->sensor_light_13->setPixmap(image_redlight);
  if(getgas_data.adc_data[14]<4.0)
    ui->sensor_light_14->setPixmap(image_greenlight);
  else
    ui->sensor_light_14->setPixmap(image_redlight);
  if(getgas_data.adc_data[15]<4.0)
    ui->sensor_light_15->setPixmap(image_greenlight);
  else
    ui->sensor_light_15->setPixmap(image_redlight);


}



void MainWindow::update100CaptureView() {

  double tempdata = (double)getcapture_data.GET_FLOW[0];
  std::cout<<"get100"<<std::endl;
  ui->capture100_0->display(tempdata);


}





void MainWindow::CtoR_updateJDataView() {

    // Try Again

    if (CtoR_joystick_data.D_BTN_X>0)
    {
        ui->joy_btn_left->setPixmap(image_greenlight);
        ui->joy_btn_right->clear();
    }
    else if (CtoR_joystick_data.D_BTN_X<0)
    {
        ui->joy_btn_left->clear();
        ui->joy_btn_right->setPixmap(image_greenlight);
    }
    else
    {
        ui->joy_btn_left->clear();
        ui->joy_btn_right->clear();
    }

    if (CtoR_joystick_data.D_BTN_Y>0)
    {
        ui->joy_btn_up->setPixmap(image_greenlight);
        ui->joy_btn_down->clear();
    }
    else if (CtoR_joystick_data.D_BTN_Y<0)
    {
        ui->joy_btn_up->clear();
        ui->joy_btn_down->setPixmap(image_greenlight);
    }
    else
    {
        ui->joy_btn_up->clear();
        ui->joy_btn_down->clear();
    }


}



void MainWindow::spinOnce(){

  std_msgs::String msg;
  std::stringstream ss;
  ss << "JJ hello world "; // << ui->hi_num->value();
  msg.data = ss.str();
  if(ros::ok()){
    hello_pub_.publish(msg);
    //.scaled(ui->sensor_light1->width(),ui->sensor_light1->height(),Qt::KeepAspectRatio, Qt::FastTransformation));
    //ui->sensor_light1->setPixmap(":/images/Red.png");
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

void MainWindow::chatterCallback(const std_msgs::String::ConstPtr &msg){
  auto qstring_msg = QString::fromStdString( msg->data.c_str() );

  //ui->chatter->setText(qstring_msg);
}







//ui->sensor_light_11->setPixmap(image_greenlight);














/*
void MainWindow::on_hi_button_clicked()
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "hello world ";// << ui->hi_num->value();
  msg.data = ss.str();

  MainWindow.publish(msg);

  //ui->hi_num->setValue(ui->hi_num->value()+1);
}
*/

void MainWindow::on_imageSwapButton_clicked()
{
    swapNum++;
    if(swapNum==3)
      swapNum=0;
}
