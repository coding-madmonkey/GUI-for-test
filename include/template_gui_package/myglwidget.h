#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>

class QPaintEvnet;

class MyGLWidget : public QOpenGLWidget
{
  Q_OBJECT
public:
  MyGLWidget(QWidget* parent =nullptr);

  //void paintEvent(QPaintEvent * event);


  // QOpenGLWidget interface
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
};

#endif // MYGLWIDGET_H
