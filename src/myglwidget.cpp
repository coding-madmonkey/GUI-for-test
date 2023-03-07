#include "../include/template_gui_package/myglwidget.h"
#include <QPainter>
#include <QPaintEvent>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>
MyGLWidget::MyGLWidget(QWidget* parent):QOpenGLWidget(parent)
{

}

const int init_h=500;
const int init_w=500;
void MyGLWidget::initializeGL()
{
  glClearColor(0.0, 0.0, 0.0, 1);
  glutInitWindowSize(init_w, init_h);
  //glEnable(GL_DEPTH_TEST);
  //glEnable(GL_LIGHT0);
  //glEnable(GL_LIGHTING);
  //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

}

void MyGLWidget::resizeGL(int new_w, int new_h)
{
        glViewport(0, 0, new_w, new_h);

        GLfloat widthFactor=(GLfloat)new_w/(GLfloat)init_w;
        GLfloat heightFactor=(GLfloat)new_h/(GLfloat)init_h;


        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double factor=0.5;
        glOrtho(-2.0*widthFactor, 2.0*widthFactor, -2.0*heightFactor, 2.0*heightFactor,-1.0, 1.0);
        //gluPerspective(45.0, float(w/h), 0.01, 100.0);
        //updateGL();
        //update();
}

void MyGLWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //glMatrixMode(GL_MODELVIEW);
  //glLoadIdentity();

  //glViewport(0, 0, width()/2, height()/2); //works!!
  //gluLookAt(0,0,5, 0,0,0, 0,1,0);

  glColor3f(1.0, 0.0, 0.0);
  glutSolidSphere(1,40,40);




  //glEnd();

  //QPainter p(this);
  //p.setPen(Qt::red);
  //p.drawLine(rect().topLeft(), rect().bottomRight());

}
