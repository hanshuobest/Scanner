#ifndef ADAPTERWIDGET_H
#define ADAPTERWIDGET_H

#include <QGLWidget>
#include <osg/ArgumentParser>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <QTimer>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QMdiSubWindow>
#include <QMdiArea>
#include <iostream>


#ifdef _DEBUG
#pragma comment(lib , "OpenThreadsd.lib")
#pragma comment(lib , "osgd.lib") 
#pragma comment(lib , "osgDBd.lib")
#pragma comment(lib , "osgUtild.lib")
#pragma comment(lib , "osgGAd.lib") 
#pragma comment(lib , "osgViewerd.lib")
#pragma comment(lib , "osgTextd.lib") 
#pragma comment(lib , "osgQtd.lib")
#pragma comment(lib , "osgManipulatord.lib")

#else
#pragma comment(lib , "OpenThreads.lib")
#pragma comment(lib , "osg.lib") 
#pragma comment(lib , "osgDB.lib")
#pragma comment(lib , "osgUtil.lib")
#pragma comment(lib , "osgGA.lib") 
#pragma comment(lib , "osgViewer.lib")
#pragma comment(lib , "osgText.lib") 
#pragma comment(lib , "osgQt.lib")
#pragma comment(lib , "osgManipulator.lib") 
#endif 


using Qt::WindowFlags ;
class QMouseEvent ;
class QWheelEvent ;
class AdapterWidget : public QGLWidget
{
	Q_OBJECT

public:
	AdapterWidget(QWidget *parent = 0 , const char *name = 0 , const QGLWidget *shareWidget = 0 , WindowFlags flag = 0) ;
	~AdapterWidget();

	osgViewer::GraphicsWindow *getGraphicsWindow()
	{
		return _gw.get() ;
	}
	const osgViewer::GraphicsWindow *getGraphicsWindow() const
	{
		return _gw.get() ;
	}

protected:
	void init() ;
	virtual void resizeGL(int w, int h) ;
	virtual void keyPressEvent(QKeyEvent *event) ;
	virtual void keyReleaseEvent(QKeyEvent *event) ;
	virtual void mousePressEvent(QMouseEvent *event) ;
	virtual void mouseReleaseEvent(QMouseEvent *event) ;
	virtual void mouseMoveEvent(QMouseEvent *event) ;
	virtual void wheelEvent(QWheelEvent *event) ;
	virtual void paintGL() ;
	
private:
	osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw ;
	
};

#endif // ADAPTERWIDGET_H
