#pragma once
#ifdef _DEBUG
#pragma comment(lib , "OpenThreadsd.lib")
#pragma comment(lib , "osgd.lib") 
#pragma comment(lib , "osgDBd.lib")
#pragma comment(lib , "osgUtild.lib")
#pragma comment(lib , "osgGAd.lib") 
#pragma comment(lib , "osgViewerd.lib")
#pragma comment(lib , "osgTextd.lib") 
#pragma comment(lib , "osgQtd.lib")

#else
#pragma comment(lib , "OpenThreads.lib")
#pragma comment(lib , "osg.lib") 
#pragma comment(lib , "osgDB.lib")
#pragma comment(lib , "osgUtil.lib")
#pragma comment(lib , "osgGA.lib") 
#pragma comment(lib , "osgViewer.lib")
#pragma comment(lib , "osgText.lib") 
#pragma comment(lib , "osgQt.lib")

#endif 


#include <QtCore/QTimer>
#include <QResizeEvent>

#include <osgViewer/View>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgQt/GraphicsWindowQt>

class OsgViewrWidget :
	public osgQt::GLWidget, public osgViewer::Viewer
{
	Q_OBJECT

public:

	OsgViewrWidget(QWidget *parent = 0);
	~OsgViewrWidget(void);

protected:
	 void paintEvent( QPaintEvent* event );
	 void resizeEvent( QResizeEvent* event );

private slots:
	void openFile();


protected:
	//QTimer _timer;
};

