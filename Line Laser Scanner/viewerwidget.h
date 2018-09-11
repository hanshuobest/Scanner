#ifndef VIEWERWIDGET_H
#define VIEWERWIDGET_H

#include <QWidget>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgQt/GraphicsWindowQt>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <QTimer>
#include <QPaintEvent>

class ViewerWidget : public QWidget , public osgViewer::Viewer
{
	Q_OBJECT

public:
	ViewerWidget(osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::Viewer::SingleThreaded):QWidget()
	{
		setThreadingModel(threadingModel) ;
//		connect(&_timer , SIGNAL(timeout()) , this , SLOT(update())) ;
		
	}
	~ViewerWidget();

	QWidget* addViewWidget( osgQt::GraphicsWindowQt* gw, osg::Node* scene )
	{
		osgViewer::View* view = new osgViewer::View;
//		addView( view );//向场景中添加一个Viewer

		osg::Camera* camera = view->getCamera();
		camera->setGraphicsContext( gw );//设置上下文

		const osg::GraphicsContext::Traits* traits = gw->getTraits();

		camera->setClearColor( osg::Vec4(0.2, 0.2, 0.6, 1.0) );//设置清除缓冲区背景的颜色
		camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
		//正交方式设置正交投影矩阵
		camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );

		view->setSceneData( scene );
		view->addEventHandler( new osgViewer::StatsHandler );
		view->setCameraManipulator( new osgGA::TrackballManipulator );

		return gw->getGLWidget();
	}

	osgQt::GraphicsWindowQt* createGraphicsWindow( int x, int y, int w, int h, const std::string& name="", bool windowDecoration=false )
	{
		osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();//负责显示设置，包括立体显示
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;//窗口特性
		traits->windowName = name;
		traits->windowDecoration = windowDecoration;
		traits->x = x;
		traits->y = y;
		traits->width = w;
		traits->height = h;
		traits->doubleBuffer = true;
		traits->alpha = ds->getMinimumNumAlphaBits();
		traits->stencil = ds->getMinimumNumStencilBits();
		traits->sampleBuffers = ds->getMultiSamples();
		traits->samples = ds->getNumMultiSamples();

		return new osgQt::GraphicsWindowQt(traits.get());
	}
	virtual void paintEvent(QPaintEvent * event)
	{
		frame() ;
	}

	void loadFile(QString path) ;

protected:
	QTimer _timer ;
private:
	osg::ref_ptr<osg::Node> m_node ;
};

#endif // VIEWERWIDGET_H
