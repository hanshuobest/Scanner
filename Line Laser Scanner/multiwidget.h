#ifndef MULTIWIDGET_H
#define MULTIWIDGET_H

#include <QWidget>
#include <QTimer>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/ViewerEventHandlers>


class MultiWidget : public QWidget,public osgViewer::CompositeViewer
{
	Q_OBJECT

public:
	MultiWidget(osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::CompositeViewer::SingleThreaded , osg::Node *node1 = NULL , osg::Node *node2 = NULL) ;
	~MultiWidget();
protected:
	virtual void paintEvent(QPaintEvent *event) ;
private:
	osgQt::GraphicsWindowQt *createGraphicsWindow(int x , int y , int w , int h, const std::string &name = "" , bool windowDecoration = false) ;
	QWidget *addViewWidget(osgQt::GraphicsWindowQt *gw , osg::Node *scene) ;
	QWidget *m_widget1 ;
	QWidget *m_widget2 ;
	osg::ref_ptr<osg::Node> m_node1 ;
	osg::ref_ptr<osg::Node> m_node2 ;
	QTimer m_timer ;
};

#endif // MULTIWIDGET_H
