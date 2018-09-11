#ifndef VIEWERQT_H
#define VIEWERQT_H

#include "adapterwidget.h"
#include <QTimer>
#include <osgGA/StateSetManipulator>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <osgQt/GraphicsWindowQt>

class QMouseEvent ;
class QWheelEvent ;
class ViewerQT : public AdapterWidget , public osgViewer::Viewer
{
	Q_OBJECT

public:
	ViewerQT(QWidget *parent = 0 , const char *name = 0 , const QGLWidget *shareWidget = 0 , WindowFlags flag = 0)
		:AdapterWidget(parent , name , shareWidget , flag),m_isUntitled(false),m_curFile(""),m_node(NULL),m_source(new pcl::PointCloud<pcl::PointXYZRGB>),m_root(new osg::Group()),m_suffix(""),m_root2(new osg::Group()),m_filtercloud(new pcl::PointCloud<pcl::PointXYZRGB>),m_geode(new osg::Geode)
	{
		this->setAttribute(Qt::WA_DeleteOnClose) ;

		getCamera()->setViewport(new osg::Viewport(0 , 0 , width() , height())) ;
		getCamera()->setProjectionMatrixAsPerspective(30.0 , static_cast<double>(width())/static_cast<double>(height()) , 1.0 , 10000) ;
		getCamera()->setGraphicsContext(getGraphicsWindow()) ;
		setThreadingModel(osgViewer::Viewer::SingleThreaded) ;
		setCameraManipulator(new osgGA::TrackballManipulator) ;
		connect(&_timer , SIGNAL(timeout()) , this , SLOT(updateGL())) ;
		_timer.start(10) ;
	}
	~ViewerQT();

	void newFile() ;
	bool loadFile(const QString &fileName) ;
	bool loadNode(osg::Node *node) ;
	void setCurrentFile(const QString &fileName) ;
	QString currentFile()const ;
	//设置隐藏模型
	void showornot(int flag) ;
	void print(QString str) ;
	osg::ref_ptr<osg::Node> getNode() const ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getcloud() const ;

	//显示ply格式点云
	void showPlycloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) ;
protected:
	void paintGL() ;
	void loadpclpointcloud(QString path) ;
	void wheelEvent(QWheelEvent *event) ;
	//创建GraphicsWindow显示场景
	osgQt::GraphicsWindowQt *createGraphicsWindow(int x , int y , int w , int h , const std::string &name = "" , bool windowDecoration = false) ;
signals:
	void sendMessage(QString str) ;
private:
	QTimer _timer ;
	bool m_isUntitled ;
	QString m_curFile ;
	std::string m_suffix ;//文件的后缀名

	osg::ref_ptr<osg::Node> m_node ;
	osg::ref_ptr<osg::Geode> m_geode ;
	osg::ref_ptr<osg::Group> m_root ;
	osg::ref_ptr<osg::Group> m_root2 ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_source ;//原始点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_filtercloud ;//滤波后的点云

};

#endif // VIEWERQT_H
 