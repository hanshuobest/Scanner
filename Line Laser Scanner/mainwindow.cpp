#include "mainwindow.h"
#include <QMenu>
#include <QAction>
#include <QIcon>
#include <QCloseEvent>
#include "OsgViewrWidget.h"
#include "capturevideo.h"
#include "captureimagedlg.h"
#include "singlecalibrator.h"
#include "sterocalibrator.h"
#include "generatepointclouddlg.h"
#include "staticsoutlierremovaldlg.h"
#include "radiusoutlierremovaldlg.h"
#include "aboutdlg.h"
#include "aligndlg.h"
#include <QTime>
#include <QListWidgetItem>
#include <QMdiSubWindow>
#include "OsgViewrWidget.h"
#include <QSettings>
#include <QPalette>
#include <QSignalMapper>
#include <QFileDialog>
#include <QFileInfo>
#include <QMdiArea>
#include <QList>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QListWidgetItem>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <osg/MatrixTransform>
#include "multiwidget.h"
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/TranslateAxisDragger>
#include <osgManipulator/ScaleAxisDragger>
#include "FilterFunction.h"
#include "RegisterFunction.h"
#include <osgUtil/Optimizer>
#include <osg/Group>
#include "VertexVisitor.h"
#include <multiwidget.h>
#include "voxelgridsampledlg.h"
#include "alignresultdlg.h"
#include <QDockWidget>
#include "mydockwiget.h"
#include <time.h>
#include "ndtparamdlg.h"
#include "randomsample.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "possiondlg.h"
#include <QProcess>
#include <osg/DrawPixels>
#include "jianyingdlg.h"
#include "videotoframe.h"
#include "imagefilterdlg.h"


int mainWindow::m_i = 0 ;
mainWindow::mainWindow(QWidget *parent)
	: QMainWindow(parent),m_single(NULL),m_stero(NULL),m_widget(NULL),m_colorcloud(new ColorCloud),m_showflag(true),m_filtercloud(new ColorCloud),m_sourcecloud(new Cloud),m_targetcloud(new Cloud),m_swroot(new osg::Switch()),m_mulwidget(NULL),m_swfilter(new osg::Switch()),m_sampleSource(new pcl::PointCloud<pcl::PointXYZ>),m_sampleflag(false)
{
	ui.setupUi(this);
	m_dockwidget = new QDockWidget ;
	m_dockwidget->setAllowedAreas(Qt::RightDockWidgetArea|Qt::TopDockWidgetArea) ;
	addDockWidget(Qt::RightDockWidgetArea , m_dockwidget) ;
	m_dockwidget->hide() ;


	m_dockwidget2 = new QDockWidget ;
	m_dockwidget2->setAllowedAreas(Qt::RightDockWidgetArea|Qt::BottomDockWidgetArea) ;
	addDockWidget(Qt::RightDockWidgetArea , m_dockwidget2) ;
	m_dockwidget2->hide() ;

	ui.dockWidget->hide() ;
 	showMaximized() ;
 	createActions() ;
 	createMenus() ;
	createTools() ;
	updateMenus() ;

	setUnifiedTitleAndToolBarOnMac(true) ;
}

mainWindow::~mainWindow()
{

}
void mainWindow::createActions()
{
	m_newAct = new QAction(tr("&New") , this) ;
	m_newAct->setIcon(QIcon("Resources\\NewFile.png")) ;
	m_newAct->setStatusTip(QStringLiteral("新建")) ;
	connect(m_newAct , SIGNAL(triggered()) , this , SLOT(slot_new())) ;

	m_openAct = new QAction(tr("&Open") , this) ;
	m_openAct->setIcon(QIcon("Resources\\openFile.png")) ;
	m_openAct->setStatusTip(QStringLiteral("打开")) ;
	connect(m_openAct , SIGNAL(triggered()) , this , SLOT(slot_open())) ;

	m_saveAct = new QAction(tr("&Save") , this) ;
	m_saveAct->setIcon(QIcon("Resources\\save.png")) ;
	m_saveAct->setEnabled(false) ;
	m_saveAct->setStatusTip(QStringLiteral("保存")) ;
	connect(m_saveAct , SIGNAL(triggered()) , this , SLOT(slot_save())) ;

	m_saveasAct = new QAction(tr("&TSave") , this) ;
	m_saveasAct->setIcon(QIcon("Resources\\saveas.png")) ;
	m_saveasAct->setEnabled(false) ;
	m_saveasAct->setStatusTip(QStringLiteral("另存为")) ;
	connect(m_saveasAct , SIGNAL(triggered()) , this , SLOT(slot_saveas())) ;

	m_exitAct = new QAction(tr("&Exit") , this) ;
	m_exitAct->setIcon(QIcon("Resources\\Exit.png")) ;
	m_exitAct->setStatusTip(QStringLiteral("退出")) ;
	connect(m_exitAct , SIGNAL(triggered()) , qApp , SLOT(closeAllWindows())) ;

	//编辑菜单下的动作
	m_clearAll = new QAction(tr("Clear") , this) ;
	m_clearAll->setIcon(QIcon("Resources\\clear.jpg")) ;
	m_clearAll->setStatusTip(QStringLiteral("清理控制台内容")) ;
	connect(m_clearAll , SIGNAL(triggered()) , this , SLOT(slot_clearAll())) ;
	
	m_voxelgridSampleAct = new QAction(tr("Voxelgrid") , this) ;

	m_voxelgridSampleAct->setStatusTip(QStringLiteral("VoxelGrid滤波器对点云下采样")) ;
	m_voxelgridSampleAct->setEnabled(false) ;
	connect(m_voxelgridSampleAct , SIGNAL(triggered()) , this , SLOT(slot_voxelgridSample())) ;

	m_randomSampleAct = new QAction(tr("RandomSample") , this) ;
	m_randomSampleAct->setEnabled(false) ;
	m_randomSampleAct->setStatusTip(QStringLiteral("均匀下采样")) ;
	connect(m_randomSampleAct , SIGNAL(triggered()) , this , SLOT(slot_randomSample())) ;

	//视频图像抓取动作
	m_captureImageAct = new QAction(tr("Capture Image") , this) ;
	m_captureImageAct->setStatusTip(QStringLiteral("抓取图片")) ;
	connect(m_captureImageAct , SIGNAL(triggered()) , this , SLOT(slot_caputureImage())) ;
	m_capturevideoAct = new QAction(tr("Capture Video") , this) ;
	m_capturevideoAct->setStatusTip(QStringLiteral("抓取视频")) ;
	connect(m_capturevideoAct , SIGNAL(triggered()) , this , SLOT(slot_captureVideo())) ;
	m_jiezhen = new QAction(QStringLiteral("video decoder") , this) ;
	m_jiezhen->setStatusTip(QStringLiteral("video decoder")) ;
	connect(m_jiezhen , SIGNAL(triggered()) , this , SLOT(slot_jiezhen())) ;
	m_jianyingAct = new QAction(tr("Digital reduction shadow method ") , this) ;
	m_jianyingAct->setStatusTip(QStringLiteral("数字减影法实现激光条分割")) ;
	connect(m_jianyingAct , SIGNAL(triggered()) , this , SLOT(slot_jianying())) ;
	m_gaussfilterAct = new QAction(tr("Gauss filter") , this) ;
	m_gaussfilterAct->setStatusTip(QStringLiteral("高斯滤波")) ;
	connect(m_gaussfilterAct , SIGNAL(triggered()) , this , SLOT(slot_gaussfilter())) ;

	m_singlecalibAct = new QAction(tr("Single camera calibrate") , this) ;
	m_singlecalibAct->setStatusTip(QStringLiteral("单目相机标定")) ;
	m_singlecalibAct->setIcon(QIcon("Resources\\camera.png")) ;
	connect(m_singlecalibAct , SIGNAL(triggered()) , this , SLOT(slot_singlecalib())) ;
	m_sterocalibAct = new QAction(tr("Stero camera calibrate") , this) ;
	m_sterocalibAct->setIcon(QIcon("Resources\\stereocamera.jpg")) ;
	m_sterocalibAct->setStatusTip(QStringLiteral("双目相机标定")) ;
	connect(m_sterocalibAct , SIGNAL(triggered()) , this , SLOT(slot_sterocalib())) ;

	m_generatepointcloudAct = new QAction(tr("Generate point Cloud") , this) ;
	m_generatepointcloudAct->setStatusTip(QStringLiteral("生成点云数据")) ;
	connect(m_generatepointcloudAct , SIGNAL(triggered()) , this , SLOT(slot_generatepointcloud())) ;
	m_statisticalOutlierRemovalAct = new QAction(tr("StatisticalOutlierRemoval") , this) ;
	m_statisticalOutlierRemovalAct->setIcon(QIcon("Resources\\ccSORFilter.png")) ;
	m_statisticalOutlierRemovalAct->setEnabled(false) ;
	m_statisticalOutlierRemovalAct->setStatusTip(QStringLiteral("StatisticalOutlierRemoval滤波器")) ;
	connect(m_statisticalOutlierRemovalAct , SIGNAL(triggered()) , this , SLOT(slot_statisticalOutlierRemoval())) ;
	m_radiusOutlierRemovalAct = new QAction(tr("RadiusOutlierRemoval") , this) ;
	m_radiusOutlierRemovalAct->setIcon(QIcon("Resources\\radiusoutlierremoval.png")) ;
	m_radiusOutlierRemovalAct->setEnabled(false) ;
	m_radiusOutlierRemovalAct->setStatusTip(QStringLiteral("RadiusOutlierRemoval滤波器")) ;
	connect(m_radiusOutlierRemovalAct , SIGNAL(triggered()) , this , SLOT(slot_radiusOutlierRemoval())) ;

	m_loadtwoPointcloudAct = new QAction(tr("Load two point cloud") , this) ;
	m_loadtwoPointcloudAct->setIcon(QIcon("Resources\\steroload.png")) ;
	m_loadtwoPointcloudAct->setStatusTip(QStringLiteral("加载待配准的源点云和目标点云")) ;
	connect(m_loadtwoPointcloudAct , SIGNAL(triggered()) , this , SLOT(slot_loadtwoPointcloud())) ;

	m_cruderegisterAct = new QAction(tr("based Image register") , this) ;
	m_cruderegisterAct->setIcon(QIcon("Resources\\basedImage.png")) ;
	m_cruderegisterAct->setStatusTip(QStringLiteral("基于影像配准")) ;
	m_cruderegisterAct->setEnabled(false) ;
	connect(m_cruderegisterAct , SIGNAL(triggered()) , this , SLOT(slot_cruderegister())) ;
	m_highregisterAct = new QAction(tr("based point cloud register") , this) ;
	m_highregisterAct->setEnabled(false) ;
	m_highregisterAct->setStatusTip(QStringLiteral("基于点云配准（ICP）")) ;
	m_highregisterAct->setIcon(QIcon("Resources\\ccAlign.png")) ;
	connect(m_highregisterAct , SIGNAL(triggered()) , this , SLOT(slot_highregister())) ;
	m_NDTAct = new QAction(tr("NDT register") , this) ;
	m_NDTAct->setEnabled(false) ;
	m_NDTAct->setStatusTip(QStringLiteral("NDT配准")) ;
	m_NDTAct->setIcon(QIcon("Resources\\NDT.png")) ;
	connect(m_NDTAct , SIGNAL(triggered()) , this , SLOT(slot_NDT())) ;
	m_possionAct = new QAction(tr("Possion reconstruct") , this) ;
	connect(m_possionAct , SIGNAL(triggered()) , this , SLOT(slot_possion())) ;

	m_newViewerAct = new QAction(tr("New Viewer") , this) ;
	m_newViewerAct->setStatusTip(QStringLiteral("新建3D视图器")) ;
	connect(m_newViewerAct , SIGNAL(triggered()) , this , SLOT(slot_newViewer())) ;
	m_zoominAct = new QAction(tr("Zoom in") , this) ;
	m_zoominAct->setIcon(QIcon("Resources\\zoomIn.png")) ;
	m_zoominAct->setEnabled(false) ;
	m_zoominAct->setStatusTip(QStringLiteral("放大")) ;
	connect(m_zoominAct , SIGNAL(triggered()) , this , SLOT(slot_zoomin())) ;
	m_zoomoutAct = new QAction(tr("Zoom out") , this) ;
	m_zoomoutAct->setIcon(QIcon("Resources\\Zoomout.png")) ;
	m_zoomoutAct->setEnabled(false) ;
	m_zoomoutAct->setStatusTip(QStringLiteral("缩小")) ;
	connect(m_zoomoutAct , SIGNAL(triggered()) , this , SLOT(slot_zoomout())) ;

	m_simulationAct = new QAction(tr("Simulation") , this) ;
	connect(m_simulationAct , SIGNAL(triggered()) , this , SLOT(slot_simulation())) ;

	m_helpAct = new QAction(tr("Help") , this) ;
	m_helpAct->setIcon(QIcon("Resources\\help.png")) ;
	m_helpAct->setStatusTip(QStringLiteral("帮助")) ;
	connect(m_helpAct , SIGNAL(triggered()) , this , SLOT(slot_help())) ;
	m_aboutAct = new QAction(tr("About") , this) ;
	m_aboutAct->setIcon(QIcon("Resources\\about.png")) ;
	m_aboutAct->setStatusTip(QStringLiteral("关于")) ;
	connect(m_aboutAct , SIGNAL(triggered()) , this , SLOT(slot_about())) ;

	//tree
	m_treemodel = new QStandardItemModel(ui.treeView) ;
	connect(ui.treeView , SIGNAL(clicked(const QModelIndex)) , this , SLOT(slot_treeclick(const QModelIndex))) ;
	connect(m_treemodel , SIGNAL(itemChanged(QStandardItem *)) , this , SLOT(treeItemChanged(QStandardItem *))) ;
	ui.treeView->setModel(m_treemodel) ;

	connect(this , SIGNAL(sendMessage(QString)) , this , SLOT(slot_receiveMessage(QString))) ;
}

void mainWindow::createMenus()
{
	m_filemenu = menuBar()->addMenu(tr("File")) ;
	m_filemenu->addAction(m_newAct) ;
	m_filemenu->addAction(m_openAct) ;
	m_filemenu->addAction(m_saveAct) ;
	m_filemenu->addAction(m_saveasAct) ;
	m_filemenu->addAction(m_exitAct) ;

	m_editmenu = menuBar()->addMenu(tr("Edit")) ;
	m_editmenu->addAction(m_clearAll) ;
	m_sample = m_editmenu->addMenu(tr("Sample")) ;
	m_sample->addAction(m_voxelgridSampleAct) ;
	m_sample->addAction(m_randomSampleAct) ;

	m_videomenu = menuBar()->addMenu(tr("Capture")) ;
	m_videomenu->addAction(m_captureImageAct) ;
	m_videomenu->addAction(m_capturevideoAct) ;
	m_videomenu->addAction(m_jiezhen) ;
	m_segmentMenu = m_videomenu->addMenu(tr("Segment")) ;
	m_segmentMenu->addAction(m_jianyingAct) ;
// 	m_filemenu = m_videomenu->addMenu(tr("Image Filter")) ;
// 	m_filemenu->addAction(m_gaussfilterAct) ;
	m_videomenu->addAction(m_gaussfilterAct) ;

	m_calibmenu = menuBar()->addMenu(tr("Camera calibrate")) ;
	m_calibmenu->addAction(m_singlecalibAct) ;
	m_calibmenu->addAction(m_sterocalibAct) ;

	m_pointcloudmenu = menuBar()->addMenu("Point cloud process") ;
	m_pointcloudmenu->addAction(m_generatepointcloudAct) ;

 	m_filemenu = m_pointcloudmenu->addMenu(tr("Filter")) ;
 	m_filemenu->addAction(m_statisticalOutlierRemovalAct) ;
	m_filemenu->addAction(m_radiusOutlierRemovalAct) ;

	m_registermenu = m_pointcloudmenu->addMenu(tr("Register")) ;
	m_registermenu->addAction(m_loadtwoPointcloudAct) ;
	m_registermenu->addSeparator() ;
	m_registermenu->addAction(m_cruderegisterAct) ;
	m_registermenu->addAction(m_highregisterAct) ;
	m_registermenu->addAction(m_NDTAct) ;

	m_reconstructmenu = m_pointcloudmenu->addMenu(tr("Reconstruct")) ;
	m_reconstructmenu->addAction(m_possionAct) ;

	m_threeviewermenu = menuBar()->addMenu(tr("3D Viewer")) ;
	m_threeviewermenu->addAction(m_newViewerAct) ;
	m_threeviewermenu->addSeparator() ;
	m_threeviewermenu->addAction(m_zoominAct) ;
	m_threeviewermenu->addAction(m_zoomoutAct) ;
	m_threeviewermenu->addAction(m_simulationAct) ;

	m_helpmenu = menuBar()->addMenu(tr("Help")) ;
	m_helpmenu->addAction(m_helpAct) ;
	m_helpmenu->addAction(m_aboutAct) ;
}

void mainWindow::createTools()
{
	ui.mainToolBar->addAction(m_newAct) ;
	ui.mainToolBar->addAction(m_openAct) ;
	ui.mainToolBar->addAction(m_saveAct) ;
	ui.mainToolBar->addAction(m_exitAct) ;
	ui.mainToolBar->addSeparator() ;

	ui.mainToolBar->addAction(m_clearAll) ;

	ui.mainToolBar->addSeparator() ;
	ui.mainToolBar->addAction(m_singlecalibAct) ;
	ui.mainToolBar->addAction(m_sterocalibAct) ;

	ui.mainToolBar->addSeparator();
	ui.mainToolBar->addAction(m_statisticalOutlierRemovalAct) ;
	ui.mainToolBar->addAction(m_radiusOutlierRemovalAct) ;

	ui.mainToolBar->addSeparator() ;
	ui.mainToolBar->addAction(m_loadtwoPointcloudAct) ;
	ui.mainToolBar->addAction(m_cruderegisterAct) ;
	ui.mainToolBar->addAction(m_highregisterAct) ;
	ui.mainToolBar->addAction(m_NDTAct) ;

	ui.mainToolBar->addSeparator() ;
	ui.mainToolBar->addAction(m_zoominAct) ;
	ui.mainToolBar->addAction(m_zoomoutAct) ;
}

void mainWindow::slot_new()
{
	mainWindow *mw = new mainWindow ;
	mw->show() ;
}

void mainWindow::slot_open()
{
	QString fileName = QFileDialog::getOpenFileName(this , QStringLiteral("打开") , QString() , QStringLiteral("点云文件 (*.pcd *.ply) ;;三维模型(*.osg*)")) ;
	if (!fileName.isEmpty())
	{
		
		QFileInfo info(fileName) ;
		QString filename = info.fileName() ;
		QString filedir = info.filePath() ;

		if (m_treemodel->rowCount(QModelIndex()) >= 1)
		{
			m_treemodel->clear() ;
		}
		QStandardItem *root = new QStandardItem(QIcon("Resources\\pcl.png") , filedir) ;
		root->setCheckable(true) ;
		root->setTristate(true) ;
		root->setCheckState(Qt::Checked) ;
		m_treemodel->appendRow(root) ;

		QStandardItem *child = new QStandardItem(filename) ;
		child->setCheckable(true) ;
		child->setCheckState(Qt::Checked) ;
		root->appendRow(child) ;

		QString suffix = info.suffix() ;
		if (suffix == "osg")
		{
			osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(fileName.toStdString()) ;
			int num = m_swroot->getNumChildren() ;
			if (num > 0)
			{
				m_swroot->removeChildren(0 , num) ;
			}
			m_swroot->addChild(node) ;

			VertexVisitor vtea ;
			node->accept(vtea) ;
			int numofpoint = vtea.extracted_verts->size() ;
			float radius = node->getBound().radius() ;
			float x = node->getBound().center().x() ;
			float y = node->getBound().center().y() ;
			float z = node->getBound().center().z() ;
			MyDockWiget *mydock = new MyDockWiget(this , numofpoint , radius , x , y , z) ;
			m_dockwidget->setWidget(mydock) ;
			m_dockwidget->setWindowTitle(QStringLiteral("模型属性")) ;
			m_dockwidget->show() ;
		}
		else if (suffix == "ply")
		{
			if (pcl::io::loadPLYFile(fileName.toStdString() , *m_colorcloud) == -1)
			{
				QString str = "load point cloud failture!" ;
				print(str) ;
				return ;
			}

			m_saveasAct->setEnabled(true) ;
			osg::ref_ptr<osg::Group> root = new osg::Group() ;
			root = convertPly(m_colorcloud) ;
			int num = m_swroot->getNumChildren() ;
			if (num > 0)
			{
				m_swroot->removeChildren(0 , num) ;
			}
			int numofpoint = m_colorcloud->size() ;
			float radius = root->getBound().radius() ;
			float x = root->getBound().center().x() ;
			float y = root->getBound().center().y() ;
			float z = root->getBound().center().z() ;
			MyDockWiget *mydock = new MyDockWiget(this , numofpoint , radius , x , y , z) ;
			m_dockwidget->setWidget(mydock) ;
			m_dockwidget->setWindowTitle(QStringLiteral("点云属性")) ;
			if (m_dockwidget->isHidden())
			{
				m_dockwidget->show() ;
			}
			m_swroot->addChild(root.get()) ;

			m_statisticalOutlierRemovalAct->setEnabled(true) ;
			m_radiusOutlierRemovalAct->setEnabled(true) ;
		}

		if (m_dockwidget2->isVisible())
		{
			m_dockwidget2->show() ;
		}

		osgUtil::Optimizer optimizer ;
		optimizer.optimize(m_swroot.get()) ;
		MultiWidget *mw = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , m_swroot.get()) ;
		setCentralWidget(mw) ;
		print(QString(tr("load model finished!"))) ;
		statusBar()->showMessage(tr("Point cloud has load") , 2000) ;
	}
}

void mainWindow::slot_save()
{

}

void mainWindow::slot_saveas()
{
	PointcloudTypeDlg *typedlg = new PointcloudTypeDlg ;
	if (!typedlg->exec())
	{
		return ;
	}
	QFileInfo info(typedlg->getpath()) ;

	if (info.suffix() == "ply")
	{
		pcl::io::savePLYFileASCII(typedlg->getpath().toStdString() , *m_colorcloud) ;
	}
// 	else if(info.suffix() == "pcd")
// 	{
// 		pcl::io::savePCDFile(typedlg->getpath().toStdString() , *m_colorcloud) ;
// 	}
// 	print(tr("save point cloud finished!")) ;
}

void mainWindow::slot_singlecalib()
{
	m_single = new SingleCalibrator ;
	connect(m_single , SIGNAL(sendMessage(const QString)) , this ,  SLOT(slot_receiveMessage(const QString))) ;
	m_single->show() ;
}

void mainWindow::slot_sterocalib()
{
	m_stero = new SteroCalibrator ;
	connect(m_stero , SIGNAL(sendMessage(const QString)) , this , SLOT(slot_receiveMessage(const QString))) ;
	m_stero->show() ;
}

void mainWindow::slot_generatepointcloud()
{
	m_generatecloud = new GeneratePointCloudDlg ;
	connect(m_generatecloud , SIGNAL(sendMessage(const QString)) , this , SLOT(slot_receiveMessage(const QString))) ;
	connect(m_generatecloud , SIGNAL(sendPointcloudInfo(QVariant &)) , this , SLOT(slot_receivePointcloudInfo(QVariant &)) , Qt::DirectConnection) ;
	if (m_generatecloud->exec())
	{
		m_generatecloud->raise() ;
		m_generatecloud->activateWindow() ;
	}
}

void mainWindow::slot_receivePointcloudInfo(QVariant &var)
{
	if (m_treemodel->rowCount(QModelIndex()) >= 1)
	{
		m_treemodel->clear() ;
	}

	PointCloudInfo info =  var.value<PointCloudInfo>() ;
	int nums = info.x_ivec.size() ;

	m_colorcloud->width = nums ;
	m_colorcloud->height = 1 ;
	m_colorcloud->is_dense = false ;
	m_colorcloud->resize(nums * 1) ;
	float r(0.0) , g(0.0) , b(0.0) ;
	for (int i = 0 ; i < nums ; ++i)
	{
		m_colorcloud->points[i].x = info.x_ivec[i] ;
		m_colorcloud->points[i].y = info.y_ivec[i] ;
		m_colorcloud->points[i].z = info.z_ivec[i] ;
		m_colorcloud->points[i].r = info.r_ivec[i] ;
		m_colorcloud->points[i].g = info.g_ivec[i] ;
		m_colorcloud->points[i].b = info.b_ivec[i] ;
	}

	osg::ref_ptr<osg::Group> root = new osg::Group() ;
	root = convertPly(m_colorcloud) ;

	float radius = root->getBound().radius() ;
	float x = root->getBound().center().x() ;
	float y = root->getBound().center().y() ;
	float z = root->getBound().center().z() ;
	MyDockWiget *mydock = new MyDockWiget(this , nums , radius , x , y , z) ;
	m_dockwidget->setWidget(mydock) ;
	m_dockwidget->setWindowTitle(QStringLiteral("生成点云属性")) ;
	if (m_dockwidget->isHidden())
	{
		m_dockwidget->show() ;
	}

	if (m_dockwidget2->isVisible())
	{
		m_dockwidget2->hide() ;
	}
	osgUtil::Optimizer optimizer ;
	optimizer.optimize(root.get()) ;

	QStandardItem *itemroot = new QStandardItem(QIcon("Resources\\pcl.png") , tr("point cloud")) ;
	itemroot->setCheckable(true) ;
	itemroot->setTristate(true) ;
	itemroot->setCheckState(Qt::Checked) ;
	m_treemodel->appendRow(itemroot) ;

	QStandardItem *itemchild = new QStandardItem(tr("point cloud")) ;
	itemchild->setCheckable(true) ;
	itemchild->setTristate(true) ;
	itemchild->setCheckState(Qt::Checked) ;
	itemroot->appendRow(itemchild) ;

	if (m_swroot->getNumChildren() > 0)
	{
		m_swroot->removeChildren(0 , 1) ;
	}


	m_swroot->addChild(root.get()) ;
	if (m_mulwidget)
	{
		delete m_mulwidget ;
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded ,  m_swroot.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	else
	{
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded ,  m_swroot.get()) ;
		setCentralWidget(m_mulwidget) ;
	}

	m_statisticalOutlierRemovalAct->setEnabled(true) ;
	m_radiusOutlierRemovalAct->setEnabled(true) ;
}

int mainWindow::m_statiscalFilterKnn = 6 ;
double mainWindow::m_statiscalNSigma = 1.0 ;
void mainWindow::slot_statisticalOutlierRemoval()
{
	StaticsOutlierRemovalDlg *sd = new StaticsOutlierRemovalDlg ;
	sd->setKnn(m_statiscalFilterKnn) ;
	sd->setNsigma(m_statiscalNSigma) ;
	sd->show() ;
	if (!sd->exec())
	{
		return ;
	}
	m_statiscalFilterKnn = sd->getKnn() ;
	m_statiscalNSigma = sd->getNsigma() ;
	m_filtercloud = statisticalOutlierRemoval(m_colorcloud , m_statiscalFilterKnn , m_statiscalNSigma) ;

	QStandardItem *root = new QStandardItem(QIcon("Resources\\pcl.png") , tr("filter point cloud")) ;
	root->setCheckable(true) ;
	root->setTristate(true) ;
	root->setCheckState(Qt::Checked) ;
	m_treemodel->appendRow(root) ;

	QStandardItem *child = new QStandardItem(tr("cloud")) ;
	child->setCheckable(true) ;
	child->setCheckState(Qt::Checked) ;
	root->appendRow(child) ;

	osg::ref_ptr<osg::Group> filterroot = new osg::Group() ;
	filterroot = convertPly(m_filtercloud) ;
	m_swfilter->removeChildren(0 , m_swfilter->getNumChildren()) ;
	m_swfilter->addChild(filterroot) ;

	if (!m_mulwidget)
	{
		delete m_mulwidget ;
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , m_swroot.get() , m_swfilter.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	else
	{
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , m_swroot.get() , m_swfilter.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	ui.dockWidget_props->show() ;
	
	float x = filterroot->getBound().center().x() ;
	float y = filterroot->getBound().center().y() ;
	float z =filterroot->getBound().center().z() ;
	MyDockWiget *mydock = new MyDockWiget(this , m_filtercloud->size() , filterroot->getBound().radius() , x , y , z) ;
	m_dockwidget2->setWidget(mydock) ;
	m_dockwidget2->setWindowTitle(QStringLiteral("滤波后点云")) ;
	m_dockwidget2->show() ;
	return ;
}

int mainWindow::m_radiusKnn = 6 ;
double mainWindow::m_radius = 0.5 ;
void mainWindow::slot_radiusOutlierRemoval()
{
	RadiusOutlierRemovalDlg *rd = new RadiusOutlierRemovalDlg ;
	rd->setKnn(m_radiusKnn) ;
	rd->setradius(m_radius) ;
	if (!rd->exec())
	{
		return ;
	}
	m_radius = rd->getradius() ;
	m_radiusKnn = rd->getKnn() ;
	if (m_colorcloud->size() == 0)
	{
		return ;
	}
	m_filtercloud = radiusOutlierRemoval(m_colorcloud , m_radiusKnn , m_radius) ;

	QStandardItem *root = new QStandardItem(QIcon("Resources\\pcl.png") , tr("filter point cloud")) ;
	root->setCheckable(true) ;
	root->setTristate(true) ;
	root->setCheckState(Qt::Checked) ;
	m_treemodel->appendRow(root) ;

	QStandardItem *child = new QStandardItem(tr("cloud")) ;
	child->setCheckable(true) ;
	child->setCheckState(Qt::Checked) ;
	root->appendRow(child) ;

	osg::ref_ptr<osg::Group> filterroot = new osg::Group() ;
	filterroot = convertPly(m_filtercloud) ;
	m_swfilter->removeChildren(0 , m_swfilter->getNumChildren()) ;
	m_swfilter->addChild(filterroot) ;

	if (!m_mulwidget)
	{
		delete m_mulwidget ;
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , m_swroot.get() , m_swfilter.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	else
	{
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , m_swroot.get() , m_swfilter.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	ui.dockWidget_props->show() ;

	float x = filterroot->getBound().center().x() ;
	float y = filterroot->getBound().center().y() ;
	float z =filterroot->getBound().center().z() ;
	MyDockWiget *mydock = new MyDockWiget(this , m_filtercloud->size() , filterroot->getBound().radius() , x , y , z) ;
	m_dockwidget2->setWidget(mydock) ;
	m_dockwidget2->setWindowTitle(QStringLiteral("滤波后点云")) ;
	m_dockwidget2->show() ;
	return ;
}

void mainWindow::slot_cruderegister()
{

};

void mainWindow::slot_highregister()
{
	ui.statusBar->showMessage(QStringLiteral("配准中...")) ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>) ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>) ;
	if(m_sampleflag)
	{
		source = m_sampleSource ;
		target = m_sampleTarget ;
	}
	else
	{
		source = m_sourcecloud ;
		target = m_targetcloud ;
	}

	double start_time(0) , end_time(0) ;
	start_time = clock() ;
	AlignResult *ar = new AlignResult ;
	ar = registerFunction::ICP<pcl::PointXYZ>(source , target) ;
	end_time = (clock() - start_time)/1000 ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>) ;
	pcl::transformPointCloud(*m_sourcecloud , *result_cloud , ar->finalmatrix) ;

	osg::ref_ptr<osg::Switch> sw1 = new osg::Switch ;
	osg::ref_ptr<osg::Group> root1 = new osg::Group() ;
	osg::ref_ptr<osg::Group> root2 = new osg::Group() ;

	root1 = convertPly(m_sourcecloud , 1.0 , 0.0 , 0.0) ;
	root2 = convertPly(m_targetcloud , 0.0 , 1.0 , 0.0) ;
	sw1->addChild(root1) ;
	sw1->addChild(root2) ;
 
	osg::ref_ptr<osg::Switch> sw2 = new osg::Switch ;
	osg::ref_ptr<osg::Group> root3 = new osg::Group() ;
	root3 = convertPly(result_cloud , 1.0 , 0.0 , 0.0) ;
	sw2->addChild(root2) ;
	sw2->addChild(root3) ;

	if (m_mulwidget)
	{
		delete m_mulwidget ;
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , sw1.get() , sw2.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	else
	{
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , sw1.get() , sw2.get()) ;
		setCentralWidget(m_mulwidget) ;
	}

	AlignResultDlg *ard = new AlignResultDlg(this , ar->finalmatrix , ar->score , end_time) ;
	ard->show() ;
}

void mainWindow::slot_newViewer()
{
	MultiWidget *mw = new MultiWidget(osgViewer::CompositeViewer::SingleThreaded , osgDB::readNodeFile("cow.osg") , osgDB::readNodeFile("cow.osg")) ;
	mw->setGeometry(100 , 100 , 800 , 400) ;
	mw->show() ;
}

void mainWindow::slot_zoomin()
{

}

void mainWindow::slot_zoomout()
{

}

void mainWindow::slot_trackball()
{

}

void mainWindow::slot_help()
{

}

void mainWindow::slot_about()
{
	AboutDlg *ad = new AboutDlg ;
	ad->exec() ;
}

void mainWindow::closeEvent(QCloseEvent *event)
{
	event->accept() ;

}

void mainWindow::dragEnterEvent(QDragEnterEvent *event)
{

}

void mainWindow::dropEvent(QDropEvent *event)
{

}

void mainWindow::wheelEvent(QWheelEvent *event)
{	

}

void mainWindow::slot_caputureImage()
{
	CaptureImageDlg *ci = new CaptureImageDlg() ;
	ci->exec() ;	
}

void mainWindow::slot_captureVideo()
{
	CaptureVideo *cv = new CaptureVideo ;
	cv->exec() ;
}

void mainWindow::slot_receiveMessage(const QString &str)
{
	m_message = str ;
	QListWidgetItem *item = new QListWidgetItem(m_message) ;
	ui.listWidget->insertItem(m_i , item) ;
	m_i++ ;

}

void mainWindow::writeSetting()
{
	QSettings setting("Software Inc" , "LineLaserScanner") ;

}

void mainWindow::readSetting()
{
	QSettings setting("Software Inc" , "LineLaserScanner") ;

}

ViewerQT *mainWindow::slot_createNewWidget()
{
	ViewerQT *widget = new ViewerQT ;
	setCentralWidget(widget) ;
	return widget ;
}


void mainWindow::updateMenus()
{

}

void mainWindow::updateWindowMenu()
{

}

void mainWindow::initTree()
{
	m_treemodel = new QStandardItemModel(ui.treeView) ;
	m_treemodel->setHorizontalHeaderLabels(QStringList()<<QStringLiteral("点云列表")) ;


	QStandardItem *root = new QStandardItem(QIcon("Resources\\pcl.png") , QStringLiteral("点云1")) ;
	root->setCheckable(true) ;
	root->setTristate(true) ;
	m_treemodel->appendRow(root) ;
	for (int i = 0 ; i < 3 ; ++i)
	{
		QStandardItem *item = new QStandardItem(QStringLiteral("点云%1").arg(i + 1)) ;
		item->setCheckable(true) ;
		root->appendRow(item) ;
	}

	connect(m_treemodel , SIGNAL(itemChanged(QStandardItem *)) , this , SLOT(treeItemChanged(QStandardItem *))) ;
	ui.treeView->setModel(m_treemodel) ;

//	createTree() ;
}

void mainWindow::createTree()
{

}

void mainWindow::treeItemChanged(QStandardItem *item)
{
	if(item == nullptr)
		return;
	if(item->isCheckable())
	{
		//如果条目是存在复选框的，那么就进行下面的操作
		Qt::CheckState state = item->checkState();//获取当前的选择状态
		if(item->isTristate())
		{
			//如果条目是三态的，说明可以对子目录进行全选和全不选的设置
			if(state != Qt::PartiallyChecked)
			{
				//当前是选中状态，需要对其子项目进行全选
				treeItem_checkAllChild(item,state == Qt::Checked ? true : false);
			}
		}
		else
		{
			//说明是两态的，两态会对父级的三态有影响
			//判断兄弟节点的情况
			treeItem_CheckChildChanged(item);
		}
	}
}

void mainWindow::treeItem_checkAllChild(QStandardItem *item , bool check)
{
	if(item == nullptr)
		return;
	int rowCount = item->rowCount();
	for(int i=0;i<rowCount;++i)
	{
		QStandardItem* childItems = item->child(i);
		treeItem_checkAllChild_recursion(childItems,check);
	}
	if(item->isCheckable())
		item->setCheckState(check ? Qt::Checked : Qt::Unchecked);
}

void mainWindow::treeItem_checkAllChild_recursion(QStandardItem *item , bool check /* = true */)
{
	if(item == nullptr)
		return;
	int rowCount = item->rowCount();
	for(int i=0;i<rowCount;++i)
	{
		QStandardItem* childItems = item->child(i);
		treeItem_checkAllChild_recursion(childItems,check);
	}
	if(item->isCheckable())
		item->setCheckState(check ? Qt::Checked : Qt::Unchecked);
}

void mainWindow::treeItem_CheckChildChanged(QStandardItem *item)
{
	if(nullptr == item)
		return;
	Qt::CheckState siblingState = checkSibling(item);
	QStandardItem * parentItem = item->parent();
	if(nullptr == parentItem)
		return;
	if(Qt::PartiallyChecked == siblingState)
	{
		if(parentItem->isCheckable() && parentItem->isTristate())
			parentItem->setCheckState(Qt::PartiallyChecked);
	}
	else if(Qt::Checked == siblingState)
	{
		if(parentItem->isCheckable())
			parentItem->setCheckState(Qt::Checked);
	}
	else
	{
		if(parentItem->isCheckable())
			parentItem->setCheckState(Qt::Unchecked);
	}
	treeItem_CheckChildChanged(parentItem);
}

Qt::CheckState mainWindow::checkSibling(QStandardItem *item)
{
	//先通过父节点获取兄弟节点
	QStandardItem * parent = item->parent();
	if(nullptr == parent)
		return item->checkState();
	int brotherCount = parent->rowCount();
	int checkedCount(0),unCheckedCount(0);
	Qt::CheckState state;
	for(int i=0;i<brotherCount;++i)
	{
		QStandardItem* siblingItem = parent->child(i);
		state = siblingItem->checkState();
		if(Qt::PartiallyChecked == state)
			return Qt::PartiallyChecked;
		else if(Qt::Unchecked == state)
			++unCheckedCount;
		else
			++checkedCount;
		if(checkedCount>0 && unCheckedCount>0)
			return Qt::PartiallyChecked;
	}
	if(unCheckedCount>0)
		return Qt::Unchecked;
	return Qt::Checked;
}

void mainWindow::slot_treeclick(const QModelIndex &index)
{
// 	if (index.isValid())
// 	{
// 		//获取复选框所在的索引
// 		QModelIndex checkIndex = m_treemodel->index(index.row() , 0) ;
// 		//获取复选框选中状态值
// 		bool bChecked = m_treemodel->data(checkIndex , Qt::CheckStateRole).toBool() ;
// 		if (index.row() %2)
// 		{
// 			if (bChecked)
// 			{
// 				m_swfilter->setValue(0 , true) ;
// 				showProplist(m_swfilter.get()) ;
// 			}
// 			else
// 			{
// 				m_swfilter->setValue(0 , false) ;
// 			}
// 		}
// 		else
// 		{
// 			if (bChecked)
// 			{
// 				m_swroot->setValue(0 , true) ;
// 				showProplist(m_swroot.get()) ;
// 			}
// 			else
// 			{
// 				m_swroot->setValue(0 , false) ;
// 			}
// 		}
// 	}
	if (index.parent().isValid())
	{
		QModelIndex parentIndex = index.parent() ;
		QModelIndex checkIndex = m_treemodel->index(index.row() , 0 , parentIndex) ;
		bool bChecked = m_treemodel->data(checkIndex , Qt::CheckStateRole).toBool() ;
		if (index.row() %2)
		if (bChecked)
		{
			m_swroot->setValue(index.row()%2 , true) ;
		}
		else
		{
			m_swroot->setValue(index.row()%2 , false) ;
		}
	}
	else
	{
		m_swroot->setAllChildrenOff() ;
	}
}

void mainWindow::slot_clearAll()
{
	int counter = ui.listWidget->count() ;
	for (int index = 0 ; index < counter ; ++index)
	{
		QListWidgetItem *item = ui.listWidget->takeItem(0) ;
		delete item ;
	}
}


ColorCloudPtr mainWindow::statisticalOutlierRemoval(ColorCloudPtr inputcloud , int knn , double thresh)
{
	ColorCloudPtr outputcloud(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor ;//创建滤波对象
	sor.setInputCloud(inputcloud) ;
	sor.setMeanK(knn) ;
	sor.setStddevMulThresh(thresh) ; //设置判断是否为离群点的阈值
	sor.filter(*outputcloud) ;

	return outputcloud ;
}


ColorCloudPtr mainWindow::radiusOutlierRemoval(ColorCloudPtr inputcloud , int knn , double radius)
{
	ColorCloudPtr outputcloud(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem ;//创建滤波器
	outrem.setInputCloud(inputcloud) ;
	outrem.setRadiusSearch(radius) ;
	outrem.setMinNeighborsInRadius(knn) ;
	outrem.filter(*outputcloud) ;
	return outputcloud ;
}

osg::ref_ptr<osg::Group> mainWindow::convertPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	osg::ref_ptr<osg::Group> root = new osg::Group() ;

	osg::ref_ptr<osg::Vec3Array> coord = new osg::Vec3Array() ;
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array() ;
	int nums = cloud->size() ;
	float r(0.0) , g(0.0) , b(0.0) ;
	for (int i = 0 ; i < nums ; ++i)
	{
		coord->push_back(osg::Vec3(cloud->points[i].x , cloud->points[i].y , cloud->points[i].z)) ;
		r = (float)(cloud->points[i].r)/255 ;
		g = (float)(cloud->points[i].g)/255 ;
		b = (float)(cloud->points[i].b)/255 ;
		color->push_back(osg::Vec4(r , g , b , 1.0)) ;
	}

	//创建几何体
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry() ;
	//设置顶点数组
	geom->setVertexArray(coord.get()) ;
	geom->setColorArray(color.get()) ;
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX) ;

	osg::Vec3Array *normals = new osg::Vec3Array ;
	normals->push_back(osg::Vec3(0.0f , 1.0f , 0.0f)) ;
	geom->setNormalArray(normals) ;
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL) ;
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS , 0 , nums)) ;

	osg::ref_ptr<osg::Geode> geode = new osg::Geode() ;
	geode->addDrawable(geom.get()) ;

	root->addChild(geode.get()) ;
	root->getOrCreateStateSet()->setMode(GL_NORMALIZE , osg::StateAttribute::ON) ;

 	osg::ref_ptr<osg::MatrixTransform> selection = new osg::MatrixTransform ;
 	selection->addChild(root) ;
 
 	osgManipulator::TrackballDragger *dragger = new osgManipulator::TrackballDragger ;
 	dragger->setupDefaultGeometry() ;
 	float scale = root->getBound().radius() * 1.0 ;
 	dragger->setMatrix(osg::Matrix::scale(scale , scale , scale) * osg::Matrix::translate(root->getBound().center())) ;
 	dragger->addTransformUpdating(selection) ;
 	dragger->setHandleEvents(true) ;
 	dragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL) ;
 
 	osgManipulator::TranslateAxisDragger *tad = new osgManipulator::TranslateAxisDragger ;
 	tad->setupDefaultGeometry() ;
 	tad->setMatrix(osg::Matrix::scale(scale , scale , scale) * osg::Matrix::translate(root->getBound().center())) ;
 	tad->setHandleEvents(true) ;
 	tad->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL) ;
 
 	osgManipulator::ScaleAxisDragger *sad = new osgManipulator::ScaleAxisDragger ;
 	sad->setupDefaultGeometry() ;
 	sad->setMatrix(osg::Matrix::scale(scale , scale , scale) * osg::Matrix::translate(root->getBound().center())) ;
 	sad->addTransformUpdating(selection) ;
 	sad->setHandleEvents(true) ;
 	sad->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL) ;

 	osg::ref_ptr<osg::Group> root1 = new osg::Group() ;
 	root1->addChild(selection) ;
 	root1->addChild(dragger) ;
 	root1->addChild(tad) ;
 	root1->addChild(sad) ;
	root1->getOrCreateStateSet()->setMode(GL_LIGHTING , osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE) ;
//	root->getOrCreateStateSet()->setMode(GL_LIGHTING , osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE) ;

	return root1 ;
}

osg::ref_ptr<osg::Group> mainWindow::convertPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , float r /* = 0.0  */, float g /* = 0.0  */, float b /* = 0.0 */)
{
	osg::ref_ptr<osg::Vec3Array> coord = new osg::Vec3Array() ;
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array() ;
	int nums = cloud->size() ;
	for (int i = 0 ; i < nums ; ++i)
	{
		coord->push_back(osg::Vec3(cloud->points[i].x , cloud->points[i].y , cloud->points[i].z)) ;
		color->push_back(osg::Vec4(r , g , b , 1.0)) ;
	}
	//创建几何体
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry() ;
	geom->setVertexArray(coord.get()) ;
	geom->setColorArray(color.get()) ;
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX) ;

	osg::Vec3Array *normals = new osg::Vec3Array ;
	normals->push_back(osg::Vec3(0.0f , 1.0f , 0.0f)) ;
	geom->setNormalArray(normals) ;
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL) ;
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS , 0 , nums)) ;

	osg::ref_ptr<osg::Geode> geode = new osg::Geode() ;
	geode->addDrawable(geom.get()) ;

	osg::ref_ptr<osg::Group> root = new osg::Group() ;
	root->addChild(geode.get()) ;
	root->getOrCreateStateSet()->setMode(GL_NORMALIZE , osg::StateAttribute::ON) ;
	root->getOrCreateStateSet()->setMode(GL_LIGHTING , osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE) ;

	return root ;
}

void mainWindow::print(QString str)
{
	QTime mytime = QTime::currentTime() ;
	QString str1 = QString("[ %1: %2: %3]:").arg(QString::number(mytime.hour())).arg(QString::number(mytime.minute())).arg(QString::number(mytime.second())) ;

	emit sendMessage(str1 + " " + str) ;
}

osg::ref_ptr<osg::Group> mainWindow::addTrackball(osg::Node *node)
{
	//创建场景组节点
	osg::ref_ptr<osg::Group> root = new osg::Group();

	osg::ref_ptr<osg::MatrixTransform> selection = new osg::MatrixTransform ;
	selection->addChild(node) ;

	//Dragger
	osg::ref_ptr<osgManipulator::TrackballDragger> dragger = new osgManipulator::TrackballDragger ;
	dragger->setupDefaultGeometry() ;

	//set Dragger
	float scale = node->getBound().radius() * 1.0 ;
	dragger->setMatrix(osg::Matrix::scale(scale , scale , scale) * osg::Matrix::translate(node->getBound().center())) ;
	dragger->addTransformUpdating(selection) ;
	dragger->setHandleEvents(true) ;
	dragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL) ;

	//tad
	osg::ref_ptr<osgManipulator::TranslateAxisDragger> tad = new osgManipulator::TranslateAxisDragger ;
	tad->setupDefaultGeometry() ;
	tad->setMatrix(osg::Matrix::scale(scale , scale , scale) * osg::Matrix::translate(node->getBound().center())) ;
	tad->addTransformUpdating(selection) ;
	tad->setHandleEvents(true) ;
	tad->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL) ;

	//sad
	osg::ref_ptr<osgManipulator::ScaleAxisDragger> sad = new osgManipulator::ScaleAxisDragger ;
	sad->setupDefaultGeometry() ;
	sad->setMatrix(osg::Matrix::scale(scale , scale , scale) * osg::Matrix::translate(node->getBound().center())) ;
	sad->addTransformUpdating(selection) ;
	sad->setHandleEvents(true) ;
	sad->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL) ;

	root->addChild(selection) ;
	root->addChild(dragger) ;
	root->addChild(tad) ;
	root->addChild(sad) ;

	return root.release() ;
}

inline void mainWindow::showProplist(osg::Node *node)
{
	ui.scrollArea->show() ;
	VertexVisitor vtea ;
	node->accept(vtea) ;

	ui.numEdit->setText(tr("%1").arg(vtea.extracted_verts->size())) ;
	osg::BoundingSphere boudingsphere = node->getBound() ;
	ui.radiusEdit->setText(tr("%1").arg(boudingsphere.radius())) ;
	ui.cenEditX->setText(tr("%1").arg(boudingsphere.center().x())) ;
	ui.cenEditY->setText(tr("%1").arg(boudingsphere.center().y())) ;
	ui.cenEditZ->setText(tr("%1").arg(boudingsphere.center().z())) ;
}

void mainWindow::slot_randomSample()
{
	m_sampleflag = true ;
	RandomSample *rs = new RandomSample ;
	if (!rs->exec())
	{
		return ;
	}
	m_sampleSource = filterfunction::randomSample<pcl::PointXYZ>(m_sourcecloud , rs->getSample()) ;
	m_sampleTarget = filterfunction::randomSample<pcl::PointXYZ>(m_targetcloud , rs->getSample()) ;

	osg::ref_ptr<osg::Group> root1 = new osg::Group() ;
	root1 = convertPly(m_sampleSource , 1.0 , 0.0 , 0.0) ;
	osg::ref_ptr<osg::Group> root2 = new osg::Group() ;
	root2 = convertPly(m_sampleTarget , 0.0 , 1.0 , 0.0) ;

	osg::ref_ptr<osg::Switch> sw = new osg::Switch ;
	sw->addChild(root1 , true) ;
	sw->addChild(root2 , true) ;
	if (m_mulwidget)
	{
		delete m_mulwidget ;
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , sw.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	else
	{
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , sw.get()) ;
		setCentralWidget(m_mulwidget) ;
	}

	int num1 = m_sampleSource->size() ;
	float radius1 = root1->getBound().radius() ;
	float x1 = root1->getBound().center().x() ;
	float y1 = root1->getBound().center().y() ;
	float z1 = root1->getBound().center().z() ;
	MyDockWiget *mydock1 = new MyDockWiget(this , num1 , radius1 , x1 , y1 , z1) ;
	m_dockwidget->setWidget(mydock1) ;
	m_dockwidget->setWindowTitle(QStringLiteral("下采样源点云")) ;

	int num2 = m_sampleTarget->size() ;
	float radius2 = root2->getBound().radius() ;
	float x2 = root2->getBound().center().x() ;
	float y2 = root2->getBound().center().y() ;
	float z2 = root2->getBound().center().z() ;
	MyDockWiget *mydock2 = new MyDockWiget(this , num2 , radius2 , x2 , y2 , z2) ;
	m_dockwidget2->setWidget(mydock2) ;
	m_dockwidget2->setWindowTitle(QStringLiteral("下采样目标点云")) ;


	QStandardItem *itemroot = new QStandardItem(QIcon("Resources\\pcl.png") , tr("After filter point cloud")) ;
	itemroot->setCheckable(true) ;
	itemroot->setCheckState(Qt::Checked) ;

	QStandardItem *itemsource = new QStandardItem(QIcon("Resources\\red.jpg") , tr("source")) ;
	QStandardItem *itemtarget = new QStandardItem(QIcon("Resources\\green.jpg") , tr("target")) ;
	itemroot->appendRow(itemsource) ;
	itemroot->appendRow(itemtarget) ;
	itemsource->setCheckable(true) ;
	itemtarget->setCheckable(true) ;
	itemsource->setCheckState(Qt::Checked) ;
	itemtarget->setCheckState(Qt::Checked) ;

	m_treemodel->appendRow(itemroot) ;
	print(QString(QStringLiteral("下采样后源点云数量%1,目标点云数量%2")).arg(m_sampleSource->size()).arg(m_sampleTarget->size())) ;
}

void mainWindow::slot_voxelgridSample()
{
	VoxelgridSampleDlg *vs = new VoxelgridSampleDlg ;
	if (!vs->exec())
	{
		return ;
	}
	double leafsize = vs->getleafsize() ;
	m_sampleSource = filterfunction::downSample<pcl::PointXYZ>(m_sourcecloud , leafsize) ;
	m_sampleTarget = filterfunction::downSample<pcl::PointXYZ>(m_targetcloud , leafsize) ;

	osg::ref_ptr<osg::Group> root1 = new osg::Group() ;
	root1 = convertPly(m_sampleSource , 1.0 , 0.0 , 0.0) ;
	osg::ref_ptr<osg::Group> root2 = new osg::Group() ;
	root2 = convertPly(m_sampleTarget , 0.0 , 1.0 , 0.0) ;

	osg::ref_ptr<osg::Switch> sw = new osg::Switch ;
	sw->addChild(root1 , true) ;
	sw->addChild(root2 , true) ;
	if (m_mulwidget)
	{
		delete m_mulwidget ;
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , sw.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	else
	{
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , sw.get()) ;
		setCentralWidget(m_mulwidget) ;
	}

	int num1 = m_sampleSource->size() ;
	float radius1 = root1->getBound().radius() ;
	float x1 = root1->getBound().center().x() ;
	float y1 = root1->getBound().center().y() ;
	float z1 = root1->getBound().center().z() ;
	MyDockWiget *mydock1 = new MyDockWiget(this , num1 , radius1 , x1 , y1 , z1) ;
	m_dockwidget->setWidget(mydock1) ;
	m_dockwidget->setWindowTitle(QStringLiteral("下采样源点云")) ;
	if (m_dockwidget->isHidden())
	{
		m_dockwidget->show() ;
	}

	int num2 = m_sampleTarget->size() ;
	float radius2 = root2->getBound().radius() ;
	float x2 = root2->getBound().center().x() ;
	float y2 = root2->getBound().center().y() ;
	float z2 = root2->getBound().center().z() ;
	MyDockWiget *mydock2 = new MyDockWiget(this , num2 , radius2 , x2 , y2 , z2) ;
	m_dockwidget2->setWidget(mydock2) ;
	m_dockwidget2->setWindowTitle(QStringLiteral("下采样目标点云")) ;
	if (m_dockwidget2->isHidden())
	{
		m_dockwidget2->show() ;
	}

	
	QStandardItem *itemroot = new QStandardItem(QIcon("Resources\\pcl.png") , tr("After filter point cloud")) ;
	itemroot->setCheckable(true) ;
	itemroot->setCheckState(Qt::Checked) ;

	QStandardItem *itemsource = new QStandardItem(QIcon("Resources\\red.jpg") , tr("source")) ;
	QStandardItem *itemtarget = new QStandardItem(QIcon("Resources\\green.jpg") , tr("target")) ;
	itemroot->appendRow(itemsource) ;
	itemroot->appendRow(itemtarget) ;
	itemsource->setCheckable(true) ;
	itemtarget->setCheckable(true) ;
	itemsource->setCheckState(Qt::Checked) ;
	itemtarget->setCheckState(Qt::Checked) ;

	m_treemodel->appendRow(itemroot) ;
	print(QString(QStringLiteral("下采样后源点云数量%1,目标点云数量%2")).arg(m_sampleSource->size()).arg(m_sampleTarget->size())) ;
	
	m_sampleflag = true ;
}

void mainWindow::slot_loadtwoPointcloud()
{
	AlignDlg *ad = new AlignDlg ;
	ad->show() ;
	if (!ad->exec())
	{
		return ;
	}

	if (m_treemodel->rowCount(QModelIndex()) >= 1)
	{
		m_treemodel->clear() ;
	}

	QString sourcepath = ad->getsourcepath() ;
	QString targetpath = ad->gettargetpath() ;

	ui.statusBar->showMessage(QStringLiteral("加载点云中...")) ;
	m_sourcecloud = ad->loadPlyFile(sourcepath.toStdString()) ;
	m_targetcloud = ad->loadPlyFile(targetpath.toStdString()) ;
	ui.statusBar->showMessage(QStringLiteral("点云加载完成！") , 2000) ;

	print(QString(QStringLiteral("源点云数量：%1，目标点云数量：%2")).arg(m_sourcecloud->size()).arg(m_targetcloud->size())) ;
	osg::ref_ptr<osg::Group> root1 = new osg::Group() ;
	root1 = convertPly(m_sourcecloud , 1.0 , 0.0 , 0.0) ;
	osg::ref_ptr<osg::Group> root2 = new osg::Group() ;
	root2 = convertPly(m_targetcloud , 0.0 , 1.0 , 0.0) ;

	float num1 = m_sourcecloud->size() ;
	float radius1 = root1->getBound().radius() ;
	float x1 = root1->getBound().center().x() ;
	float y1 = root1->getBound().center().y() ;
	float z1 = root1->getBound().center().z() ;

	MyDockWiget *mydock1 = new MyDockWiget(this , num1 , radius1 , x1 , y1 , z1) ;
	m_dockwidget->setWidget(mydock1) ;
	m_dockwidget->setWindowTitle(QStringLiteral("源点云信息")) ;
	if (m_dockwidget->isHidden())
	{
		m_dockwidget2->show() ;
	}

	float num2 = m_targetcloud->size() ;
	float radius2 = root2->getBound().radius() ;
	float x2 = root2->getBound().center().x() ;
	float y2 = root2->getBound().center().y() ;
	float z2 = root2->getBound().center().z() ;

	MyDockWiget *mydock2 = new MyDockWiget(this , num2 , radius2 , x2 , y2 , z2) ;
	m_dockwidget2->setWidget(mydock2) ;
	m_dockwidget2->setWindowTitle(QStringLiteral("目标点云信息")) ;
	if (m_dockwidget2->isHidden())
	{
		m_dockwidget2->show() ;
	}

	m_swroot->removeChildren(0 , m_swroot->getNumChildren()) ;
	m_swroot->addChild(root1 , true) ;
	m_swroot->addChild(root2 , true) ;
	if (m_mulwidget)
	{
		delete m_mulwidget ;
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , m_swroot.get()) ;
		setCentralWidget(m_mulwidget) ;
	}
	else
	{
		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , m_swroot.get()) ;
		setCentralWidget(m_mulwidget) ;
	}

	QStandardItem *itemroot = new QStandardItem(QIcon("Resources\\pcl.png") , tr("Point cloud")) ;
	itemroot->setCheckable(true) ;
	itemroot->setCheckState(Qt::Checked) ;

	QStandardItem *itemsource = new QStandardItem(QIcon("Resources\\red.jpg") , tr("source")) ;
	QStandardItem *itemtarget = new QStandardItem(QIcon("Resources\\green.jpg") , tr("target")) ;
	itemroot->appendRow(itemsource) ;
	itemroot->appendRow(itemtarget) ;
	itemsource->setCheckable(true) ;
	itemtarget->setCheckable(true) ;
	itemsource->setCheckState(Qt::Checked) ;
	itemtarget->setCheckState(Qt::Checked) ;

	m_treemodel->appendRow(itemroot) ;
	ui.treeView->setModel(m_treemodel) ;

	m_voxelgridSampleAct->setEnabled(true) ;
	m_randomSampleAct->setEnabled(true) ;

	m_highregisterAct->setEnabled(true) ;
	m_NDTAct->setEnabled(true) ;
}

void mainWindow::slot_NDT()
{
	ui.statusBar->showMessage(QStringLiteral("配准中...")) ;
	NDTparamDlg *np = new NDTparamDlg ;
	if (!np->exec())
	{
		return ;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>) ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>) ;
	if(m_sampleflag)
	{
		source = m_sampleSource ;
		target = m_sampleTarget ;
	}
	else
	{
		source = m_sourcecloud ;
		target = m_targetcloud ;
	}

	double start_time(0) , end_time(0) ;
	start_time = clock() ;

	AlignResult *ar = registerFunction::NDT(source , target , np->getstep() , np->getresolution()) ;
	end_time = (clock() - start_time)/1000 ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>) ;
	pcl::transformPointCloud(*m_sourcecloud , *result_cloud , ar->finalmatrix) ;

	osg::ref_ptr<osg::Switch> sw1 = new osg::Switch ;
	osg::ref_ptr<osg::Group> root1 = new osg::Group() ;
	osg::ref_ptr<osg::Group> root2 = new osg::Group() ;

	root1 = convertPly(m_sourcecloud , 1.0 , 0.0 , 0.0) ;
	root2 = convertPly(m_targetcloud , 0.0 , 1.0 , 0.0) ;
	sw1->addChild(root1) ;
	sw1->addChild(root2) ;

	osg::ref_ptr<osg::Switch> sw2 = new osg::Switch ;
	osg::ref_ptr<osg::Group> root3 = new osg::Group() ;
	root3 = convertPly(result_cloud , 1.0 , 0.0 , 0.0) ;
	sw2->addChild(root2) ;
	sw2->addChild(root3) ;

	MultiWidget *mw = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , sw1.get() , sw2.get()) ;
	setCentralWidget(mw) ;

	AlignResultDlg *ard = new AlignResultDlg(this , ar->finalmatrix , ar->score , end_time) ;
	ard->show() ;
}

void mainWindow::slot_jianying()
{
	JianYingDlg *jd = new JianYingDlg ;
	jd->show() ;
}

void mainWindow::slot_possion()
{
	PossionDlg *pd = new PossionDlg ;
	if (!pd->exec())
	{
		return ;
	}

	QProcess pro ;
	QStringList arg ;

	QString cloud = QFileInfo(pd->getpointcloudFile()).baseName() ;
	QString savecloud = QFileInfo(pd->getsavePath()).baseName() ;

	QString str = "/c PoissonRecon --in " + cloud + ".ply --out " + savecloud + ".ply -depth 10 --pointWeigt 0" ;
	arg<<str ;
	pro.start("cmd.exe" , arg) ;
	pro.waitForFinished() ;
	print(tr("possion reconstruct finished!")) ;
}

void mainWindow::slot_simulation()
{
// 	std::string str1 = "1.BMP" ;
// 	std::string str2 = "2.BMP" ;
// 
// 	osg::ref_ptr<osg::Group> root = new osg::Group() ;
// 	osg::ref_ptr<osg::Group> r1 = new osg::Group() ;
// 	osg::ref_ptr<osg::Group> r2 = new osg::Group() ;
// 	r1 = osgloadImage(str1 , osg::Vec3(0.0 , 0.0 , 0.0)) ;
// 	r2 = osgloadImage(str2 , osg::Vec3(100 , 50 , 100)) ;
// 
// 	root->addChild(r1) ;
// 	root->addChild(r2) ;
// 
// 	osgUtil::Optimizer optimizer ;
// 	optimizer.optimize(root) ;
// 
// 	if (m_mulwidget)
// 	{
// 		delete m_mulwidget ;
// 		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , root.get()) ;
// 		setCentralWidget(m_mulwidget) ;
// 	}
// 	else
// 	{
// 		m_mulwidget = new MultiWidget(osgViewer::ViewerBase::SingleThreaded , root.get()) ;
// 		setCentralWidget(m_mulwidget) ;
// 	}
}

osg::ref_ptr<osg::Group> mainWindow::osgloadImage(std::string &image1 , osg::Vec3 &v1)
{
	osg::ref_ptr<osg::Group> root = new osg::Group() ;
	osg::ref_ptr<osg::DrawPixels> bmp = new osg::DrawPixels ;
	bmp->setPosition(v1) ;
	bmp->setImage(osgDB::readImageFile(image1)) ;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode ;
	geode->addDrawable(bmp.get()) ;

	root->addChild(geode.get()) ;
	return root.release() ;
}

void mainWindow::slot_jiezhen()
{
	VideotoFrame *vf = new VideotoFrame ;
	vf->show() ;
}

void mainWindow::slot_gaussfilter()
{
	ImageFilterDlg *ifd = new ImageFilterDlg ;
	ifd->show() ;
}