#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_mainwindow.h"

//#include "pcltype.h"
#include "viewerqt.h"

class Qmenu ;
class QAction ;
class QCloseEvent ;
class SingleCalibrator ;
class SteroCalibrator ;
class GeneratePointCloudDlg ;
class OsgViewrWidget ;
class QSignalMapper ;
class ViewerQT ;
class QMdiArea ;
class QStandardItemModel ;
class QStandardItem ;
class QDragEnterEvent ;
class QDropEvent ;
class QWheelEvent ;
class QMdiArea ;
class PointCloud ;
class MultiWidget ;
class MyDockWiget ;
class QDockWidget ;

typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud ;
typedef ColorCloud::Ptr ColorCloudPtr ;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud ;
typedef Cloud::Ptr CloudPtr ;
class mainWindow : public QMainWindow
{
	Q_OBJECT

public:
	mainWindow(QWidget *parent = 0);
	~mainWindow();
	void createActions() ;
	void createMenus() ;
	void createTools() ;
	void writeSetting() ;
	void readSetting() ;

public slots:
	void slot_new() ;
	void slot_open() ;
	void slot_save() ;
	void slot_saveas() ;

	//编辑
	void slot_clearAll() ;
	void slot_randomSample() ;
	void slot_voxelgridSample() ;


	//视频处理
	void slot_caputureImage() ;
	void slot_captureVideo() ;
	void slot_jiezhen() ;
	void slot_jianying() ;
	void slot_gaussfilter() ;

	void slot_singlecalib() ;
	void slot_sterocalib() ;

	// 点云生成菜单
	void slot_generatepointcloud() ;
	void slot_receivePointcloudInfo(QVariant &var) ;
	void slot_statisticalOutlierRemoval() ;
	void slot_radiusOutlierRemoval() ;
	void slot_cruderegister() ;
	void slot_highregister() ;
	void slot_loadtwoPointcloud() ;
	void slot_NDT() ;
	void slot_possion() ;

	//三维槽函数
	void slot_newViewer() ;
	void slot_zoomin() ;
	void slot_zoomout() ;
	void slot_trackball() ;
	void slot_simulation() ;

	//帮助菜单
	void slot_help() ;
	void slot_about() ;

	void slot_receiveMessage(const QString &str) ;
	//创建新的显示窗口
	ViewerQT *slot_createNewWidget() ;
	void updateMenus() ;
	void updateWindowMenu() ;

	//tree
	void treeItemChanged(QStandardItem *item) ;
	/************************************************************************/
	/* 递归设置所有的子目录为全选或全不选状态                                                                     */
	/************************************************************************/
	void treeItem_checkAllChild(QStandardItem *item , bool check = true) ;
	void treeItem_checkAllChild_recursion(QStandardItem *item , bool check = true) ;
	//根据子节点的改变，更改父节点的选择情况
	void treeItem_CheckChildChanged(QStandardItem *item) ;
	/************************************************************************/
	/* 测量兄弟节点的情况，如果都选中返回Qt::Checked , 都不选中返回Qt::Unchecked ，部分选中返回Qt::PartiallyChecked                                                                     */
	/************************************************************************/
	Qt::CheckState checkSibling(QStandardItem *item) ;
	void slot_treeclick(const QModelIndex &index) ;

signals:
	void sendMessage(QString str) ;

protected:
	void closeEvent(QCloseEvent *event) ;
	void dragEnterEvent(QDragEnterEvent *event) ;
	void dropEvent(QDropEvent *event) ;
	void initTree() ;
	void createTree() ;
	void wheelEvent(QWheelEvent *event) ;
	osg::ref_ptr<osg::Group> osgloadImage(std::string &image1 , osg::Vec3 &v1) ;
	//将点云转为Node类型
	osg::ref_ptr<osg::Group> convertPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) ;
	osg::ref_ptr<osg::Group> convertPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , float r = 0.0 , float g = 0.0 , float b = 0.0) ;
private:
	/************************************************************************/
	/* 统计异常值滤波器
	 * @Input        inputcloud 输入点云
	 * @Input        knn        统计时邻域点数量
	 * @Input        thresh     判断是否为离群点的阈值
	 * @Output       返回滤波后的点云*/
	/************************************************************************/
	 ColorCloudPtr statisticalOutlierRemoval(ColorCloudPtr inputcloud , int knn , double thresh) ;
	/************************************************************************/
	/* 半径异常值滤波
	 * @Input         inputcloud  输入点云
	 * @Input         knn         设置查询点的邻近点集数
	 * @Input         radius      设置邻近点半径
	 * @Ouput         返回滤波后的点云*/
	/************************************************************************/
	ColorCloudPtr radiusOutlierRemoval(ColorCloudPtr inputcloud , int knn , double radius) ;

	//显示属性表
	inline void showProplist(osg::Node *node) ;

private:
	void print(QString str) ;

	//将节点增加旋转球
	osg::ref_ptr<osg::Group> addTrackball(osg::Node *node) ;
private:
	Ui::mainWindowClass ui;
	QSignalMapper *m_windowMapper ;
	//文件菜单
	QMenu *m_filemenu ;
	QAction *m_newAct ;
	QAction *m_openAct ;
	QAction *m_saveAct ;
	QAction *m_saveasAct ;
	QAction *m_closeAct ;
	QAction *m_nextSubwindow ;
	QAction *m_exitAct ;

	//编辑菜单
	QMenu *m_editmenu ;
	QAction *m_clearAll ;
	QMenu *m_sample ;
	QAction *m_randomSampleAct ;//随机下采样
	QAction *m_voxelgridSampleAct ;//构造三维体素格下采样

	//视频菜单
	QMenu *m_videomenu ;
	QAction *m_captureImageAct ;
	QAction *m_capturevideoAct ;
	QAction *m_jiezhen ;//视频解帧动作
	QMenu *m_segmentMenu ;
	QAction *m_jianyingAct ;
	QMenu *m_imagefilterMenu ;
	QAction *m_gaussfilterAct ;

	//标定菜单
	QMenu *m_calibmenu ;
	QAction *m_singlecalibAct ;
	QAction *m_sterocalibAct ;

	//点云菜单
	QMenu *m_pointcloudmenu ;
	QAction *m_generatepointcloudAct ;

	//点云子菜单
	QMenu *m_filtermenu ;
	QAction *m_statisticalOutlierRemovalAct ;
	QAction *m_radiusOutlierRemovalAct ;

	//配准子菜单
	QMenu *m_registermenu ;
	QAction *m_cruderegisterAct ; //粗配准
	QAction *m_loadtwoPointcloudAct ;//加载点云
	QAction *m_highregisterAct ;  //精配准
	QAction *m_NDTAct ;

	//重建子菜单
	QMenu *m_reconstructmenu ;
	QAction *m_possionAct ;

	//3D显示菜单
	QMenu *m_threeviewermenu ;
	QAction *m_newViewerAct ;
	QAction *m_zoominAct ;
	QAction *m_zoomoutAct ;
	QAction *m_trackball ;//旋转球

	QAction *m_simulationAct ;//模拟动作

	//帮助菜单
	QMenu *m_helpmenu ;
	QAction *m_helpAct ;
	QAction *m_aboutAct ;

	QString m_message ; //保存传递过来的消息
	static int m_i ;
	SingleCalibrator *m_single ;//定义单目标定指针
	SteroCalibrator *m_stero ; //定义双目标定指针
	GeneratePointCloudDlg *m_generatecloud ;//生成点云对话框

	//osg
	ViewerQT *m_widget ;
	MultiWidget *m_mulwidget ;

	//主窗口控件
	QDockWidget *m_dockwidget ;
	QDockWidget *m_dockwidget2 ;

	osg::ref_ptr<osg::Switch> m_swroot ;//根开关节点
	osg::ref_ptr<osg::Switch> m_swfilter ;//保存滤波后的点云节点
	bool m_showflag ; //是否显示模型

	//pcl点云文件
	ColorCloudPtr m_colorcloud ; //输入的源点云
	ColorCloudPtr m_filtercloud ;//滤波后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_sampleSource ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_sampleTarget ;
	CloudPtr m_sourcecloud ;
	CloudPtr m_targetcloud ;
	bool m_sampleflag ; //是否下采样

	static int m_statiscalFilterKnn ;
	static double m_statiscalNSigma ;
	static int m_radiusKnn ;
	static double m_radius ;
	QStandardItemModel *m_treemodel ;
	QStandardItemModel *m_treemodel2 ;
};

#endif // MAINWINDOW_H
