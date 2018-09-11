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

	//�༭
	void slot_clearAll() ;
	void slot_randomSample() ;
	void slot_voxelgridSample() ;


	//��Ƶ����
	void slot_caputureImage() ;
	void slot_captureVideo() ;
	void slot_jiezhen() ;
	void slot_jianying() ;
	void slot_gaussfilter() ;

	void slot_singlecalib() ;
	void slot_sterocalib() ;

	// �������ɲ˵�
	void slot_generatepointcloud() ;
	void slot_receivePointcloudInfo(QVariant &var) ;
	void slot_statisticalOutlierRemoval() ;
	void slot_radiusOutlierRemoval() ;
	void slot_cruderegister() ;
	void slot_highregister() ;
	void slot_loadtwoPointcloud() ;
	void slot_NDT() ;
	void slot_possion() ;

	//��ά�ۺ���
	void slot_newViewer() ;
	void slot_zoomin() ;
	void slot_zoomout() ;
	void slot_trackball() ;
	void slot_simulation() ;

	//�����˵�
	void slot_help() ;
	void slot_about() ;

	void slot_receiveMessage(const QString &str) ;
	//�����µ���ʾ����
	ViewerQT *slot_createNewWidget() ;
	void updateMenus() ;
	void updateWindowMenu() ;

	//tree
	void treeItemChanged(QStandardItem *item) ;
	/************************************************************************/
	/* �ݹ��������е���Ŀ¼Ϊȫѡ��ȫ��ѡ״̬                                                                     */
	/************************************************************************/
	void treeItem_checkAllChild(QStandardItem *item , bool check = true) ;
	void treeItem_checkAllChild_recursion(QStandardItem *item , bool check = true) ;
	//�����ӽڵ�ĸı䣬���ĸ��ڵ��ѡ�����
	void treeItem_CheckChildChanged(QStandardItem *item) ;
	/************************************************************************/
	/* �����ֵܽڵ������������ѡ�з���Qt::Checked , ����ѡ�з���Qt::Unchecked ������ѡ�з���Qt::PartiallyChecked                                                                     */
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
	//������תΪNode����
	osg::ref_ptr<osg::Group> convertPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) ;
	osg::ref_ptr<osg::Group> convertPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , float r = 0.0 , float g = 0.0 , float b = 0.0) ;
private:
	/************************************************************************/
	/* ͳ���쳣ֵ�˲���
	 * @Input        inputcloud �������
	 * @Input        knn        ͳ��ʱ���������
	 * @Input        thresh     �ж��Ƿ�Ϊ��Ⱥ�����ֵ
	 * @Output       �����˲���ĵ���*/
	/************************************************************************/
	 ColorCloudPtr statisticalOutlierRemoval(ColorCloudPtr inputcloud , int knn , double thresh) ;
	/************************************************************************/
	/* �뾶�쳣ֵ�˲�
	 * @Input         inputcloud  �������
	 * @Input         knn         ���ò�ѯ����ڽ��㼯��
	 * @Input         radius      �����ڽ���뾶
	 * @Ouput         �����˲���ĵ���*/
	/************************************************************************/
	ColorCloudPtr radiusOutlierRemoval(ColorCloudPtr inputcloud , int knn , double radius) ;

	//��ʾ���Ա�
	inline void showProplist(osg::Node *node) ;

private:
	void print(QString str) ;

	//���ڵ�������ת��
	osg::ref_ptr<osg::Group> addTrackball(osg::Node *node) ;
private:
	Ui::mainWindowClass ui;
	QSignalMapper *m_windowMapper ;
	//�ļ��˵�
	QMenu *m_filemenu ;
	QAction *m_newAct ;
	QAction *m_openAct ;
	QAction *m_saveAct ;
	QAction *m_saveasAct ;
	QAction *m_closeAct ;
	QAction *m_nextSubwindow ;
	QAction *m_exitAct ;

	//�༭�˵�
	QMenu *m_editmenu ;
	QAction *m_clearAll ;
	QMenu *m_sample ;
	QAction *m_randomSampleAct ;//����²���
	QAction *m_voxelgridSampleAct ;//������ά���ظ��²���

	//��Ƶ�˵�
	QMenu *m_videomenu ;
	QAction *m_captureImageAct ;
	QAction *m_capturevideoAct ;
	QAction *m_jiezhen ;//��Ƶ��֡����
	QMenu *m_segmentMenu ;
	QAction *m_jianyingAct ;
	QMenu *m_imagefilterMenu ;
	QAction *m_gaussfilterAct ;

	//�궨�˵�
	QMenu *m_calibmenu ;
	QAction *m_singlecalibAct ;
	QAction *m_sterocalibAct ;

	//���Ʋ˵�
	QMenu *m_pointcloudmenu ;
	QAction *m_generatepointcloudAct ;

	//�����Ӳ˵�
	QMenu *m_filtermenu ;
	QAction *m_statisticalOutlierRemovalAct ;
	QAction *m_radiusOutlierRemovalAct ;

	//��׼�Ӳ˵�
	QMenu *m_registermenu ;
	QAction *m_cruderegisterAct ; //����׼
	QAction *m_loadtwoPointcloudAct ;//���ص���
	QAction *m_highregisterAct ;  //����׼
	QAction *m_NDTAct ;

	//�ؽ��Ӳ˵�
	QMenu *m_reconstructmenu ;
	QAction *m_possionAct ;

	//3D��ʾ�˵�
	QMenu *m_threeviewermenu ;
	QAction *m_newViewerAct ;
	QAction *m_zoominAct ;
	QAction *m_zoomoutAct ;
	QAction *m_trackball ;//��ת��

	QAction *m_simulationAct ;//ģ�⶯��

	//�����˵�
	QMenu *m_helpmenu ;
	QAction *m_helpAct ;
	QAction *m_aboutAct ;

	QString m_message ; //���洫�ݹ�������Ϣ
	static int m_i ;
	SingleCalibrator *m_single ;//���嵥Ŀ�궨ָ��
	SteroCalibrator *m_stero ; //����˫Ŀ�궨ָ��
	GeneratePointCloudDlg *m_generatecloud ;//���ɵ��ƶԻ���

	//osg
	ViewerQT *m_widget ;
	MultiWidget *m_mulwidget ;

	//�����ڿؼ�
	QDockWidget *m_dockwidget ;
	QDockWidget *m_dockwidget2 ;

	osg::ref_ptr<osg::Switch> m_swroot ;//�����ؽڵ�
	osg::ref_ptr<osg::Switch> m_swfilter ;//�����˲���ĵ��ƽڵ�
	bool m_showflag ; //�Ƿ���ʾģ��

	//pcl�����ļ�
	ColorCloudPtr m_colorcloud ; //�����Դ����
	ColorCloudPtr m_filtercloud ;//�˲���ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_sampleSource ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_sampleTarget ;
	CloudPtr m_sourcecloud ;
	CloudPtr m_targetcloud ;
	bool m_sampleflag ; //�Ƿ��²���

	static int m_statiscalFilterKnn ;
	static double m_statiscalNSigma ;
	static int m_radiusKnn ;
	static double m_radius ;
	QStandardItemModel *m_treemodel ;
	QStandardItemModel *m_treemodel2 ;
};

#endif // MAINWINDOW_H
