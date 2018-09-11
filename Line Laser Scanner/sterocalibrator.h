#ifndef STEROCALIBRATOR_H
#define STEROCALIBRATOR_H

#include <QMainWindow>
#include "ui_sterocalibrator.h"
#include <opencv.hpp>
#include "loadstereoimgdlg.h"
#include "CameraCalibrator.h"

class QAction ;
class QMenu ;
class QGraphicsScene ;
class LoadStereoImgDlg ;
class QStandardItemModel ;
class QStandardItem ;
class SteroCalibrator : public QMainWindow
{
	Q_OBJECT

public:
	SteroCalibrator(QWidget *parent = 0);
	~SteroCalibrator();
	void createMenus() ;
	void createActions() ;
	void createTools() ;

protected:
	void createThumbnail(const QStringList strlist1 , const QStringList strlist2) ;
	void paintEvent(QPaintEvent *event) ;
	void setBackImage(const QString strname1 , const QString strname2) ;
	void resizeEvent(QResizeEvent *event) ;
	bool drawchessboardCorners(cv::Mat &src , cv::Mat &dst) ;
	StereoParams *calibrateStereoCamera(const CameraParams &inputparam1 , const CameraParams &inputparam2) ;
public slots:
	void slot_addImage() ;
	void slot_startCalib() ;
 	void slot_receiveImagelist1(QStringList lst) ;
 	void slot_receiveImagelist2(QStringList lst) ;
	void slot_showImage(const QModelIndex &index) ;
	void slot_receiveParam(QVariant var) ;
signals:
	void sendMessage(const QString &str) ;
private:
	Ui::SteroCalibrator ui;
	QAction *m_addImageAct ;
	QAction *m_startCalibAct ;
	QStringList m_list ;//保存图像列表
	QString m_strName1 ;
	QString m_strName2 ;
	QGraphicsScene *m_scene1 ;
	QGraphicsScene *m_scene2 ;
	bool m_flag ;//是否已添加图片
	
	QStringList m_strlst1 ;//保存相机1图像列表
	QStringList m_strlst2 ;//保存相机2图像列表
	std::vector<std::string> m_leftlist ;
	std::vector<std::string> m_rightlist ;
	QStandardItemModel *m_model ;
	int m_imagenums ; //选择图片的数量
	cv::Size m_boardsize ;
	int m_size ;//棋盘格单元尺寸的大小
	LoadStereoImgDlg *m_ld ;//加载对话框
	Param m_param ;
	StereoParams *m_stereoparam ;
	CameraCalibrator *m_calib ;
	
};

#endif // STEROCALIBRATOR_H
