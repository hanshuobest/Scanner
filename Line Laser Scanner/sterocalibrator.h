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
	QStringList m_list ;//����ͼ���б�
	QString m_strName1 ;
	QString m_strName2 ;
	QGraphicsScene *m_scene1 ;
	QGraphicsScene *m_scene2 ;
	bool m_flag ;//�Ƿ������ͼƬ
	
	QStringList m_strlst1 ;//�������1ͼ���б�
	QStringList m_strlst2 ;//�������2ͼ���б�
	std::vector<std::string> m_leftlist ;
	std::vector<std::string> m_rightlist ;
	QStandardItemModel *m_model ;
	int m_imagenums ; //ѡ��ͼƬ������
	cv::Size m_boardsize ;
	int m_size ;//���̸�Ԫ�ߴ�Ĵ�С
	LoadStereoImgDlg *m_ld ;//���ضԻ���
	Param m_param ;
	StereoParams *m_stereoparam ;
	CameraCalibrator *m_calib ;
	
};

#endif // STEROCALIBRATOR_H
