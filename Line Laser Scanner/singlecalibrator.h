#ifndef SINGLECALIBRATOR_H
#define SINGLECALIBRATOR_H

#include <QMainWindow>
#include "ui_singlecalibrator.h"
#include <QTime>
#include <opencv.hpp>
#include "pattensizedlg.h"
#include "CameraCalibrator.h"
class QMenu ;
class QAction ;
class QGraphicsScene ;
class PattenSizeDlg ;
class QVariant ;
class ShowCameraParamDlg ;
class SingleCalibrator : public QMainWindow
{
	Q_OBJECT

public:
	SingleCalibrator(QWidget *parent = 0);
	~SingleCalibrator();
	void createMenus() ;
	void createActions() ;
	void createTools() ;

protected:
	void createThumbnail(const QStringList strlist) ;
	void paintEvent(QPaintEvent *event) ;
	void setBackImage(const QString strname) ;
	bool drawchessboardCorners(cv::Mat &src , cv::Mat &dst) ;
public slots:
	void slot_addImage() ;
	void slot_startCalib() ;
	void slot_itemclick(QListWidgetItem *item) ;
	void slot_recieveParam(QVariant var) ;
signals:
	void sendMessage(const QString &str) ;
private:
	Ui::SingleCalibrator ui;
	QAction *m_addImageAct ;
	QAction *m_startCalibAct ;
	QStringList m_list ;//����ͼ���б�
	QString m_strName ;
	QGraphicsScene *m_scene ;
	float m_w ;
	float m_h ;
	float m_scale ;
    bool m_flag ;//�Ƿ������ͼƬ
	QTime m_time ;

	PattenSizeDlg *m_psdlg ; //���̸����ԶԻ���
	cv::Size m_boardsize ;//���̸�ߴ�
	int m_size ;//���̸�Ԫ��С
	Param m_param ;

	CameraCalibrator m_Calib ;
	CameraParams m_cameraparam ;//�������ָ�����

	ShowCameraParamDlg *m_showCamParam ;
};

#endif // SINGLECALIBRATOR_H
