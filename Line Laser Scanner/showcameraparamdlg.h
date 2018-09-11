#ifndef SHOWCAMERAPARAMDLG_H
#define SHOWCAMERAPARAMDLG_H

#include <QDialog>
#include "ui_showcameraparamdlg.h"
#include <opencv.hpp>


class QStandardItemModel ;
class ShowCameraParamDlg : public QDialog
{
	Q_OBJECT

public:
//	ShowCameraParamDlg(QWidget *parent = 0);
	ShowCameraParamDlg(QWidget *parent , cv::Mat &inMatrix , cv::Mat &rMatrix , cv::Mat &tMatrix , cv::Mat &dMatrix) ;
	~ShowCameraParamDlg();
	void setupModel() ;
	void setupView() ;
	void createActions() ;

public slots:

	void slot_saveIn() ;
	void slot_saveR() ;
	void slot_saveT() ;
	void slot_saveD() ;

private:
	Ui::ShowCameraParamDlg ui;
	QString m_path ;
	QStandardItemModel *m_model ;
	QStandardItemModel *m_model2 ;
	QStandardItemModel *m_model3 ;
	QStandardItemModel *m_model4 ;
	cv::Mat m_intrinsicmatrix ;
	cv::Mat m_rotationmatrix ;
	cv::Mat m_translationmatrix ;
	cv::Mat m_distortvector ;
};

#endif // SHOWCAMERAPARAMDLG_H
