#ifndef SHOWSTEREOCAMERAPARAMDLG_H
#define SHOWSTEREOCAMERAPARAMDLG_H

#include <QDialog>
#include "ui_showstereocameraparamdlg.h"
#include <opencv.hpp>

class QCloseEvent ;
class QStandardItemModel ;
class ShowStereoCameraParamDlg : public QDialog
{
	Q_OBJECT

public:
	ShowStereoCameraParamDlg(QWidget *parent , cv::Mat &rmatrix , cv::Mat &tmatrix) ;
	~ShowStereoCameraParamDlg();

protected:
	void closeEvent(QCloseEvent *event) ;
	void setupModel() ;
	void setupView() ;

public slots:
	void slot_saveR() ;
	void slot_saveT() ;
signals:
	void sendMessage(QString str) ;
private:
	Ui::ShowStereoCameraParamDlg ui;
	QStandardItemModel *m_model ;
	QStandardItemModel *m_model2 ;
	QString m_path ;
	cv::Mat m_rmatrix ;
	cv::Mat m_tmatrix ;
};

#endif // SHOWSTEREOCAMERAPARAMDLG_H
