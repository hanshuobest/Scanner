#ifndef IMAGEFILTERDLG_H
#define IMAGEFILTERDLG_H

#include <QDialog>
#include "ui_imagefilterdlg.h"
#include <opencv.hpp>
class QGraphicsScene ;

class ImageFilterDlg : public QDialog
{
	Q_OBJECT

public:
	ImageFilterDlg(QWidget *parent = 0);
	~ImageFilterDlg();

public slots:
	void slot_selectImage() ;
	void slot_gauss() ;
	void slot_media() ;

private:
	Ui::ImageFilterDlg ui;
	QGraphicsScene *m_scene ;
	cv::Mat m_src ;
	cv::Mat m_dst ;
	float m_scale ;
};

#endif // IMAGEFILTERDLG_H
