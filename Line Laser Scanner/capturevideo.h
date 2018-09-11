/************************************************************************/
/* 视频抓取类                                                                     */
/************************************************************************/
#ifndef CAPTUREVIDEO_H
#define CAPTUREVIDEO_H

#include <QDialog>
#include "ui_capturevideo.h"

class CaptureVideo : public QDialog
{
	Q_OBJECT

public:
	CaptureVideo(QWidget *parent = 0);
	~CaptureVideo();

private:
	Ui::CaptureVideo ui;
};

#endif // CAPTUREVIDEO_H
