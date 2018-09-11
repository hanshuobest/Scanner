#ifndef CAPTUREIMAGEDLG_H
#define CAPTUREIMAGEDLG_H

#include <QDialog>
#include "ui_CaptureImageDlg.h"
class CaptureImageDlg : public QDialog , public Ui::ImageDlg
{
	Q_OBJECT

public:
	CaptureImageDlg(QWidget *parent);
	CaptureImageDlg() ;
	~CaptureImageDlg();

private:
	
};

#endif // CAPTUREIMAGEDLG_H
