#ifndef ABOUTDLG_H
#define ABOUTDLG_H

#include <QDialog>
#include "ui_aboutdlg.h"

class AboutDlg : public QDialog
{
	Q_OBJECT

public:
	AboutDlg(QWidget *parent = 0);
	~AboutDlg();

private:
	Ui::AboutDlg ui;
};

#endif // ABOUTDLG_H
