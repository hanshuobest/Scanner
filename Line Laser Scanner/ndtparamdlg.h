#ifndef NDTPARAMDLG_H
#define NDTPARAMDLG_H

#include <QDialog>
#include "ui_ndtparamdlg.h"

class NDTparamDlg : public QDialog
{
	Q_OBJECT

public:
	NDTparamDlg(QWidget *parent = 0);
	~NDTparamDlg();
	float getstep()const ;
	float getresolution() const ;
private:
	Ui::NDTparamDlg ui;
	float m_step ;
	float m_resolution ;
};

#endif // NDTPARAMDLG_H
