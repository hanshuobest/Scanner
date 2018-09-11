#ifndef STATICSOUTLIERREMOVALDLG_H
#define STATICSOUTLIERREMOVALDLG_H

#include <QDialog>
#include "ui_staticsoutlierremovaldlg.h"

class StaticsOutlierRemovalDlg : public QDialog
{
	Q_OBJECT

public:
	StaticsOutlierRemovalDlg(QWidget *parent = 0);
	~StaticsOutlierRemovalDlg();
public:
	int getKnn() const ;
	void setKnn(int Knn) ;
	double getNsigma() const ;
	void setNsigma(double sigma) ;
private:
	Ui::StaticsOutlierRemovalDlg ui;
	static int m_knn ;
	static double m_nsigma ;
};

#endif // STATICSOUTLIERREMOVALDLG_H
