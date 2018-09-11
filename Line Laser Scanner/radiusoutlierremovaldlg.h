#ifndef RADIUSOUTLIERREMOVALDLG_H
#define RADIUSOUTLIERREMOVALDLG_H

#include <QDialog>
#include "ui_radiusoutlierremovaldlg.h"

class RadiusOutlierRemovalDlg : public QDialog
{
	Q_OBJECT

public:
	RadiusOutlierRemovalDlg(QWidget *parent = 0);
	~RadiusOutlierRemovalDlg();
	void setKnn(int knn) ;
	int getKnn() const ;
	void setradius(double radius) ;
	double getradius() const ;
private:
	Ui::RadiusOutlierRemovalDlg ui;
	static int m_knn ;
	static double m_radius ;
};

#endif // RADIUSOUTLIERREMOVALDLG_H
