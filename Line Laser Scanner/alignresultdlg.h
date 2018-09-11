#ifndef ALIGNRESULTDLG_H
#define ALIGNRESULTDLG_H

#include <QDialog>
#include "ui_alignresultdlg.h"
#include <Eigen3/Eigen/Eigen>
class AlignResultDlg : public QDialog
{
	Q_OBJECT

public:
	AlignResultDlg(QWidget *parent , Eigen::Matrix4f &matrix , double score , double eclipstime) ;
	~AlignResultDlg();

private:
	Ui::AlignResultDlg ui;
	Eigen::Matrix4f m_finaltransformation ;
	double m_score ;
	double m_escapetime ;
};

#endif // ALIGNRESULTDLG_H
