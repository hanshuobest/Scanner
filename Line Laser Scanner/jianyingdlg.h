#ifndef JIANYINGDLG_H
#define JIANYINGDLG_H

#include <QDialog>
#include "ui_jianyingdlg.h"

class JianYingDlg : public QDialog
{
	Q_OBJECT

public:
	JianYingDlg(QWidget *parent = 0);
	~JianYingDlg();
public slots:
	void slot_selectBack() ;
	void slot_selectK() ;
	void slot_selectK1() ;
	void slot_selectTar() ;
	void slot_process() ;
private:
	Ui::JianYingDlg ui;
	std::string  m_k ;
	std::string m_k1 ;
	std::string m_target ;
	std::string m_back ;

};

#endif // JIANYINGDLG_H
