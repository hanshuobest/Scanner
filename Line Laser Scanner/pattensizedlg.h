/************************************************************************/
/* ���̸����ԶԻ���                                                                     */
/************************************************************************/
#ifndef PATTENSIZEDLG_H
#define PATTENSIZEDLG_H

#include <QDialog>
#include "ui_pattensizedlg.h"
#include <QMetaType>
#include "struct_type.h"

class QVariant ;
class PattenSizeDlg : public QDialog
{
	Q_OBJECT

public:
	PattenSizeDlg(QWidget *parent = 0);
	~PattenSizeDlg();

public slots:
	void slot_Ok() ;
	void slot_selectComb(int index) ;
signals:
	//���������Ϣ�ź�
	void sendParam(QVariant var) ;
private:
	Ui::PattenSizeDlg ui;
	int m_x ; //���̸�ߴ�
	int m_y ; 
	int m_size ;//���̸�ÿ���С
	Param m_param ;
};

#endif // PATTENSIZEDLG_H
