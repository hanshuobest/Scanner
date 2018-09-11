/************************************************************************/
/* 棋盘格属性对话框                                                                     */
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
	//发射参数信息信号
	void sendParam(QVariant var) ;
private:
	Ui::PattenSizeDlg ui;
	int m_x ; //棋盘格尺寸
	int m_y ; 
	int m_size ;//棋盘格每块大小
	Param m_param ;
};

#endif // PATTENSIZEDLG_H
