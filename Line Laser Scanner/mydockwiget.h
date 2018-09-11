#ifndef MYDOCKWIGET_H
#define MYDOCKWIGET_H

#include <QWidget>
#include "ui_mydockwiget.h"

class MyDockWiget : public QWidget
{
	Q_OBJECT

public:
	MyDockWiget(QWidget *parent , int num , float radius , float x , float y , float z) ;
	~MyDockWiget();

private:
	Ui::MyDockWiget ui;

};

#endif // MYDOCKWIGET_H
