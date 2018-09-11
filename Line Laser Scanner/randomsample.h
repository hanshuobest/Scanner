#ifndef RANDOMSAMPLE_H
#define RANDOMSAMPLE_H

#include <QDialog>
#include "ui_randomsample.h"

class RandomSample : public QDialog
{
	Q_OBJECT

public:
	RandomSample(QWidget *parent = 0);
	~RandomSample();
	int getSample() const ;
private:
	Ui::RandomSample ui;
	static int m_sample ;
};

#endif // RANDOMSAMPLE_H
