#ifndef VOXELGRIDSAMPLEDLG_H
#define VOXELGRIDSAMPLEDLG_H

#include <QDialog>
#include "ui_voxelgridsampledlg.h"

class VoxelgridSampleDlg : public QDialog
{
	Q_OBJECT

public:
	VoxelgridSampleDlg(QWidget *parent = 0);
	~VoxelgridSampleDlg();
	double getleafsize() const ;
public slots:
	void slot_default() ;
	void slot_help() ;
private:
	Ui::VoxelgridSampleDlg ui;
};

#endif // VOXELGRIDSAMPLEDLG_H
