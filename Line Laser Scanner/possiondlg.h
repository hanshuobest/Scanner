#ifndef POSSIONDLG_H
#define POSSIONDLG_H

#include <QDialog>
#include "ui_poissonrecon.h"
class PossionDlg : public QDialog , public Ui::PoissonRecon
{
	Q_OBJECT

public:
	PossionDlg();
	~PossionDlg();
	QString getpointcloudFile() const ;
	QString getsavePath() const ;
public slots:
	void slot_select() ;
	void slot_saveas() ;
private:
	QString m_file ;
	QString m_savepath ;
	Ui_PoissonRecon ui ;
};

#endif // POSSIONDLG_H
