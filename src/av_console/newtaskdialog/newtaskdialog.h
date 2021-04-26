#ifndef NEWTASKDIALOG_H
#define NEWTASKDIALOG_H

#include <QDialog>

namespace Ui {
class NewTaskDialog;
}

class NewTaskDialog : public QDialog
{
    Q_OBJECT

public:
    explicit NewTaskDialog(QWidget *parent = 0);
    ~NewTaskDialog();

private:
    Ui::NewTaskDialog *ui;
};

#endif // NEWTASKDIALOG_H
