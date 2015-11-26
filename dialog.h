/**
  *****************************************************************************
  * @file    dialog.h
  * @author  Yue Wang
  * @date    03-June-2013
  * @brief   PC part, using Qt.
   ****************************************************************************
  */
#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "qextserialport/qextserialport.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

public slots:
    void pushButtonClicked();
    void serialDataReady();
    void ServoCMDchanged(int _cmd);
    void BlueLedClicked(bool _state);
    void GreenLedClicked(bool _state);
    void OnOff_CheckBoxSTM_Clicked(bool _state);
    void Direction_CheckBoxSTM_Clicked(bool _state);
    void Speed_SpliterSTM_ValueChanged(int _spd);
    void OnOff_CheckBoxUS_Clicked(bool _state);

private:
    Ui::Dialog *ui;
    QextSerialPort *port;
    void UsReceiver(char *_s);
};

#endif // DIALOG_H
