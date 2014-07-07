/**
  *****************************************************************************
  * @file    dialog.cpp
  * @author  Yue Wang 12027710
  * @version V1.0.0
  * @date    03-June-2013
  * @brief   For the Assignment 6, 159.270 Hardware Oriented Programming.
   ****************************************************************************
  */

#include "dialog.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    PortSettings settings = {BAUD9600, DATA_8, PAR_NONE, STOP_1, FLOW_OFF, 10};
    port = new QextSerialPort("COM3", settings, QextSerialPort::EventDriven);
    connect(port, SIGNAL(readyRead()), SLOT(serialDataReady()));
    port->open(QIODevice::ReadWrite);
}

Dialog::~Dialog()
{
    delete ui;
    delete port;
}

void Dialog::pushButtonClicked(void)
{
    QByteArray qba = (ui->lineEdit->text() + '\n').toLatin1();
    port->write(qba.data());

    QString s = ui->lineEdit->text();
    if((ui->lineEdit->text()).startsWith('M'))
    {
        if((s.at(1)).isSpace())
        {
            s = s.remove(0, 2);
        }
        else
        {
            s = s.remove(0, 1);
        }
     ui->label->setText(s);
    }
}

void Dialog::serialDataReady()
{
    if (port->canReadLine())
    {
        char s[80];
        port->readLine(s, 80);
        s[strlen(s)-1] = '\0';
        switch (s[0])
        {
        case 'U':
            Dialog::UsReceiver(s); break;
        case -100:
            ui->lineEdit->clear(); break;
        default:
            ui->label->setText(s); break;
        }
    }
}

void Dialog::ServoCMDchanged(int _cmd)
{
    QString s = QString::number( _cmd);

    QByteArray qba = ('s'+ s + '\n').toLatin1();
    port->write(qba.data());
}


void Dialog::BlueLedClicked(bool _state)
{
    if(_state)
    {
        QString s="b1";//"b1" turns on blue LED.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
    else
    {
        QString s="b0";//"b0" turns off blue LED.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
}

void Dialog::GreenLedClicked(bool _state)
{
    if(_state)
    {
        QString s="g1";//"X" turns on green LED.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
    else
    {
        QString s="g0";//"x" turns off green LED.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
}

void Dialog::OnOff_CheckBoxSTM_Clicked(bool _state)
{
    if(_state)
    {
        QString s="c1";//"c1" turns on stepper.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
    else
    {
        QString s="c0";//"c0" turns off stepper.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
}

void Dialog::Direction_CheckBoxSTM_Clicked(bool _state)
{
    if(_state)
    {
        QString s="d1";//"d1" clockwise.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
    else
    {
        QString s="d0";//"d0" counter clockwise.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
}

void Dialog::Speed_SpliterSTM_ValueChanged(int _spd)
{
    QString s = QString::number( _spd);

    QByteArray qba = ('R'+ s + '\n').toLatin1();
    port->write(qba.data());
}

void Dialog::OnOff_CheckBoxUS_Clicked(bool _state)
{
    if(_state)
    {
        QString s="u1";//"u1" turns on Usensor.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
    else
    {
        QString s="u0";//"u0" turns off Usensor.
        QByteArray qba = (s + '\n').toLatin1();
        port->write(qba.data());
    }
}

void Dialog::UsReceiver(char *_s)
{
    uint16_t d = (_s[2]<<8) | _s[3];

    ui->progressBar_Us->setValue(d);
    QString distance = QString::number(d);
    ui->label->setText(distance);
}




