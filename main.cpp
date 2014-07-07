/**
  *****************************************************************************
  * @file    main.cpp
  * @author  Yue Wang 12027710
  * @version V1.0.0
  * @date    03-June-2013
  * @brief   For the Assignment 6, 159.270 Hardware Oriented Programming.
   ****************************************************************************
  */

#include "dialog.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dialog w;
    w.show();

    return a.exec();
}
