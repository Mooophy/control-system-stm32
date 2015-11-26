/**
  *****************************************************************************
  * @file    main.cpp
  * @author  Yue Wang
  * @date    03-June-2013
  * @brief   PC part, using Qt
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
