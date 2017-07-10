#include <QtGui>
#include <QApplication>
#include "main_window.hpp"

int main(int argc, char **argv) 
{
    QApplication app(argc, argv);
    px4_gcs::ICSL_GCS w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}

//#include "modules/parsing_module.h"
//
//int main(int argc, char** argv)
//{
//	px4_gcs::ParsingModule parser;
//	return 0;
//}
