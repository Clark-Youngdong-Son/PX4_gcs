#include "modules/keyboard_module.h"

using namespace std;

namespace px4_gcs
{

KeyboardModule::KeyboardModule( QWidget* _widget, QObject* _object )
{
	// assign parent widget
	widget = new QWidget;
	widget = _widget;
	node = new QObject;
	node = _object;

	key1 = new QShortcut(Qt::Key_Z, widget);	
	key2 = new QShortcut(Qt::Key_Space, widget);
	key3 = new QShortcut(Qt::Key_X, widget);
	key4 = new QShortcut(Qt::Key_O, widget);
	key5 = new QShortcut(Qt::Key_P, widget);
	key6 = new QShortcut(Qt::Key_A, widget);
	key7 = new QShortcut(Qt::Key_D, widget);
	key8 = new QShortcut(Qt::Key_W, widget);
	key9 = new QShortcut(Qt::Key_S, widget);
	key10 = new QShortcut(Qt::Key_M, widget);
	key11 = new QShortcut(Qt::Key_Comma, widget);
}

KeyboardModule::~KeyboardModule()
{
	delete key1;
	delete key2;
	delete key3;
	delete key4;
	delete key5;
	delete key6;
	delete key7;
	delete key8;
	delete key9;
	delete key10;
	delete key11;
	delete widget;
	delete node;
}

void KeyboardModule::connect()
{
	//QObject::connect( key1, SIGNAL(activated()), node, SLOT(on_btn_Z_pressed()) );
	//QObject::connect( key2, SIGNAL(activated()), node, SLOT(on_btn_Space_pressed()) );
	//QObject::connect( key3, SIGNAL(activated()), node, SLOT(on_btn_X_pressed()) );
	QObject::connect( key4, SIGNAL(activated()), node, SLOT(on_btn_O_pressed()) );
	QObject::connect( key5, SIGNAL(activated()), node, SLOT(on_btn_P_pressed()) );
	QObject::connect( key6, SIGNAL(activated()), node, SLOT(on_btn_A_pressed()) );
	QObject::connect( key7, SIGNAL(activated()), node, SLOT(on_btn_D_pressed()) );
	QObject::connect( key8, SIGNAL(activated()), node, SLOT(on_btn_W_pressed()) );
	QObject::connect( key9, SIGNAL(activated()), node, SLOT(on_btn_S_pressed()) );
	QObject::connect( key10, SIGNAL(activated()), node, SLOT(on_btn_M_pressed()) );
	QObject::connect( key11, SIGNAL(activated()), node, SLOT(on_btn_Comma_pressed()) );
}

void KeyboardModule::log( const string msg )
{
	for(unsigned int i=0; i<modules.size(); i++)
		modules[i]->log( msg );
}

} // namespace px4_gcs
