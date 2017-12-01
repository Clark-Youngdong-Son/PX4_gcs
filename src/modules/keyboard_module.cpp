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
	key12 = new QShortcut(Qt::Key_N, widget);
	key13 = new QShortcut(Qt::Key_I, widget);
	key14 = new QShortcut(Qt::Key_J, widget);
	key15 = new QShortcut(Qt::Key_K, widget);
	key16 = new QShortcut(Qt::Key_L, widget);
	key17 = new QShortcut(Qt::Key_T, widget);
	key18 = new QShortcut(Qt::Key_G, widget);
	key19 = new QShortcut(Qt::Key_R, widget);
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
	delete key12;
	delete key13;
	delete key14;
	delete key15;
	delete key16;
	delete key17;
	delete key18;
	delete key19;
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
	QObject::connect( key12, SIGNAL(activated()), node, SLOT(on_btn_N_pressed()) );
	QObject::connect( key13, SIGNAL(activated()), node, SLOT(on_btn_I_pressed()) );
	QObject::connect( key14, SIGNAL(activated()), node, SLOT(on_btn_J_pressed()) );
	QObject::connect( key15, SIGNAL(activated()), node, SLOT(on_btn_K_pressed()) );
	QObject::connect( key16, SIGNAL(activated()), node, SLOT(on_btn_L_pressed()) );
	QObject::connect( key17, SIGNAL(activated()), node, SLOT(on_btn_T_pressed()) );
	QObject::connect( key18, SIGNAL(activated()), node, SLOT(on_btn_G_pressed()) );
	QObject::connect( key19, SIGNAL(activated()), node, SLOT(on_btn_R_pressed()) );
}

void KeyboardModule::log( const string msg )
{
	for(unsigned int i=0; i<modules.size(); i++)
		modules[i]->log( msg );
}

} // namespace px4_gcs
