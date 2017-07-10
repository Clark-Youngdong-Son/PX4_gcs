#ifndef px4_gcs_KEYBOARD_MODULE_H
#define px4_gcs_KEYBOARD_MODULE_H

#include <QtGui/QWidget>
#include <QObject>
#include <QShortcut>

#include <string>

#include "modules/module.h"

namespace px4_gcs
{

class KeyboardModule : public Module
{
	public:
		KeyboardModule( QWidget*, QObject* );
		~KeyboardModule();

		void connect();
		void log( const std::string );

	private:
		QWidget* widget;
		QObject* node;
		QShortcut* key1;
		QShortcut* key2;
		QShortcut* key3;	
		QShortcut* key4;
		QShortcut* key5;
		QShortcut* key6;
		QShortcut* key7;
		QShortcut* key8;
		QShortcut* key9;
};

} // namespace px4_gcs

#endif
