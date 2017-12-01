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
		QShortcut* key10;
		QShortcut* key11;
		QShortcut* key12;
		QShortcut* key13;
		QShortcut* key14;
		QShortcut* key15;
		QShortcut* key16;
		QShortcut* key17;
		QShortcut* key18;
		QShortcut* key19;
};

} // namespace px4_gcs

#endif
