#ifndef px4_gcs_LOGGING_MODULE_H
#define px4_gcs_LOGGING_MODULE_H

#include <QtGui/QListView>
#include <QStringListModel>
#include <string>

#include "modules/module.h"

namespace px4_gcs
{

class LoggingModule : public Module
{
	public:
		LoggingModule(QListView* _widget);
		~LoggingModule();
		void log( const std::string );

	private:
		QListView* widget;
		QStringListModel* model;
};

} // namespace px4_gcs
#endif
