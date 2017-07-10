#include "modules/logging_module.h"

namespace px4_gcs
{

LoggingModule::LoggingModule(QListView* _widget)
{
	widget = new QListView;
	widget = _widget;

	model = new QStringListModel;

	widget->setModel( model );
}

LoggingModule::~LoggingModule()
{
	delete widget;
	delete model;
}

void LoggingModule::log( const std::string msg )
{
	QVariant new_row( QString(msg.c_str()) );

	model->insertRows( model->rowCount(), 1 );
	model->setData( model->index(model->rowCount()-1), new_row);
	widget->scrollToBottom();
}

} // namespace px4_gcs
