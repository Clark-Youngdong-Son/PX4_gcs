#ifndef px4_gcs_PARSING_MODULE_H
#define px4_gcs_PARSING_MODULE_H

#include "libxml/xmlmemory.h"
#include "libxml/parser.h"
#include "libxml/tree.h"
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <QtGui/QTableWidget>

#include "modules/module.h"

namespace px4_gcs
{

class ParameterModule : public Module
{
	public:
		ParameterModule();
		~ParameterModule();
		void add_widget( QTableWidget* );
		void load( std::string );
		void save( std::string );
	
		void log( const std::string );
		void update_fcu_values( std::vector<std::string>, std::vector<double> );
		void query_list( std::vector<std::string>&, std::vector<std::string>&, std::vector<double>& );

	private:
		std::vector<QTableWidget*> widgets;
		std::vector<QStringList> headers;
		std::vector<QStringList> names;
		std::vector<QStringList> types;

		std::string blank;
};

} // namespace px4_gcs
#endif
