#include "modules/parameter_module.h"

using namespace std;

namespace px4_gcs
{

ParameterModule::ParameterModule()
{ 
	blank = "                         ";
	widgets.clear();
	headers.clear();
	names.clear();
	types.clear();
}

ParameterModule::~ParameterModule()
{
	widgets.clear();
	headers.clear();
	names.clear();
	types.clear();
}

void ParameterModule::add_widget( QTableWidget* _widget )
{	
	QStringList labels;
	for(int i=0; i<20; i++)
		labels << blank.c_str();
	
	_widget->setVerticalHeaderLabels(labels);
	widgets.push_back( _widget );
}

void ParameterModule::load( string filename )
{
	xmlDoc* doc = xmlParseFile( filename.c_str() );
	xmlNode* top = xmlDocGetRootElement( doc );
	xmlNode* cur = top->xmlChildrenNode->next;

	int i = 0;
	while( cur != NULL )
	{
		xmlNode* child = cur->xmlChildrenNode->next;	
		
		QStringList _header;
		QStringList _name;
		QStringList _type;

		int j = 0;
		while( child != NULL )
		{
			// name of node
			_name << (const char*)child->name;
			
			// type
			xmlNode* sub = child->xmlChildrenNode->next;
			_type << (const char*)xmlNodeGetContent(sub);

			// description
			sub = sub->next->next;
			_header << (const char*)xmlNodeGetContent(sub);
	
			// value
			sub = sub->next->next;
			if( strcmp(_type[j].toStdString().c_str(), "double") ) 
			{
				long int _value = strtol((const char*)xmlNodeGetContent(sub), NULL, 10);
				QTableWidgetItem* tmp 
					= new QTableWidgetItem( QString::number(_value), 0 );
				widgets[i]->setItem(j,1,tmp);
			}
			else
			{
				double _value = strtod((const char*)xmlNodeGetContent(sub), NULL);
				QTableWidgetItem* tmp 
					= new QTableWidgetItem( QString::number(_value,'f',4), 0 );
				widgets[i]->setItem(j,1,tmp);
			}

			child = child->next->next;
			j++;
		}
		widgets[i]->setVerticalHeaderLabels( _header );
		names.push_back( _name );
		headers.push_back( _header );
		types.push_back( _type );

		cur = cur->next->next;
		i++;
	}

	log( "loading parameters finished." );
}

void ParameterModule::save( string filename )
{
	xmlDoc* _doc = xmlNewDoc( (xmlChar*)"1.0" );
	xmlNode* _root = xmlNewNode(NULL, (xmlChar*)"xml");
	xmlDocSetRootElement(_doc, _root);
	
	for(unsigned int i=0; i<names.size(); i++)
	{
		ostringstream i_ss; 
		i_ss << i+1;
		xmlNode* _tbl = xmlNewNode(NULL, (xmlChar*)("tbl_"+i_ss.str()).c_str());
	
		for(unsigned int j=0; j<names[i].length(); j++)
		{
			xmlNode* _name = xmlNewNode(NULL, (xmlChar*)names[i][j].toStdString().c_str());
			xmlNode* _description = xmlNewNode(NULL, (xmlChar*)"description");
			xmlNode* _value = xmlNewNode(NULL, (xmlChar*)"value");
			
			ostringstream value_ss;
			value_ss << widgets[i]->item(j,1)->text().toStdString();
			xmlNodeSetContent(_description, (xmlChar*)headers[i][j].toStdString().c_str());
			xmlNodeSetContent(_value, (xmlChar*)value_ss.str().c_str());

			xmlAddChild( _name, _description );
			xmlAddChild( _name, _value );
			xmlAddChild( _tbl, _name );
		}
		xmlAddChild( _root, _tbl );
	}

	int res = xmlSaveFormatFileEnc( filename.c_str(), _doc, "UTF-8", 1 );
	log( "saving parameters finished." );
}

void ParameterModule::update_fcu_values( vector<string> _name, vector<double> _value )
{
	// serch name and update corresponding values
	for(unsigned int i=0; i<_name.size(); i++)
	{
		QString tmp_name = QString( _name[i].c_str() );
		for(unsigned int j=0; j<names.size(); j++) // j : table idx
		{
			int idx = names[j].indexOf(tmp_name); // idx : row of table
			if( idx != -1 )
			{
				// in this case, name found.
				if( strcmp(types[j][idx].toStdString().c_str(), "double") )
				{
					QTableWidgetItem* tmp 
						= new QTableWidgetItem( QString::number(_value[i]), 0 );
					widgets[j]->setItem(idx,0,tmp);
				}
				else
				{
					QTableWidgetItem* tmp 
						= new QTableWidgetItem( QString::number(_value[i],'f',4), 0 );
					widgets[j]->setItem(idx,0,tmp);
				}
			}
		}
	}
}

void ParameterModule::query_list(vector<string>& _names, 
								vector<string>& _types, 
								vector<double>& _values)
{
	_names.clear();
	_types.clear();
	_values.clear();

	for( unsigned int i=0; i<headers.size(); i++)
	{
		for( unsigned int j=0; j<headers[i].length(); j++)
		{
			// first check any data in FCU column
			if( widgets[i]->item(j,0) == NULL ) // it means there is no data
			{
				// so add to query name
				_names.push_back( names[i][j].toStdString() );
				_types.push_back( types[i][j].toStdString() );
				_values.push_back( widgets[i]->item(j,1)->text().toDouble() );
			}
			else
			{
				if( strcmp(widgets[i]->item(j,0)->text().toStdString().c_str(), 
							widgets[i]->item(j,1)->text().toStdString().c_str()) )
				{
					// it means, FCU and GCS values are different, so add to query name
					_names.push_back( names[i][j].toStdString() );
					_types.push_back( types[i][j].toStdString() );
					_values.push_back( widgets[i]->item(j,1)->text().toDouble() );
				}
			}
		}
	}
}

void ParameterModule::log( const string msg )
{
	for(unsigned int i = 0; i < modules.size(); i++) 
		modules[i]->log( msg );
}


} // namespace px4_gcs
