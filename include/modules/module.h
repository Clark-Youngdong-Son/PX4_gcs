#ifndef px4_gcs_MODULE_H
#define px4_gcs_MODULE_H

#include <string>
#include <vector>

namespace px4_gcs
{

class Module
{
	public:
		explicit Module(){};
		virtual ~Module(){};
		
		virtual void log( const std::string ){};
		
		void add( Module* _module ){ modules.push_back( _module ); }

	protected:
		std::vector<Module*> modules;
};

} // namespace px4_gcs

#endif
