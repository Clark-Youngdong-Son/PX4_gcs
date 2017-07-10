#ifndef px4_gcs_DRAWING_MODULE_H
#define px4_gcs_DRAWING_MODULE_H

#include <qcustomplot.h>
#include <string>

#include "modules/module.h"

namespace px4_gcs
{

class DrawingModule : public Module
{
	public:
		DrawingModule(QCustomPlot* _widget);
		~DrawingModule();

		void setMargin(double _margin);
		void setYLims(double _min, double _max);

		void draw(double _time, double _value, int _index);
		void log( const std::string );

	private:
		QCustomPlot* widget;
		void arrangeAxis(double _time, double _value);
		
		double margin;
		double height;
		double lim[2]; // lim[0] : min, lim[1] : max
};

} // namespace px4_gcs

#endif
