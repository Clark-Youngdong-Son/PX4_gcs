#include "modules/drawing_module.h"

namespace px4_gcs
{

DrawingModule::DrawingModule(QCustomPlot* _widget)
{
	widget = new QCustomPlot;
	widget = _widget;
	
	// grapth setting (temporal)
	widget->addGraph();
	widget->graph(0)->setPen(QPen(QColor(40,110,255)));
	//widget->graph(0)->pen().setWidth(10);
	widget->addGraph();
	widget->graph(1)->setPen(QPen(QColor(255,110,40)));
	//widget->graph(1)->pen().setWidth(10);
	widget->addGraph();
	widget->graph(2)->setPen(QPen(QColor(0,0,0)));

	widget->yAxis->setSubTicks(false);
	widget->xAxis->setRange(0, 4);

	// default setting
	margin = 1.0;
	lim[0] = -4.0;
	lim[1] = 4.0;
	height = lim[1] - lim[0];	
}

DrawingModule::~DrawingModule()
{
	delete widget;
}

void DrawingModule::setMargin(double _margin)
{
	margin = _margin;
}

void DrawingModule::setYLims(double _min, double _max)
{
	lim[0] = _min;
	lim[1] = _max;
	height = lim[1] - lim[0];
	margin = (_max - _min)*0.1;
	widget->yAxis->setRange(lim[0], lim[1]);
}

void DrawingModule::draw(double _time, double _value, int _index)
{
	widget->graph(_index)->addData(_time, _value);
	
	arrangeAxis(_time, _value);
	
	widget->replot();
}

void DrawingModule::arrangeAxis(double _time, double _value)
{
	widget->xAxis->setRange(_time, 4, Qt::AlignRight);
	if(_value < lim[0] + margin)
	{
		lim[0] = _value - margin;
		lim[1] = lim[0] + height;
		widget->yAxis->setRange( _value - margin, _value + height - margin);
	}
	else if(_value > lim[1] - margin)
	{
		lim[1] = _value + margin;
		lim[0] = lim[1] - height;
		widget->yAxis->setRange(_value - height + margin, _value + margin);
	}
}

void DrawingModule::log( const std::string msg )
{
	// do nothing
}

} // namespace px4_gcs
