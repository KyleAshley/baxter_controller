#include "util.h"


slidingWindow::slidingWindow()
{

}

slidingWindow::slidingWindow(int max)
{
	this->max_size = max;
}

slidingWindow::~slidingWindow()
{

}

void slidingWindow::add(double val)
{
	if(w.size() >= this->max_size)
		w.erase(w.begin());

	w.push_back(val);
}

void slidingWindow::clear()
{
	w.clear();
}

void slidingWindow::resize(int size)
{
	this->clear();
	this->max_size = size;
}

// STATISTICS
double slidingWindow::average()
{
	if (w.size() > 0)
	{
		double sum = 0.0;
		for(int i = 0; i < w.size(); i++)
			sum += w[i];
		if (sum > 0)
			return sum / (double) w.size();
		else 
			return 0.0;
	}
}

double slidingWindow::std_dev()
{
	if(w.size() > 0)
	{
		double sum = 0.0;
		for(int i = 0; i < w.size(); i++)
			sum += w[i];
		double mean = sum / (double) w.size();
		double sq_sum = std::inner_product(w.begin(), w.end(), w.begin(), 0.0);
		return sqrt(sq_sum / w.size() - mean * mean);
	}
}