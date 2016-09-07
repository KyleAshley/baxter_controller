#include <stdio.h>
#include <vector>
#include <numeric>
#include <math.h>

using namespace std;

class slidingWindow
{
public:
	slidingWindow();
	slidingWindow(int max);
	~slidingWindow();

	void add(double val);
	void resize(int size);
	void clear();

	double average();
	double std_dev();

private:
	vector<double> w;
	int max_size;
};


