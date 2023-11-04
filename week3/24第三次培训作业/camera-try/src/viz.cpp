#include <opencv2/viz.hpp>
#include <iostream>
using namespace cv;
using namespace std;
 
int main()
{
	viz::Viz3d myWindow("Viz Demo");
	myWindow.spin();
	cout << "First event loop is over" << endl;
	viz::Viz3d sameWindow = viz::getWindowByName("Viz Demo");
	sameWindow.spin();
	cout << "Second event loop is over" << endl;
	sameWindow.spinOnce(1, true);
	while (!sameWindow.wasStopped())
	{
		sameWindow.spinOnce(1, true);
	}
	cout << "Last event loop is over" << endl;
	return 0;
}