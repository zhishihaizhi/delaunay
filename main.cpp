#include "Triangulation.h"

void main(int argc, char* argv[])

{
	IplImage* white = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);
	cvZero(white);
	cvAddS(white,cvScalar(255,255,255),white);
	cout << "doing triangulation..." << endl;
	vector<Vec3i> tri;

	vector<CPoint2Df> vec_points;
	for (int i = 0; i < 10; i++)
	{
		float x = (float)(rand() % (400 - 10))+250;
		float y = (float)(rand() % (400 - 10))+250;
		CPoint2Df fp = CPoint2Df(x, y);
		vec_points.push_back(fp);
	}
	int xmid, ymid, dma;
	TriDiagram tria;
	tria.triangulate(vec_points, xmid, ymid, dma);
	tria.draw2Image(white);
	cvShowImage("white1", white);
	vector<CEdge> edges;
	edges.push_back(CEdge(2, 3));
	tria.boundaryConstraint(white, edges);
	cvWaitKey();
}
