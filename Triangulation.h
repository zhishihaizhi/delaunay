#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <queue>
#include <map>
using namespace std;
using namespace cv;

class CPoint2Df
{
public:
	CPoint2Df(){}
	~CPoint2Df(){}
	CPoint2Df(float tx, float ty){x = tx; y = ty; }
	//int index;
	float x, y;
	//凹凸定点标志
	//bool mark;
};

class CEdge
{
public:
	CEdge(){}
	CEdge(int tstart, int tendn){ start = tstart; endn = tendn; }
	~CEdge(){}
	int start;
	int endn;
};

class CTriangle
{
public:
	CTriangle(){ available = true; }
	CTriangle(int tnodeA, int tnodeB, int tnodeC){ nodeA = tnodeA; nodeB = tnodeB; nodeC = tnodeC; available = true; }
	CTriangle(int tindex, int tnodeA, int tnodeB, int tnodeC){ index = tindex; nodeA = tnodeA; nodeB = tnodeB; nodeC = tnodeC; available = true; }
	int index;
	//三角形三个顶点的索引
	int nodeA, nodeB, nodeC;
	bool available;
};

class TriDiagram
{
public:
	TriDiagram(void);
	~TriDiagram(void);
	void triangulate(vector<CPoint2Df> &vertices, int &xmid, int &ymid, int &dmax);
	void TriDiagram::draw2Image(IplImage *img);
	void boundaryConstraint(IplImage *img, vector<CEdge> &conEdges);
private:
	int getcoincidTri(const CEdge &intersectEdge, const int triIndex);
	bool isCoincidePoint(const CPoint2Df &a, const CPoint2Df &b);
	bool isIntersectEdge(const CPoint2Df &t1, const CPoint2Df &t2, const CPoint2Df &p1, const CPoint2Df &p2);
	bool isCoincideEdge(const CPoint2Df &a, const CPoint2Df &b, const CPoint2Df &c, const CPoint2Df &d);
	void getAid();
	bool isInCircle(CPoint2Df &pt, CTriangle &tri, float &xCenter, float &yCenter, float &radius);
	std::vector<CPoint2Df> mTriVertices;
	std::vector<CTriangle> mTriangles;
	//顶点对应的三角形index
	std::map<int, vector<int>> node2Tri;
};
