#include "Triangulation.h"
#include <algorithm>
TriDiagram::TriDiagram(void)
{
}

TriDiagram::~TriDiagram(void)
{
}

bool cmpX(const CPoint2Df &p1, const CPoint2Df &p2)
{
	return p1.x < p2.x;
}

bool TriDiagram::isInCircle(CPoint2Df &pt, CTriangle &tri, float &xCenter, float &yCenter, float &radius)
{
	if (abs(mTriVertices[tri.nodeA].y - mTriVertices[tri.nodeB].y) < 1 && abs(mTriVertices[tri.nodeA].y - mTriVertices[tri.nodeC].y) < 1)
		return false;

	int x1 = mTriVertices[tri.nodeA].x;
	int y1 = mTriVertices[tri.nodeA].y;
	int x2 = mTriVertices[tri.nodeB].x;
	int y2 = mTriVertices[tri.nodeB].y;
	int x3 = mTriVertices[tri.nodeC].x;
	int y3 = mTriVertices[tri.nodeC].y;

	float m1 = 0.0;
	float m2 = 0.0;
	float mx1 = 0.0;
	float mx2 = 0.0;
	float my1 = 0.0;
	float my2 = 0.0;
	float dx = 0.0;
	float dy = 0.0;
	float rsqr = 0.0;
	float drsqr = 0.0;

	if (abs(y2 - y1) < 5)
	{
		m2 = -(x3 - x2) / (float)(y3 - y2);
		mx2 = (x2 + x3) / 2.0;
		my2 = (y2 + y3) / 2.0;
		xCenter = (x2 + x1) / 2.0;
		yCenter = m2 * (xCenter - mx2) + my2;
	}
	else if (abs(y3 - y2) < 5)
	{
		m1 = -(x2 - x1) / (float)(y2 - y1);
		mx1 = (x1 + x2) / 2.0;
		my1 = (y1 + y2) / 2.0;
		xCenter = (x3 + x2) / 2.0;
		yCenter = m1 * (xCenter - mx1) + my1;
	}
	else
	{
		m1 = -(float)((x2 - x1) / (float)(y2 - y1));//中垂线斜率
		m2 = -(float)((x3 - x2) / (float)(y3 - y2));
		mx1 = (float)(x1 + x2) / 2.0;//中点
		mx2 = (float)(x2 + x3) / 2.0;
		my1 = (float)(y1 + y2) / 2.0;
		my2 = (float)(y2 + y3) / 2.0;
		xCenter = (float)((m1*mx1 - m2*mx2 + my2 - my1) / (m1 - m2));//中垂线交点
		yCenter = (float)(m1*(xCenter - mx1) + my1);

	}

	dx = x2 - xCenter;
	dy = y2 - yCenter;
	rsqr = dx * dx + dy * dy;
	radius = sqrt(rsqr);
	dx = pt.x - xCenter;
	dy = pt.y - yCenter;
	drsqr = dx * dx + dy * dy;

	if (drsqr <= rsqr)
		return true;
	else
		return false;
}

void TriDiagram::triangulate(vector<CPoint2Df> &vertices, int &xmid, int &ymid ,int &dmax)
{
	int vertnum = vertices.size();
	int trinum = 0;
	mTriVertices.resize(vertnum);
	std::copy(vertices.begin(), vertices.end(), mTriVertices.begin());

	int xmin, xmax, ymin, ymax;

	float xCenter = 0, yCenter = 0, radius = 0;
	bool inCircle = false;

	//找到xy的最大最小值
	xmin = mTriVertices[0].x;
	ymin = mTriVertices[0].y;
	xmax = xmin;
	ymax = ymin;
	for (int i = 0; i < vertnum; i++)
	{
		if (mTriVertices[i].x < xmin) xmin = mTriVertices[i].x;
		if (mTriVertices[i].x > xmax) xmax = mTriVertices[i].x;
		if (mTriVertices[i].y < ymin) ymin = mTriVertices[i].y;
		if (mTriVertices[i].y > ymax) ymax = mTriVertices[i].y;
	}

	dmax = xmax - xmin > ymax - ymin ? xmax - xmin : ymax - ymin;

	xmid = (xmax + xmin) / 2;
	ymid = (ymax + ymin) / 2;

	sort(mTriVertices.begin(), mTriVertices.end(), cmpX);
	mTriVertices.push_back(CPoint2Df(xmin - xmid-10, ymin-10));
	mTriVertices.push_back(CPoint2Df(xmid, 2*ymax));
	mTriVertices.push_back(CPoint2Df(xmax + xmid+10, ymin-10));
	trinum = 1;
	queue<CTriangle> tempTriangles;
	tempTriangles.push(CTriangle(vertnum, vertnum + 1, vertnum + 2));
	vector<CEdge> edgeBuffer;
	for (int i = 0; i < vertnum; i++)
	{
 		edgeBuffer.clear();
		int temptrinum = trinum;
		for (int j = 0; j < trinum; j++)
		{
			inCircle = isInCircle(mTriVertices[i], tempTriangles.front(), xCenter, yCenter, radius);

			if (inCircle)
			{
				edgeBuffer.push_back(CEdge(tempTriangles.front().nodeA, tempTriangles.front().nodeB));
				edgeBuffer.push_back(CEdge(tempTriangles.front().nodeB, tempTriangles.front().nodeC));
				edgeBuffer.push_back(CEdge(tempTriangles.front().nodeC, tempTriangles.front().nodeA));
				tempTriangles.pop();
				temptrinum--;
			}
			//该点在三角形外接圆右侧，则表示左侧三角形为Delaunay三角形，将该三角形保存至triangles中
			else if (xCenter + radius < mTriVertices[i].x)
			{
				mTriangles.push_back(tempTriangles.front());
				temptrinum--;
				tempTriangles.pop();
			}
			//该点在三角形外接圆外侧，为不确定三角形，所以跳过
			else
			{
				tempTriangles.push(tempTriangles.front());
				tempTriangles.pop();
			}
		}
		trinum = temptrinum;
		if (!edgeBuffer.empty())
		{
			//如果有一条边被加入两次，那么它被删除，因为位于两个三角形的外接圆内
			for (int j = 0; j < edgeBuffer.size() - 1; j++)
			{
				for (int k = j + 1; k < edgeBuffer.size(); k++)
				{
					if (((edgeBuffer[j].start == edgeBuffer[k].start) && (edgeBuffer[j].endn == edgeBuffer[k].endn)) ||
						((edgeBuffer[j].endn == edgeBuffer[k].start) && (edgeBuffer[j].start == edgeBuffer[k].endn)))
					{
						edgeBuffer[j].start = -1;
						edgeBuffer[j].endn = -1;
						edgeBuffer[k].start = -1;
						edgeBuffer[k].endn = -1;
					}
				}
			}
			//添加新三角形
			for (int j = 0; j < edgeBuffer.size(); j++)//围绕该点没有被删的边
			{
				if (edgeBuffer[j].start >= 0 && edgeBuffer[j].endn >= 0)
				{
					tempTriangles.push(CTriangle(edgeBuffer[j].start, edgeBuffer[j].endn, i));
					trinum += 1;
				}
			}

		}
	}
	while (!tempTriangles.empty())
	{
		mTriangles.push_back(tempTriangles.front());
		tempTriangles.pop();
	}
}

void TriDiagram::getAid()
{
	for (int i = 0; i < mTriangles.size(); i++)
	{
		mTriangles[i].index = i;
		mTriangles[i].available = true;
		node2Tri[mTriangles[i].nodeA].push_back(i);
		node2Tri[mTriangles[i].nodeB].push_back(i);
		node2Tri[mTriangles[i].nodeC].push_back(i);
	}
}

double determinant(double v1, double v2, double v3, double v4)  // 行列式  
{
	return (v1*v4 - v2*v3);
}

bool TriDiagram::isIntersectEdge(const CPoint2Df &a, const CPoint2Df &b, const CPoint2Df &c, const CPoint2Df &d)
{
	if (a.x == c.x && a.y == c.y || b.x == d.x && b.y == d.y || a.x == d.x && a.y == d.y || b.x == c.x && b.y == c.y)
		return false;
	double delta = determinant(b.x - a.x, c.x - d.x, b.y - a.y, c.y - d.y);
	if (delta <= (1e-6) && delta >= -(1e-6))  // delta=0，表示两线段重合或平行  
	{
		return false;
	}
	double namenda = determinant(c.x - a.x, c.x - d.x, c.y - a.y, c.y - d.y) / delta;
	if (namenda > 1 || namenda < 0)
	{
		return false;
	}
	double miu = determinant(b.x - a.x, c.x - a.x, b.y - a.y, c.y - a.y) / delta;
	if (miu > 1 || miu < 0)
	{
		return false;
	}
	return true;
}

bool TriDiagram::isCoincideEdge(const CPoint2Df &a, const CPoint2Df &b, const CPoint2Df &c, const CPoint2Df &d)
{
	if (a.x == c.x && a.y == c.y && b.x == d.x && b.y == d.y || a.x == d.x && a.y == d.y && b.x == c.x && b.y == c.y)
		return true;
	return false;
}

bool TriDiagram::isCoincidePoint(const CPoint2Df &a, const CPoint2Df &b)
{
	if (a.x == b.x && a.y == b.y)
		return true;
	return false;
}


int TriDiagram::getcoincidTri(const CEdge &intersectEdge, const int triIndex)
{
	int startnode = intersectEdge.start;
	int endnode = intersectEdge.endn;
	vector<int> startTris = node2Tri[startnode];
	vector<int> endTris = node2Tri[endnode];
	for (auto st : startTris)
		for (auto et : endTris)
			if (st == et&&st != triIndex)
				return st;
	return -1;
}
void TriDiagram::boundaryConstraint(IplImage *img, vector<CEdge> &conEdges)
{
	getAid();
	for (auto edge : conEdges)
	{
		bool hasIntersect = false;
		bool coincide = false;
		int startnode = edge.start;
		int endnode = edge.endn;
		IplImage *newimg = cvCloneImage(img);
		cvLine(newimg, cv::Point(mTriVertices[startnode].x, mTriVertices[startnode].y), cv::Point(mTriVertices[endnode].x, mTriVertices[endnode].y), CV_RGB(255, 0, 0), 1, CV_AA);
		cvShowImage("img", newimg);
		cvWaitKey();
		vector<int> triIndexs = node2Tri[startnode];
		vector<int> intersectTirs;
		/*for (auto tri : triIndexs)
		{
			int nodeA = mTriangles[tri].nodeA;
			int nodeB = mTriangles[tri].nodeB;
			int nodeC = mTriangles[tri].nodeC;
			cvLine(newimg, cv::Point(mTriVertices[nodeA].x, mTriVertices[nodeA].y), cv::Point(mTriVertices[nodeB].x, mTriVertices[nodeB].y), CV_RGB(255, 255, 0), 1, CV_AA);
			cvLine(newimg, cv::Point(mTriVertices[nodeA].x, mTriVertices[nodeA].y), cv::Point(mTriVertices[nodeC].x, mTriVertices[nodeC].y), CV_RGB(255, 255, 0), 1, CV_AA);
			cvLine(newimg, cv::Point(mTriVertices[nodeB].x, mTriVertices[nodeB].y), cv::Point(mTriVertices[nodeC].x, mTriVertices[nodeC].y), CV_RGB(255, 255, 0), 1, CV_AA);

			cvShowImage("img1", newimg);
			cvWaitKey();


		}*/
		vector<CEdge> aroundEdge;
		CEdge intersectEdge;
		//找到第一个三角形
		for (int i = 0; i < triIndexs.size();i++)
		{
			//如果和已有边重合，不需要进行其他操作
			int nodeA = mTriangles[triIndexs[i]].nodeA;
			int nodeB = mTriangles[triIndexs[i]].nodeB;
			int nodeC = mTriangles[triIndexs[i]].nodeC;
			IplImage *newimg1 = cvCloneImage(newimg);
			cvLine(newimg1, cv::Point(mTriVertices[nodeA].x, mTriVertices[nodeA].y), cv::Point(mTriVertices[nodeB].x, mTriVertices[nodeB].y), CV_RGB(255, 255, 0), 1, CV_AA);
			cvLine(newimg1, cv::Point(mTriVertices[nodeA].x, mTriVertices[nodeA].y), cv::Point(mTriVertices[nodeC].x, mTriVertices[nodeC].y), CV_RGB(255, 255, 0), 1, CV_AA);
			cvLine(newimg1, cv::Point(mTriVertices[nodeB].x, mTriVertices[nodeB].y), cv::Point(mTriVertices[nodeC].x, mTriVertices[nodeC].y), CV_RGB(255, 255, 0), 1, CV_AA);

			cvShowImage("img1", newimg1);
			cvWaitKey();


			if (isCoincideEdge(mTriVertices[nodeA], mTriVertices[nodeB], mTriVertices[startnode], mTriVertices[endnode])
				|| isCoincideEdge(mTriVertices[nodeB], mTriVertices[nodeC], mTriVertices[startnode], mTriVertices[endnode])
				|| isCoincideEdge(mTriVertices[nodeA], mTriVertices[nodeC], mTriVertices[startnode], mTriVertices[endnode]))
			{
				coincide = true;
				break;
			}
			bool ABIntersect = isIntersectEdge(mTriVertices[nodeA], mTriVertices[nodeB], mTriVertices[startnode], mTriVertices[endnode]);
			bool CBIntersect = isIntersectEdge(mTriVertices[nodeC], mTriVertices[nodeB], mTriVertices[startnode], mTriVertices[endnode]);
  			bool ACIntersect = isIntersectEdge(mTriVertices[nodeA], mTriVertices[nodeC], mTriVertices[startnode], mTriVertices[endnode]);
			
			if (ABIntersect||CBIntersect||ACIntersect)
			{
				hasIntersect = true;
				intersectTirs.push_back(triIndexs[i]);
				if (!ABIntersect)
					aroundEdge.push_back(CEdge(nodeA, nodeB));
				else
					intersectEdge=CEdge(nodeA, nodeB);
				if (!CBIntersect)
					aroundEdge.push_back(CEdge(nodeB, nodeC));
				else
					intersectEdge=CEdge(nodeB, nodeC);
				if (!ACIntersect)
					aroundEdge.push_back(CEdge(nodeA, nodeC));
				else
					intersectEdge=CEdge(nodeA, nodeC);
				break;
			}

		}
		//如果和已有边重合，不需要进行其他操作
		if (coincide)
			break;
		//继续查找重合三角形
		int rd = 0;
		while (true)
		{
			int i = getcoincidTri(intersectEdge, intersectTirs[rd]);
			int nodeA = mTriangles[i].nodeA;
			int nodeB = mTriangles[i].nodeB;
			int nodeC = mTriangles[i].nodeC;
			int L1S, L1E, L2S, L2E;
			if ((nodeA == intersectEdge.start&&nodeB == intersectEdge.endn || nodeB == intersectEdge.start&&nodeA == intersectEdge.endn))
			{
				L1S = nodeC;
				L1E = nodeB;
				L2S = nodeC;
				L2E = nodeA;
			}
			else if ((nodeC == intersectEdge.start&&nodeB == intersectEdge.endn || nodeB == intersectEdge.start&&nodeC == intersectEdge.endn))
			{
				L1S = nodeA;
				L1E = nodeB;
				L2S = nodeC;
				L2E = nodeA;
			}
			else if ((nodeC == intersectEdge.start&&nodeA == intersectEdge.endn || nodeA == intersectEdge.start&&nodeC == intersectEdge.endn))
			{
				L1S = nodeB;
				L1E = nodeA;
				L2S = nodeB;
				L2E = nodeC;
			}
			else
			{
				printf("wrong\n");
			}

			if (endnode == nodeA || endnode == nodeB || endnode == nodeC)
			{
				intersectTirs.push_back(mTriangles[i].index);
				aroundEdge.push_back(CEdge(L1S, L1E));
				aroundEdge.push_back(CEdge(L2S, L2E));
				break;
			}

			IplImage *newimg1 = cvCloneImage(newimg);
			cvLine(newimg1, cv::Point(mTriVertices[L1S].x, mTriVertices[L1S].y), cv::Point(mTriVertices[L1E].x, mTriVertices[L1E].y), CV_RGB(255, 255, 0), 1, CV_AA);
			cvLine(newimg1, cv::Point(mTriVertices[L2S].x, mTriVertices[L2S].y), cv::Point(mTriVertices[L2E].x, mTriVertices[L2E].y), CV_RGB(255, 255, 0), 1, CV_AA);
			
			cvShowImage("img1", newimg1);
			cvWaitKey();

			if (isCoincideEdge(mTriVertices[L1S], mTriVertices[L1E], mTriVertices[startnode], mTriVertices[endnode])
				|| isCoincideEdge(mTriVertices[L2S], mTriVertices[L2E], mTriVertices[startnode], mTriVertices[endnode]))
			{
				coincide = true;
				break;
			}
			bool L1Intersect = false, L2Intersect = false;
			L1Intersect = isIntersectEdge(mTriVertices[L1S], mTriVertices[L1E], mTriVertices[startnode], mTriVertices[endnode]);
			L2Intersect = isIntersectEdge(mTriVertices[L2S], mTriVertices[L2E], mTriVertices[startnode], mTriVertices[endnode]);
			
			if (L1Intersect || L2Intersect)
			{
				hasIntersect = true;
				intersectTirs.push_back(mTriangles[i].index);
				if (!L1Intersect)
					aroundEdge.push_back(CEdge(L1S, L1E));
				else
					intersectEdge = CEdge(L1S, L1E);
				if (!L2Intersect)
					aroundEdge.push_back(CEdge(L2S, L2E));
				else
					intersectEdge = CEdge(L2S, L2E);
				rd += 1;
			}
		}
		if (!coincide)
		{
			CPoint2Df pt = CPoint2Df((mTriVertices[startnode].x + mTriVertices[endnode].x) / 2.0, (mTriVertices[startnode].y + mTriVertices[endnode].y) / 2);
			mTriVertices.push_back(pt);
			for (auto edgen : aroundEdge)
			{
				mTriangles.push_back(CTriangle(mTriVertices.size()-1,mTriVertices.size()-1, edgen.start, edgen.endn));
			}
			for (auto tri : intersectTirs)
			{
				mTriangles[tri].available = false;
			}
		}
	}
	IplImage* white2 = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);
	cvZero(white2);
	cvAddS(white2, cvScalar(255, 255, 255), white2);
	draw2Image(white2);
	cvShowImage("white2", white2);
	cvWaitKey();
}

void TriDiagram::draw2Image(IplImage *img)
{
	for (int i = 0; i < mTriangles.size(); i++)
	{
		if (!mTriangles[i].available)
			continue;
		CPoint2Df p1 = mTriVertices[mTriangles[i].nodeA];
		CPoint2Df p2 = mTriVertices[mTriangles[i].nodeB];
		CPoint2Df p3 = mTriVertices[mTriangles[i].nodeC];

		cvLine(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), CV_RGB(0, 0, 0), 1, CV_AA);
		cvLine(img, cv::Point(p2.x, p2.y), cv::Point(p3.x, p3.y), CV_RGB(0, 0, 0), 1, CV_AA);
		cvLine(img, cv::Point(p3.x, p3.y), cv::Point(p1.x, p1.y), CV_RGB(0, 0, 0), 1, CV_AA);
	}
}
