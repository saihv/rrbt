#pragma once

#include "rrbt.hpp"

struct Point
{
	double x, y, z;
	Point(double _x, double _y, double _z) {
		this->x = _x;
		this->y = _y;
		this->z = _z;
	}

	Point() {}
	Point& operator = (const Point& a)
	{
		x = a.x;
		y = a.y;
		z = a.z;
		return *this;
	}
};

struct Sample
{
	Point position;
	double yaw;
};


class Utils
{
public:
	Utils()
	{

	}

	Point steerPosition (Point qRand, Point qNear, float eps)
	{
		Point qNew;
		float d = computeDistPts(qNear, qRand);
		if (d >= eps) 
		{
			qNew.x = qNear.x + ((qRand.x - qNear.x) * eps) / d;
			qNew.y = qNear.y + ((qRand.y - qNear.y) * eps) / d;
			qNew.z = qNear.z + ((qRand.z - qNear.z) * eps) / d;
		}
		else 
		{
			qNew = qRand;
		}

		return qNew;
	}

	double steerYaw(double pr, double pn, double eps)
	{
		double pnew;
		if (abs(pr - pn) > eps) {
			if (pr > pn)
				pnew = pn + eps;
			else
				pnew = pn - eps;
		}
		else
			pnew = pr;

		return pnew;
	}

	float computeDistPts(Point p1, Point p2) 
	{
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
	}

	void printNode(Point node)
	{
		std::cout << "Inserted node at coordinates: ";
		std::cout << "(" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
	}
};