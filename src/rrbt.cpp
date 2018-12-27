#include "stdafx.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <queue>
#include "rrbt.hpp"
#include "rrbt_utils.hpp"
#include "rrbt_params.hpp"
#include "belief.hpp"
#include "vision.hpp"

#include <chrono>

using namespace boost;
using namespace std::chrono;

int main(int argc, char *argv[])
{
	srand(time(NULL));
	RRBT rrbt;

	int nIter = 500;

	rrbt.initTree(nIter);

	Belief belief;
	Vision vision;

	Sample start, goal, qRand, qNear, qNew;
	int nearIdx = 0;

	start.position.x = 0;
	start.position.y = 0;
	start.position.z = 0;
	start.yaw = 0;

	goal.position.x = 20;
	goal.position.y = 0;
	goal.position.z = 0;
	goal.yaw = 0;

	Point beacon;

	beacon.x = 0;
	beacon.y = 100;
	beacon.z = 0;

	std::queue <int> nodeQueue;

	RRBT::vertex_t startNode = rrbt.addNode(start.position, start.yaw, 0, -1, 0, 0, 100, 5*Eigen::MatrixXd::Identity(DIM, DIM));
	int J = 100;

	vision.readMapData();
	
	int j = 1;
	Sample sample;

	Mat thresh = 0.05 * Eigen::MatrixXd::Identity(DIM, DIM);

	while (j < nIter)
	{
		//std::cout << j << std::endl;
		qRand.position.x = std::rand() % 100;
		qRand.position.y = std::rand() % 100;
		qRand.position.z = std::rand() % 100;

		double psiRand = ((double)rand()) / RAND_MAX * 360.0 - 180.0;
		j++;

		nearIdx = rrbt.Graph[rrbt.nearestNeighbor(qRand.position)].index;

		qNear.position = rrbt.Graph[nearIdx].coords;
		qNew.position = rrbt.utils.steerPosition(qRand.position, qNear.position, EPS_POS);
		double psiNew = rrbt.utils.steerYaw(qRand.yaw, qNear.yaw, EPS_YAW);

		int newIdx = rrbt.Graph.m_vertices.size();

		// double newSigma = rrbt.computeDistNodes(qNew.position, beacon);

		double newSigma = vision.evaluateSample(qNew.position, psiNew);

		std::cout << "sigma is " << newSigma << std::endl;
		double newCost = rrbt.Graph[nearIdx].cost + rrbt.computeDistNodes(qNew.position, qNear.position);
		RRBT::vertex_t current = rrbt.addNode(qNew.position, psiNew, newIdx, nearIdx, newCost, newSigma, 0, Eigen::MatrixXd::Identity(DIM, DIM));

		belief.propagateBelief(rrbt.Graph[nearIdx].cov, qNear.position, qNew.position, newSigma, rrbt.Graph[newIdx].cov);
		rrbt.Graph[newIdx].J = W_DIST * newCost + W_COV * rrbt.Graph[newIdx].cov.trace();
		nodeQueue.push(newIdx);

		while (nodeQueue.size() > 0)
		{
			size_t cIdx = nodeQueue.front();
			nodeQueue.pop();

			std::vector<int> knn;
			rrbt.knnSearch(rrbt.Graph[cIdx].coords, rrbt.BALL_SIZE, knn);

			rrbt.identifyParents(cIdx);

			for (size_t nctr = 0; nctr < knn.size(); nctr++) {
				if (rrbt.nodeParents[knn[nctr]] != 1 && cIdx != knn[nctr] && cIdx != rrbt.Graph[knn[nctr]].parent) {
					int nIdx = knn[nctr];

					Mat newCov;
					belief.propagateBelief(rrbt.Graph[cIdx].cov, rrbt.Graph[cIdx].coords, rrbt.Graph[nIdx].coords, rrbt.Graph[nIdx].sigma, newCov);
					double travelCost = rrbt.Graph[cIdx].cost + rrbt.computeDistNodes(rrbt.Graph[cIdx].coords, rrbt.Graph[nIdx].coords);

					double newCost = W_COV * (newCov + thresh).trace() + W_DIST * travelCost;
					double presentCost = rrbt.Graph[nIdx].J;

					if (newCost < presentCost) {
						rrbt.Graph[nIdx].parent = cIdx;
						rrbt.Graph[nIdx].cost = travelCost;
						rrbt.Graph[nIdx].cov = newCov;
						rrbt.Graph[nIdx].J = newCost;
						nodeQueue.push(nIdx);
					}
				}
			}
		}
	}


	nearIdx = rrbt.Graph[rrbt.nearestNeighbor(goal.position)].index;
	int parent = rrbt.Graph[nearIdx].parent;

	std::cout << "PATH TO GOAL: " << std::endl;
	while (parent != 0)
	{
		std::cout << "Node : (" << rrbt.Graph[parent].coords.x << ", " << rrbt.Graph[parent].coords.y << ", " << rrbt.Graph[parent].coords.z << ")" << std::endl;
		parent = rrbt.Graph[parent].parent;
	}

	getchar();
	return 0;
}