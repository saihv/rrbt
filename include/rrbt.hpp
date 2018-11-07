#pragma once
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

#include "boost/graph/graph_traits.hpp"
#include "boost/graph/adjacency_list.hpp"

#include "rrbt_params.hpp"
#include "rrbt_utils.hpp"
#include "kdtree.h"

#include <Eigen/Dense>
#include <Eigen/Core>

using namespace boost;

typedef int nodeType;

#if DIM == 2
	typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat;
#elif DIM == 3
	typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Mat;
#endif


class RRBT {
public:
	struct Node {
		Point coords;
		double yaw;
		int index, parent;
		double cost, sigma, J;
		Mat cov;
	};

	Utils utils;

	typedef adjacency_list<vecS, vecS, undirectedS, RRBT::Node> RRG;
	typedef RRG::vertex_descriptor vertex_t;
	typedef RRG::edge_descriptor edge_t;

	RRBT::RRG Graph;

	void initTree(int &nIter)
	{
		kd_rrbt = kd_create(3);
		nodeNum.reserve(nIter);
		nodeParents.reserve(nIter);
	}

	vertex_t addNode(Point point, double yaw, int idx, int parent, double cost, double sigma, double J, Mat cov) {
		vertex_t v = add_vertex(Graph);

		Graph[v].coords = point;
		Graph[v].yaw = yaw;
		Graph[v].index = idx;
		Graph[v].cost = cost;
		Graph[v].sigma = sigma;
		Graph[v].parent = parent;
		Graph[v].J = J;
		Graph[v].cov = cov;

		nodeNum.push_back(idx);
		if (kd_insert3(static_cast<kdtree*>(kd_rrbt), point.x, point.y, point.z, &nodeNum[idx]) != 0)
			std::cerr << "Unable to insert node!" << std::endl;

		return v;
	}

	void modifyNode(int nodeIdx, int parent, float cost, float sigma) {
		Graph[nodeIdx].parent = parent;
		Graph[nodeIdx].cost = cost;
		Graph[nodeIdx].sigma = sigma;
		Graph[nodeIdx].index = nodeIdx;
	}

	float computeDistNodes(Point p1, Point p2) {
		float dist = 0;
		dist = utils.computeDistPts(p1, p2);

		return dist;
	}

	float computeCov(Point point, Point beacon) {
		
	}

	int nearestNeighbor(Point &q)
	{
		queryPt[0] = q.x;
		queryPt[1] = q.y;
		queryPt[2] = q.z;

		knnResults = kd_nearest(static_cast<kdtree*>(kd_rrbt), queryPt);
		resIdx = (int*)kd_res_item(knnResults, resultPt);

		int idx = *resIdx;

		kd_res_free(knnResults);
		return idx;
	}

	void knnSearch(Point &q, int &radius, std::vector <int> &results)
	{
		double queryPt[3] = { q.x, q.y, q.z };
		
		knnResults = kd_nearest_range(static_cast<kdtree*>(kd_rrbt), queryPt, radius);
		results.clear();

		while (!kd_res_end(knnResults)) {
			//std::cout << "Found " << kd_res_size(knnResults) << " results" << std::endl;
			resIdx = (int*)kd_res_item(knnResults, resultPt);
			results.push_back(*resIdx);
			//std::cout << "Nearest neighbor data: " << *pch << std::endl;

			kd_res_next(knnResults);
		}
		kd_res_free(knnResults);
	}

	bool onPath(RRBT &rrbt, int node, int goal)
	{
		int evalNode;
		evalNode = rrbt.Graph[goal].parent;

		if (evalNode == node)
			return true;

		while (evalNode != -1) {
			if (evalNode == node)
				return true;
			else
				evalNode = rrbt.Graph[evalNode].parent;
		}
		return false;
	}

	void identifyParents(int idx)
	{
		int node;
		node = Graph[idx].parent;

		std::fill(nodeParents.begin(), nodeParents.end(), 0);

		while (node != -1) {
			nodeParents[node] = 1;
			node = Graph[node].parent;
		}
	}

	int BALL_SIZE = 20;

	std::vector <bool> nodeParents;

private:
	void *kd_rrbt;
	double *queryPt = new double[3];
	double *resultPt = new double[3];

	int *resIdx;
	struct kdres *knnResults;

	std::vector <int> nodeNum;


	/*
	int nearestNeighbor(RRBT &rrbt, Point q)
	{
		double minDist = 10000, dist = 0;
		int nn = 0;
		for (unsigned int i = 0; i < num_vertices(rrbt.Graph); ++i) {
			dist = rrbt.computeDistNodes(q, rrbt.Graph[i].coords);
			if (dist < minDist && dist != 0) {
				minDist = dist;
				nn = i;
			}
		}
		return nn;
	}

	std::vector <int> knnSearch(RRBT &rrbt, Point q, int radius)
	{
		std::vector <int> neighbors;
		double dist = 0;
		for (unsigned int i = 0; i < num_vertices(rrbt.Graph); ++i) {
			dist = rrbt.computeDistNodes(q, rrbt.Graph[i].coords);
			if (dist <= radius && dist != 0) {
				neighbors.push_back(i);
			}
		}
		return neighbors;
	}
	*/
};
