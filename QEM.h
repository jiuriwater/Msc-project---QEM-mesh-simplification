#pragma once
#include "libraries/glm/glm/glm.hpp"

#include <iostream>
#include <cassert>
#include <ctime>
#include <map>
#include <set>
#include <queue>
#include <fstream>
#include <chrono>
#include <sstream>
#include <iomanip>

	struct Vertex {

		glm::dvec3 position;

		glm::dmat4 Quadric;

		std::set<int> faceList;

	};

	struct Face {

		int v1;
		int v2;
		int	v3;

		glm::dvec3 normal;
		double d;

	};

	struct Edge {
		int e1;
		int e2;

		double error;
		glm::dvec3 position;

		int time;
		Edge(int e1, int e2, double error, double x, double y, double z, int times)
			: e1(e1), e2(e2), error(error), position(x, y, z), time(times) {}
	};

void simplification(std::string inputModel, std::string outputModel, float Rate, float EPS);

void clearData();