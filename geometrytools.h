#pragma once
#include "shapes.h"
#include <iostream>
#include <array>

namespace geo {
	//--------------------------------------------VECTOR MATHS--------------------------------------------

	vec add(vec a, vec b);
	vec sub(vec a, vec b);
	double dot(vec a, vec b);
	vec scale(vec a, double b);
	double magnitude(vec A);
	void printVec(vec A);
	bool compare(vec A, vec B);

	//--------------------------------------------MATRIX MATHS--------------------------------------------

	void printMatrix(std::array<double, 3> M);
	double det2x2(std::array <double, 4> M);
	double det3x3(std::array <double, 9> M);
	std::array<double, 9> cofactorsPlus(std::array<double, 9> M);
	std::array < double, 9 > invert3x3(std::array<double, 9> M);
	std::array <double, 3> multiply3x3X3x1(std::array<double, 9> A, std::array<double, 3> B);

	//------------------------------------------UTILITY FUNCTIONS------------------------------------------

	double getV(vec A, vec B, vec Q);
	vec closestPoint(vec A, vec B, vec Q);
	int support(const convex& A, const vec &d);
	vec minowskyDifference(const convex &A, const convex &B, const vec d);
	bool contains(std::vector<vec> &V, vec v);
	vec epa(const convex& A, const convex& B, simplexVec S);
	int mostPerpendicular(const convex& A, const int v, const vec penetrationNormal);
	bool onLeftOfLine(const vec A, const vec B, const vec Q, bool clockwise);
	vec getNextAndPrevious(int a, int size);
	bool isPointInside(vec p, const convex* A, int startpoint1 = 0, int startpoint2 = 1);
	void putInOrder(int& a, int& b, int size);

	//returns a manifold containing the forces A should exert on B.
	vec getCollisionData(const convex& A, const convex& B, collision& man, line& incline, line& refline);

}