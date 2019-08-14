#pragma once
#include "shapes.h"
#include <iostream>
#include <array>

namespace geo {
	//--------------------------------------------VECTOR MATHS--------------------------------------------

	double dot(vec a, vec b);
	double magnitude(vec A);
	void printVec(vec A);

	//--------------------------------------------MATRIX MATHS--------------------------------------------

	void printMatrix(std::array<double, 3> M);
	double det2x2(std::array <double, 4> M);
	double det3x3(std::array <double, 9> M);
	std::array<double, 9> cofactorsPlus(std::array<double, 9> M);
	std::array < double, 9 > invert3x3(std::array<double, 9> M);
	std::array <double, 3> multiply3x3X3x1(std::array<double, 9> A, std::array<double, 3> B);
	

	template <class T> class Matrix2D {
		//		|[0], [1]|		//
		//		|[2], [3]|		//
		std::array <T, 4> fields;
		
	public:
		Matrix2D(T a, T b, T c, T d) {
			this->fields = { a,b,c,d };
		}

		Matrix2D(T floats[4]) {
			this->fields = { floats[0],floats[1],floats[2],floats[3] };
		}

		Matrix2D operator*(Matrix2D mat) {
			float values[4];
			values[0] = this->fields[0] * mat[0] + this->fields[1] * mat[2];
			values[1] = this->fields[0] * mat[1] + this->fields[1] * mat[3];
			values[2] = this->fields[2] * mat[0] + this->fields[3] * mat[2];
			values[3] = this->fields[2] * mat[1] + this->fields[3] * mat[3];
			return Matrix2D(values);
		}

		Matrix2D operator *= (Matrix2D mat) {
			this->fields = (this->operator*(mat)).fields;
		}

		geo::vec operator * (geo::vec vector) {
			return { this->fields[0] * vector.x + this->fields[1] * vector.y, this->fields[2] * vector.x + this->fields[3] * vector.y };
		}

		float& operator[] (size_t position) {
			return this->fields[position];
		}

	};

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