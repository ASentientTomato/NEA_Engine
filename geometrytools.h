#pragma once
#include "shapes.h"
#include <iostream>
#include <array>

namespace geo {
	//--------------------------------------------VECTOR MATHS--------------------------------------------

	inline double dot(vec a, vec b);
	inline double magnitude(vec A);
	void printVec(vec A);

	//--------------------------------------------MATRIX MATHS--------------------------------------------

	void printMatrix(std::array<double, 3> M);
	double det2x2(std::array <double, 4> M);
	double det3x3(std::array <double, 9> M);
	std::array<double, 9> cofactorsPlus(std::array<double, 9> M);
	std::array < double, 9 > invert3x3(std::array<double, 9> M);
	std::array <double, 3> multiply3x3X3x1(std::array<double, 9> A, std::array<double, 3> B);
	

	/*template <class T> class Matrix2D {
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
			T values[4];
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

	*/
	template <class T> class Matrix3D {

		//		{0,1,2}
		//		{3,4,5}
		//		{6,7,8}

		//TODO: this can be made more efficient since we don't actually need the bottom row.
	public:
		std::array<std::array<T, 3>, 3> fields;

		Matrix3D() {};

		Matrix3D(T floats[9]) {
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					fields[i][j] = floats[i * 3 + j];
				}
			}
		}
		Matrix3D(std::array<std::array<T,3>,3> floats) {
			this->fields = floats;
		}

		Matrix3D operator*(Matrix3D mat) {
			std::array<std::array<T, 3>, 3> result = { {0} };
			
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						result[i][j] += this->fields[i][k] * mat.fields[k][j];
					}
				}
			}
			result[2][2] = 1;
			return Matrix3D(result);
		}

		void operator *= (Matrix3D mat) {
			this->fields = (this->operator*(mat)).fields;
		}


		void print() {
			for (std::array<float, 3> i : this->fields) {
				for (int j = 0; j < 3; j++) {
					std::cout << i[j] << " ";
				}
				std::cout << std::endl;
			}
		}

		float& operator[] (int position) {
			return this->fields[position];
		}

		//ignoring bottom row...
		geo::vec operator*(geo::vec vector) {
			geo::vec vecc;
			vecc.x = (this->fields[0][0] * vector.x + this->fields[0][1] * vector.y + this->fields[0][2]);
			vecc.y = (this->fields[1][0] * vector.x + this->fields[1][1] * vector.y + this->fields[1][2]);
			return vecc;
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