#pragma once
#include "shapes.h"
#include <iostream>
#include <array>

namespace geo {
	//--------------------------------------------VECTOR MATHS--------------------------------------------

	//calculates the dot product of two vectors.
	inline double dot(vec a, vec b);

	//calculates the magnitude of a vector.
	inline double magnitude(vec A);

	//prints a vector (for testing).
	void printVec(vec A);

	//rotates a vector by t radians.
	vec rotate(vec v, float t);

	//--------------------------------------------MATRIX MATHS--------------------------------------------

	//prints a matrix (for testing).
	void printMatrix(std::array<double, 3> M);

	//calculates the determinant of a 2x2 matrix.
	double det2x2(std::array <double, 4> M);

	//calculates the determinant of a 3x3 matrix.
	double det3x3(std::array <double, 9> M);

	//finds the matrix of cofactors for a 3x3 matrix.
	std::array<double, 9> cofactorsPlus(std::array<double, 9> M);

	//inverts a 3x3 matrix.
	std::array < double, 9 > invert3x3(std::array<double, 9> M);

	//multiplies a 3x3 matrix by a 3x1 matrix (or 3x1 vector).
	std::array <double, 3> multiply3x3X3x1(std::array<double, 9> A, std::array<double, 3> B);



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
		Matrix3D(std::array<std::array<T, 3>, 3> floats) {
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

	//Calculates the v-value of a point Q over a line AB.
	double getV(vec A, vec B, vec Q);

	//Calculates the closest point on a line AB to a point Q.
	vec closestPoint(vec A, vec B, vec Q);

	//Returns the point in A which has the greatest dot product with d.
	int support(const convex& A, const vec& d);

	//Returns the difference between support(A,d) and support(b,-d).
	vec minkowskiDifference(const convex& A, const convex& B, const vec d);

	//returns True if V contains an instance of v.
	bool contains(std::vector<vec>& V, vec v);

	//Runs EPA, returning the collision normal for two shapes that are colliding.
	vec epa(const convex& A, const convex& B, simplexVec S);

	/*	Calculates which vertex is most perpendicular to penetrationNormal:
	  A[v+1]-A[v] or A[v]-A[v-1]. Considers circular nature of shapes.	*/
	int mostPerpendicular(const convex& A, const int v, const vec penetrationNormal);

	//Returns true if Q is on the left of the line AB.
	bool onLeftOfLine(const vec A, const vec B, const vec Q, bool clockwise);

	/*	Gets the indices of the next and previous points to a in a shape of size "size".
	  Considers the cyclical nature of shapes - the two ends of the shape are connected. */
	vec getNextAndPrevious(int a, int size);

	/*	returns true if p is inside the convex shape A. "startpoint1" and "startpoint2"
		represent the indices of a likely separating axis, allowing the function to return
		early. They can, however, be set arbitrarily.	*/
	bool isPointInside(vec p, const convex* A, int startpoint1 = 0, int startpoint2 = 1);

	/*	rearranges two vertices (which it takes by reference), a and b, in a shape of size
		"size" so that the index of b is greater than the index of a. Considers the fact
		that shapes are cyclical.	*/
	void putInOrder(int& a, int& b, int size);

	/*	returns a manifold containing information about the collision between A and B.
		When impulses are calculated, remember that the collision normal points from A to B!	*/
	vec getCollisionData(const convex& A, const convex& B, collision& man, line& incline, line& refline);

}