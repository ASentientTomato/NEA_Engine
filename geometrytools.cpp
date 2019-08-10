#pragma once
#include "geometrytools.h"


namespace geo{
	//----------------------------------------------VECTOR MATHS-----------------------------------------------
	vec add(vec a, vec b) {
		return vec{ a.x + b.x, a.y + b.y };
	}
	vec sub(vec a, vec b) {
		return vec{ a.x - b.x, a.y - b.y };
	}
	double dot(vec a, vec b) {
		return a.x * b.x + a.y * b.y;
	}
	vec scale(vec a, double b) {
		return vec{ a.x * b, a.y * b };
	}
	double magnitude(vec A) {
		return sqrt(A.x * A.x + A.y * A.y);
	}
	void printVec(vec A) {
		std::cout << "(" << A.x << ", " << A.y << ") ";
	}
	bool compare(vec A, vec B) {
		return (A.x == B.x && A.y == B.y);
	}

	//----------------------------------------------MATRIX MATHS-----------------------------------------------
	//matrices are defined here such that [1,2,3,4] is equivalent to the matrix
	//	|1  2|
	//	|3  4|
	//MAY BE DANGEROUS TO CALL: Some matrices are non-invertible. Others might behave that way due to floating point errors.
	void printMatrix(std::array<double, 3> M) {
		for (double i : M) {
			std::cout << i << ", ";
		}
	}

	double det2x2(std::array <double, 4> M) {
		return M[0] * M[3] - M[1] * M[2];
	}
	double det3x3(std::array <double, 9> M) {
		std::array <double, 4> a = { M[4],M[5],M[7],M[8] };
		std::array <double, 4> b = { M[3],M[5],M[6],M[8] };
		std::array <double, 4> c = { M[3],M[4],M[6],M[7] };
		return M[0] * det2x2(a) - M[1] * det2x2(b) + M[2] * det2x2(c);
	}
	std::array < double, 9 > cofactorsPlus(std::array<double, 9> M) {	//This line might not work for all compilers...? (C++11 onwards, I think)
	//i swear to god if i made a typo here
	//EDIT: I made a typo here.

		std::array <double, 4> a = { M[4],M[5],M[7],M[8] };
		std::array <double, 4> b = { M[3],M[5],M[6],M[8] };
		std::array <double, 4> c = { M[3],M[4],M[6],M[7] };
		std::array <double, 4> d = { M[1],M[2],M[7],M[8] };
		std::array <double, 4> e = { M[0],M[2],M[6],M[8] };
		std::array <double, 4> f = { M[0],M[1],M[6],M[7] };
		std::array <double, 4> g = { M[1],M[2],M[4],M[5] };
		std::array <double, 4> h = { M[0],M[2],M[3],M[5] };
		std::array <double, 4> i = { M[0],M[1],M[3],M[4] };

		std::array<double, 9> X;
		X[0] = det2x2(a);
		X[1] = -1 * (det2x2(b));
		X[2] = det2x2(c);
		X[3] = -1 * det2x2(d);
		X[4] = det2x2(e);
		X[5] = -1 * det2x2(f);
		X[6] = det2x2(g);
		X[7] = -1 * det2x2(h);
		X[8] = det2x2(i);

		return X;
	}
	std::array < double, 9 > invert3x3(std::array<double, 9> M) {
		std::array<double, 9> X;
		double invDet = 1 / det3x3(M);
		M = cofactorsPlus(M);
		X[0] = invDet * M[0];
		X[1] = invDet * M[3];
		X[2] = invDet * M[6];
		X[3] = invDet * M[1];
		X[4] = invDet * M[4];
		X[5] = invDet * M[7];
		X[6] = invDet * M[2];
		X[7] = invDet * M[5];
		X[8] = invDet * M[8];

		return X;
	}
	std::array <double, 3> multiply3x3X3x1(std::array<double, 9>A, std::array<double, 3>B) {
		std::array<double, 3> X;
		X[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
		X[1] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
		X[2] = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
		return X;
	}

	//-----------------------------------------------Collision Detection-----------------------------------------------

	double getV(vec A, vec B, vec Q) {
		//let's use doubles! (naive solution, but it might just work:)
		vec n = sub(B, A);
		double abMag = sqrt(double(n.x*n.x + n.y*n.y));
		abMag = 1 / abMag;
		n = scale(n, abMag);

		double v = dot(sub(Q, A), n);
		v *= abMag;
		return v;
	}


	vec closestPoint(vec A, vec B, vec Q)
	{
		double v = getV(A, B, Q);
		if (v <= 0) {
			return A;
		}
		else if (v >= 1) {
			return B;
		}
		else {
		}
		return A + scale(sub(B, A), v); //add(scale(A, 1-v), scale(B, v)); works too
	}//add(A, scale(sub(B, A), v)); 

	int support(const convex& A, const vec& d) {
		//Find the greatest point in A, in the direction d.
		int index = 0;
		double biggestProjection = dot(A.points[index], d);
		for (int i = 1; i < A.points.size(); i++) {
			double val = dot(A.points[i], d);
			if (val > biggestProjection) {
				index = i;
				biggestProjection = val;
			}
		}
		return index;
	}

	vec minowskyDifference(const convex &A, const convex &B, const vec d)
	{
		return sub(A.points[support(A, d)], B.points[support(B, scale(d, -1))]);
	}

	bool contains(std::vector<vec> &V, vec v) {
		for (vec i : V) {
			if (compare(i, v)) {
				return true;
			}
		}
		return false;
	}
	vec epa(const convex &A, const convex &B, simplexVec S) {
		vec origin = vec{ 0,0 };
		std::vector<vec> shape = { S.vertexA,S.vertexB,S.vertexC };
		vec direction;
		while (true) {
			vec a = shape[shape.size() - 1]; vec b = shape[0];
			int smallestIndex = 0;
			vec temp = closestPoint(a, b, origin);
			direction = temp;
			double uDistance = dot(temp, temp);	//not sqrt, for efficiency purposes.

			for (int i = 1; i < shape.size(); ++i) {
				a = shape[i - 1]; b = shape[i];
				temp = closestPoint(a, b, origin);
				if (dot(temp, temp) < uDistance) {
					uDistance = dot(temp, temp);
					smallestIndex = i;
					direction = temp;
				}
			}

			vec newPoint = minowskyDifference(A, B, direction);
			if (contains(shape, newPoint)) {
				return direction;
			}
			else {
				{
					std::vector<vec> temp;
					for (int j = 0; j < shape.size(); ++j) {
						if (j == smallestIndex) {
							temp.push_back(newPoint);
						}
						temp.push_back(shape[j]);
					}
					shape = temp;
				}

			}

		}

	}

	int mostPerpendicular(const convex &A, const int v, const vec penetrationNormal)
	{
		int left;
		int right;
		if (v == A.points.size() - 1) {
			right = 0;
			left = v - 1;
		}
		else if (v == 0) {
			left = A.points.size() - 1;
			right = v + 1;
		}
		else {
			left = v - 1;
			right = v + 1;
		}
		vec l = sub(A.points[v], A.points[left]);
		vec r = sub(A.points[v], A.points[right]);

		l = scale(l, 1 / magnitude(l));
		r = scale(r, 1 / magnitude(r));

		if (abs(dot(r, penetrationNormal)) <= abs(dot(l, penetrationNormal))) {
			//the right edge is more perpendicular
			return right;
		}
		else {
			//the left edge is more perpendicular
			return left;
		}
	}
	//don't forget to only call this with index of B > index of A, to maintain vector winding
	bool onLeftOfLine(const vec A, const vec B, const vec Q, bool clockwise) {
		vec v = sub(B, A);
		v = vec{ v.y,v.x*-1 };
		return !((dot(v, sub(Q, A)) > 0) ^ clockwise);

		//1, 1 = 1
		//1, 0 = 0
		//0, 1 = 0
		//0, 0 = 1

	}

	//x = previous, y = next
	vec getNextAndPrevious(int a, int size) {
		size -= 1;
		if (a == size)
		{
			return vec{ (double)a - 1,(double)0 };
		}
		else if (a == 0)
		{
			return vec{ (double)size,(double)1 };
		}
		return vec{ (double)a - 1,(double)a + 1 };
	}

	//-by considering the two startpoints, I can make this method somewhat more efficient
	//-since I know where p is inside the shape, I can make considerable efficiency improvements (over, say, using GJK here)
	bool isPointInside(vec p, const convex *A, int startpoint1, int startpoint2)
	{
		bool clockwise = A->clockwise;
		//this method uses SAT tests rather than GJK
		//if the point isn't inside, this method (with good startpoints) exits almost immediately.

		if (onLeftOfLine(A->points[startpoint1], A->points[startpoint2], p, clockwise)) {
			return false;	//this is usually the best separating axis
		}

		bool forward;	//it is sometimes faster to search the shape backwards.
		double v = getV(A->points[startpoint1], A->points[startpoint2], p);
		forward = (v > 0);


		if (forward) {
			for (int i = startpoint2 + 1; i < A->points.size(); ++i)
			{
				if (onLeftOfLine(A->points[i - 1], A->points[i], p, clockwise)) { return false; }
			}
			for (int i = 0; i < startpoint1; ++i)
			{
				if (onLeftOfLine(A->points[i], A->points[i + 1], p, clockwise)) { return false; }
			}
		}
		else {
			for (int i = startpoint1; i > 0; --i)
			{
				if (onLeftOfLine(A->points[i - 1], A->points[i], p, clockwise)) { return false; }
			}
			for (int i = A->points.size() - 1; i > startpoint2; --i)
			{
				if (onLeftOfLine(A->points[i - 1], A->points[i], p, clockwise)) { return false; }
			}
		}
		//test the edge cases (point at A.size, and point at 0)
		if (onLeftOfLine(A->points[A->points.size() - 1], A->points[0], p, clockwise)) { return false; }

		return true;
	}



	//takes in two integers (a and b) which MUST be next to each other
	void putInOrder(int &a, int &b, int size) {
		int c;
		if (b == size && a == 0) {//4,5
			c = a;
			a = b;
			b = c;
			return;
		}
		else if (b == 0 && a == size) {
			return;
		}
		else if (a > b) {
			c = b;
			b = a;
			a = c;
			return;
		}
	}

	//returns collision data containing the direction of the force that should be applied to B.
	vec getCollisionData(const convex &A, const convex &B, collision& man, line& incline, line& refline)
		//incline and refline are here for testing
	{

		//pick arbitrary initial simplex
		simplexVec S;
		S.count = 1;	//TODO: this function is structured in a really dumb way.
		S.vertexA = minowskyDifference(A, B, A.points[0]);
		vec d = scale(A.points[0], -1);	//this is lacking a lot of potential efficiency improvements.
		S.count = 2;	//I should implement them at some point.
		S.vertexB = minowskyDifference(A, B, d);
		if (S.vertexA.x == S.vertexB.x && S.vertexA.y == S.vertexB.y) {
			return S.vertexA;
		}
		vec lastd = { DBL_MAX,DBL_MAX };
		while (true) {
			if (S.count == 2) {

				vec onLine = closestPoint(S.vertexA, S.vertexB, vec{ 0,0 });
				if (dot(onLine,onLine) < dot(lastd,lastd)) {
					lastd = onLine;
				}
				else {
					return lastd;
				}
				d = scale(onLine, -1);
				//if you get infinite loops, check this condition.
				if (d.x == 0 && d.y == 0 || abs(d.x) < 0.0000001 && abs(d.y) < 0.0000001) {
					return onLine;
				}
				vec toAdd = minowskyDifference(A, B, d);
				if (compare(toAdd, S.vertexA) || compare(toAdd, S.vertexB)) {
					return onLine;
				}
				else {
					S.count = 3;
					S.vertexC = toAdd;
				}
			}
			else if (S.count == 3) {
				vec A1 = S.vertexA;
				vec B1 = S.vertexB;
				vec C1 = S.vertexC;

				vec Q = { 0,0 };

				double vAB = getV(A1, B1, Q);
				double vBC = getV(B1, C1, Q);
				double vCA = getV(C1, A1, Q);

				vec closest;

				std::array <double, 9> M = { A1.x,B1.x,C1.x,A1.y,B1.y,C1.y,1,1,1 };
				std::array<double, 3> R = { Q.x,Q.y,1 };
				R = multiply3x3X3x1(invert3x3(M), R);
				double u = R[0];
				double v = R[1];
				double w = R[2];
				

				if (w > 0 && v > 0 && u > 0) {
					//it's in the triangle
					vec penetrationNormal = epa(A, B, S);
					//^returns a vector pointing in the direction B needs to move in to no longer collide with A

					//now, calculate the collision location:
					{
						double mag = magnitude(penetrationNormal);
						man.contacts[0].normal = scale(penetrationNormal, 1 / mag);//correct
						man.contacts[0].penetration = mag;//correct
					}

					d = penetrationNormal;
					//find the (index of) points of most penetration
					int maxPenetrationA = support(A, d);
					d = scale(d, -1);
					int maxPenetrationB = support(B, d);

					int maxPenetrationA2 = mostPerpendicular(A, maxPenetrationA, penetrationNormal);
					int maxPenetrationB2 = mostPerpendicular(B, maxPenetrationB, penetrationNormal);

					vec lineB = sub(B.points[maxPenetrationB2], B.points[maxPenetrationB]);
					vec lineA = sub(A.points[maxPenetrationA2], A.points[maxPenetrationA]);

					const convex *referenceShape;
					int reference1;
					int reference2;

					const convex *incidentShape;
					int incident1;
					int incident2;


					double a = abs(dot(scale(lineB, 1 / magnitude(lineB)), penetrationNormal));
					double b = abs(dot(scale(lineA, 1 / magnitude(lineA)), penetrationNormal));

					//fixing rounding error problems:
					if (a < 0.000000001) {
						a = 0;
					}
					if (b < 0.000000001) {
						b = 0;
					}

					//determine reference and incident shape (incident = more perpendicular to the penetration normal = the penetrating shape)
					if (a < b || (a == b && dot(lineB, lineB) > dot(lineA, lineA))) {
						referenceShape = &B;
						man.aIsReference = false;
						reference1 = maxPenetrationB;
						reference2 = maxPenetrationB2;

						incidentShape = &A;
						incident1 = maxPenetrationA;
						incident2 = maxPenetrationA2;
					}

					else {
						referenceShape = &A;
						man.aIsReference = true;
						reference1 = maxPenetrationA;
						reference2 = maxPenetrationA2;

						incidentShape = &B;
						incident1 = maxPenetrationB;
						incident2 = maxPenetrationB2;
						d = penetrationNormal;
					}



					if (a == b)
					{
						//there's ambiguity in this situation.
						int re1 = reference1; int re2 = reference2;
						putInOrder(re1, re2, referenceShape->points.size() - 1);
						//if (re1 != re2 + 1 && re2 != re1 + 1) {
						//} what the hell is this
						if (isPointInside(incidentShape->points[incident2], referenceShape, re1, re2)) {
							//switch incident shape points
							//int temp = incident1;
							//incident1 = incident2;
							//incident2 = temp;
							std::swap(incident1, incident2);
						}
					}

					man.contacts[0].position = incidentShape->points[incident1];//add((*incidentShape).points[incident1], d);

					int biggerreference = reference1;
					int smallerreference = reference2;
					putInOrder(smallerreference, biggerreference, referenceShape->points.size() - 1);


					//check if incident2 is also inside referenceShape
					if (!isPointInside(incidentShape->points[incident2], referenceShape, smallerreference, biggerreference))//this is almost always the best separating axis:
					{
						//...it's not:

						//one of the reference shape's vertices might be touching the shape.
						double v = getV(incidentShape->points[incident1], incidentShape->points[incident2], referenceShape->points[reference1]);

						int a = incident1;
						int b = incident2;
						putInOrder(a, b, incidentShape->points.size() - 1);

						int suspiciousPoint;
						if (v < 0 || v > 1) {
							suspiciousPoint = reference2;
						}
						else {
							suspiciousPoint = reference1;
						}

						if (isPointInside(referenceShape->points[suspiciousPoint], incidentShape, a, b)) {
							man.contactcount = 2;
							vec pen = closestPoint(incidentShape->points[incident1], incidentShape->points[incident2], referenceShape->points[suspiciousPoint]);
							man.contacts[1].position = pen;
							pen = sub(referenceShape->points[suspiciousPoint], pen);
							man.contacts[1].penetration = magnitude(pen);
							if(!man.aIsReference){
								pen = scale(pen, -1);
							}
							man.contacts[1].normal = scale(pen, 1 / man.contacts[1].penetration);
							refline.start = referenceShape->points[reference1];
							refline.direction = sub(referenceShape->points[reference2], refline.start);

							incline.start = incidentShape->points[incident1];
							incline.direction = sub(incidentShape->points[incident2], incline.start);

							return Q;
						}

						man.contactcount = 1;


						refline.start = referenceShape->points[reference1];
						refline.direction = sub(referenceShape->points[reference2], refline.start);

						incline.start = incidentShape->points[incident1];
						incline.direction = sub(incidentShape->points[incident2], incline.start);


						return Q;
					}

					//...it is!

					//we now know the second point is also inside the shape. We don't, however, know that it is closest to the reference face.
					//1. calculate v
					//2. if v is bigger than 1, find penetration normal based on next face. if it's smaller than 0, check previous face.
							//-if it is between zero and one, the line is closest to the reference face.


					//assumption: the second vertex, if inside the shape, penetrated through the same line as the first vertex


					man.contactcount = 2;
					vec penetrationVector = sub(closestPoint(referenceShape->points[reference1], referenceShape->points[reference2], incidentShape->points[incident2]), incidentShape->points[incident2]);
					man.contacts[1].penetration = magnitude(penetrationVector);
					man.contacts[1].normal = scale(penetrationVector, 1 / man.contacts[1].penetration);
					if (!man.aIsReference) {
						man.contacts[1].normal = scale(man.contacts[1].normal, -1);
					}
					man.contacts[1].position = incidentShape->points[incident2];//add(incidentShape->points[incident2],penetrationVector);



					refline.start = referenceShape->points[reference1];
					refline.direction = sub(referenceShape->points[reference2], refline.start);

					incline.start = incidentShape->points[incident1];
					incline.direction = sub(incidentShape->points[incident2], incline.start);


					return Q;
				}
				else if (vCA > 0 && vCA < 1 && v <= 0) {
					//it's on CA - B can be removed.
					S.vertexB = S.vertexC;
					S.count = 2;
				}
	
				else {

					//it's on BC - A can be removed.
					S.vertexA = S.vertexB;
					S.vertexB = S.vertexC;
					S.count = 2;
				}


			}
		}
	}
}