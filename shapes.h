#pragma once
#include <vector>

namespace geo{
	typedef struct vec {
		double x;
		double y;
	} vec;

	typedef struct convex {
		std::vector<vec> points;
		bool clockwise;
	} convex;

	typedef struct line
	{
		vec start;
		vec direction;
	} line;

	typedef struct simplexVec
	{
		vec vertexA;
		vec vertexB;
		vec vertexC;
		int count;
	} simplexVec;

	typedef struct contact {
		vec position;
		vec normal;
		double penetration;
	} contact;

	typedef struct collision {
		contact contacts[2];
		unsigned int contactcount;
	} collision;
}