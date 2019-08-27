#pragma once
#include <vector>

namespace geo{
	class vec {
	public:
		double x;
		double y;
		inline vec operator+(const vec& v) const {		//these inline declarations are likely already performed by the compiler, but why not use em anyway just in case
			return vec{ x + v.x, y + v.y };
		}
		inline void operator+=(const vec& v) {
			this->x += v.x;
			this->y += v.y;
		}
		inline vec operator-(const vec& v) const {
			return vec{ x - v.x, y - v.y };
		}
		inline void operator-=(const vec& v) {
			this->x -= v.x;
			this->y -= v.y;
		}
		inline vec operator*(const double& d) const {
			return vec{ x*d, y*d };
		}
		inline bool operator==(const vec& v) const {
			return (x == v.x && y == v.y);
		}
	};

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
		bool aIsReference;
	} collision;
}