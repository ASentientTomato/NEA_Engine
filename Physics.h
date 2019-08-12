#pragma once
#include "geometrytools.h"
//#include "Physics.cpp"

class Rigidbody;

struct manifold {
	Rigidbody* A;
	Rigidbody* B;
	geo::collision col;
};

geo::vec cross(geo::vec V, double x);
geo::vec cross(double x, geo::vec V);
double cross(geo::vec A, geo::vec B);

struct translation {
	double mass;
	double inv_mass;
	geo::vec accelaration = geo::vec{ 0,0 };
	geo::vec velocity = geo::vec{ 0,0 };
};
struct rotation {
	float momentOfInertia = 10000;
	float inv_MOI = 0.0001;
	float angularVelocity = 0;//radians per second
	float accelaration = 0;
	float angle = 0;
};
class Rigidbody {
public:
	translation trans;
	rotation rot;
	geo::vec com;
	//center of mass relative to vertex at 0, when shape is not rotated
	geo::convex shape;
	float restitution = 0.7;
	float static_friction = 0.5;
	float dynamic_friction = 0.5;

	geo::vec getCOM();
	void calculateCOM();
	double getArea();
	//to set mass to infinite, simply pass a negative number
	Rigidbody(geo::convex shape, translation trans, rotation rot, float resitution);
	Rigidbody();
	void translate(geo::vec translationVector);
	void rotate(float radians, geo::vec centerOfRotation);
	void rotate(float radians);
	void applyForce(geo::vec toApply);
	void forceIntegration(float dt);
	void applyImpulse(const geo::vec& impulse, const geo::vec& rx);
};

void resolveManifold(manifold man);