#pragma once
#include "geometrytools.h"
#include "Graphics.h"

//#include "Physics.cpp"

class Rigidbody;

struct manifold {
	Rigidbody* A;
	Rigidbody* B;
	geo::collision col;
	manifold();
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
	float inv_MOI = 0.0001f;
	float angularVelocity = 0;//radians per second
	float accelaration = 0;
	float angle = 0;
};
class Shape : public geo::convex{
public:
	void translate(geo::vec translationVector);
	void rotate(float radians, const geo::vec& centerOfRotation);
	void zoom(float zoomFactor, geo::vec centerOfZoom);
	Shape applyMatrix(geo::Matrix3D<float> mat, const geo::vec& center);
	int next(int i);
	int previous(int i);
	geo::vec& operator[](int i);

	Graphics::Displayable displayable;
};
class Rigidbody : public Shape{
public:
	translation trans;
	rotation rot;
	geo::vec com;
	//center of mass relative to vertex at 0, when shape is not rotated
	
	float restitution = 0.7f;
	float static_friction = 0.5f;
	float dynamic_friction = 0.5f;

	geo::vec getCOM();
	void calculateCOM();
	double getArea();
	//to set mass to infinite, simply pass a negative number
	Rigidbody(geo::convex shape, translation trans, rotation rot, float resitution);
	Rigidbody();
	void applyForce(geo::vec toApply);
	void forceIntegration(float dt);
	void applyImpulse(const geo::vec& impulse, const geo::vec& rx);
	void rotate(float radians);
	void rotate(float radians, const geo::vec& centerOfRotation);
};

void resolveManifold(manifold man);



//returns true if shape's winding is clockwise
//works for all shapes
bool isClockwise(Shape sh);