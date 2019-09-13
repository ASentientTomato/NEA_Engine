#pragma once
#include "Physics.h"


//from 0 to 1, how much overlapping shapes should be translated to prevent them from overlapping
#define CORRECTION_FACTOR 0.8


///////////////////////////////////////////////////////////////////	Maths ///////////////////////////////////////////////////////////////////

//this should all be in geometry
geo::vec cross(geo::vec V, double x) {
	return { x * V.y, -x * V.x };
}
geo::vec cross(double x, geo::vec V) {
	return { -x * V.y, x * V.x };
}
double cross(geo::vec A, geo::vec B) {
	return  A.x * B.y - A.y * B.x;
}

manifold::manifold(){
	A = NULL;
	B = NULL;
	col.contactcount = 0;
}

bool isClockwise(Shape sh) {
	double area = 0;
	size_t j = sh.points.size() - 1;
	for (int i = 0; i < sh.points.size(); i++) {
		area += (sh.points[j].x + sh.points[i].x) * (sh.points[j].y - sh.points[i].y);
		j = i;
	}
	return(area <= 0);
}


///////////////////////////////////////////////////////////////////	Shape Class	///////////////////////////////////////////////////////////////////
void Shape::translate(geo::vec translationVector) {
	for (geo::vec& i : this->points) {
		i = i + translationVector;
	}
}

void Shape::rotate(float radians, const geo::vec& centerOfRotation) {
	geo::vec v;
	float cosine = cos(radians);
	float sine = sin(radians);
	for (geo::vec& i : this->points) {
		v = i - centerOfRotation;
		i = centerOfRotation + geo::vec{ cosine * v.x - sine * v.y, sine * v.x + cosine * v.y };
	}
}

void Shape::zoom(float zoomFactor, geo::vec centerOfZoom) {
	geo::vec temp;

	for (geo::vec& i : this->points) {

		temp = i - centerOfZoom;
		temp = temp * zoomFactor;
		i = centerOfZoom + temp;

	}

}



int Shape::next(int i) {
	if (i >= this->points.size()-1) {
		return 0;
	}
	else {
		return i+1;
	}
}

int Shape::previous(int i) {
	if (i <= 0) {
		return this->points.size() - 1;
	}
	else {
		return i - 1;
	}
}
geo::vec& Shape::operator[](int i) {
	return this->points[i];
}
/*template<class T> Shape Shape::applyMatrix(geo::Matrix3D<T> mat, const geo::vec& center) {
	geo::vec temp;
	Shape shape;
	for (int i = 0; i < this->points.size(); i++) {
		temp = this->points[i] - center;
		temp = mat * temp;
		shape.points.push_back(temp + center);

	}
}*/

Shape Shape::applyMatrix(geo::Matrix3D<float> mat, const geo::vec& center) {
	geo::vec temp;
	Shape shape;
	for (int i = 0; i < this->points.size(); i++) {
		temp = this->points[i] - center;
		temp = mat * temp;
		shape.points.push_back(temp + center);
	}
	return shape;
}

///////////////////////////////////////////////////////////////////	Rigidbody Class	///////////////////////////////////////////////////////////////////
geo::vec Rigidbody::getCOM() {
	float cosine = cos(this->rot.angle);
	float sine = sin(this->rot.angle);
	geo::vec centerOfMass = geo::vec{ cosine * this->com.x - sine * this->com.y, sine * this->com.x + cosine * this->com.y };
	return centerOfMass + this->points[0];
}


void Rigidbody::calculateCOM()
{
	geo::vec centroid = { 0, 0 };
	double signedArea = 0.0;
	double x0 = 0.0; // Current vertex X
	double y0 = 0.0; // Current vertex Y
	double x1 = 0.0; // Next vertex X
	double y1 = 0.0; // Next vertex Y
	double a = 0.0;  // Partial signed area

	int i = 0;
	for (i = 0; i < this->points.size() - 1; ++i)
	{
		x0 = this->points[i].x;
		y0 = this->points[i].y;
		x1 = this->points[i + 1].x;
		y1 = this->points[i + 1].y;
		a = x0 * y1 - x1 * y0;
		signedArea += a;
		centroid.x += (x0 + x1) * a;
		centroid.y += (y0 + y1) * a;
	}

	x0 = this->points[i].x;
	y0 = this->points[i].y;
	x1 = this->points[0].x;
	y1 = this->points[0].y;
	a = x0 * y1 - x1 * y0;
	signedArea += a;
	centroid.x += (x0 + x1) * a;
	centroid.y += (y0 + y1) * a;

	signedArea *= 0.5;
	centroid.x /= (6.0 * signedArea);
	centroid.y /= (6.0 * signedArea);

	this->com = centroid - this->points[0];
	this->rot.angle = 0;
}


void Rigidbody::rotate(float radians, const geo::vec& centerOfRotation) {
	geo::vec v;
	float cosine = cos(radians);
	float sine = sin(radians);
	for (geo::vec& i : this->points) {
		v = i - centerOfRotation;
		i = centerOfRotation + geo::vec{ cosine * v.x - sine * v.y, sine * v.x + cosine * v.y };
	}
	this->rot.angle += radians;
}


void Rigidbody::rotate(float radians) {
	//c++ is being annoying
	this->rotate(radians, this->getCOM());
}


double Rigidbody::getArea() {
	double area = 0;
	size_t j = this->points.size() - 1;
	for (int i = 0; i < this->points.size(); i++) {
		area += (this->points[j].x + this->points[i].x) * (this->points[j].y - this->points[i].y);
		j = i;
	}
	return abs(area / 2.0);
}


//to set mass to infinite, simply pass a negative number
Rigidbody::Rigidbody(geo::convex shape, translation trans, rotation rot, float resitution) {
	this->points = shape.points;
	this->clockwise = isClockwise(*this);
	this->clockwise = shape.clockwise;
	this->rot = rot;
	this->restitution = restitution;
	calculateCOM();
	//infinite mass
	if (trans.mass < 0) {
		trans.inv_mass = 0;
		trans.mass = DBL_MAX;
	}
	else {
		trans.inv_mass = 1 / trans.mass;
	}
	this->trans = trans;
}


Rigidbody::Rigidbody() {}



void Rigidbody::applyForce(geo::vec toApply) {
	this->trans.accelaration = this->trans.accelaration + (toApply * this->trans.inv_mass);
}


void Rigidbody::forceIntegration(float dt) {
	//semi-implicit euler integration//
	//¯\_(ツ)_/¯//
	this->trans.velocity = this->trans.velocity + (this->trans.accelaration * dt);
	this->translate(this->trans.velocity * dt);
	this->trans.accelaration = { 0,0 };

	this->rot.angularVelocity = this->rot.angularVelocity + (dt * this->rot.accelaration);
	this->rotate(this->rot.angularVelocity * dt);
	this->rot.accelaration = 0;
}


void Rigidbody::applyImpulse(const geo::vec& impulse, const geo::vec& rx) {
	this->trans.velocity = this->trans.velocity + (impulse * this->trans.inv_mass);
	this->rot.angularVelocity += this->rot.inv_MOI * cross(rx, impulse);
}


void resolveManifold(manifold man) {

	static int i = 0;
	i++;
	if (i == 24) {
		std::cout << "bruh";
	}

	//TODO: test out other methods to calculate the frictions (i.e. pythagoras, always choose greatest/smallest).
	float staticFriction = (man.A->static_friction + man.B->static_friction) / 2;
	float dynamicFriction = (man.A->dynamic_friction + man.B->dynamic_friction) / 2;



	double restitution;
	if (man.A->restitution < man.B->restitution) {
		restitution = man.A->restitution;
	}
	else {
		restitution = man.B->restitution;
	}


	//translate the shapes by some correction factor, to ensure stability (decrease jittering).
	//TODO: maybe instead of using contacts[0] for this use average contact or something along those lines.

	{
		geo::vec alpha;
		//If both shapes are unmovable, move them equal amounts

		bool aIsInfinite = (man.A->trans.inv_mass < 1e-200);
		bool bIsInfinite = (man.B->trans.inv_mass < 1e-200);
		if (aIsInfinite && bIsInfinite) {
			alpha = man.col.contacts[0].normal * (man.col.contacts[0].penetration * CORRECTION_FACTOR / 2);
			man.B->translate(alpha);
			man.A->translate(alpha * -1);
		}

		//If one shape is immovable, only move the other
		else if (aIsInfinite || bIsInfinite) {
			alpha = man.col.contacts[0].normal * (man.col.contacts[0].penetration * CORRECTION_FACTOR);
			if (aIsInfinite) {
				man.B->translate(alpha);
			}
			else {
				man.A->translate(alpha * -1);
			}
		}

		//If both shapes are movable, move either
		else {
			alpha = man.col.contacts[0].normal * ((man.col.contacts[0].penetration * CORRECTION_FACTOR) / (man.A->trans.mass + man.B->trans.mass));
			man.B->translate(alpha * man.A->trans.mass);
			man.A->translate(alpha * -man.B->trans.mass);
		}
		if (dot(alpha, alpha) < 1e-350) {
			alpha = { 0, 0 };
		}
	}


	for (unsigned int i = 0; i < man.col.contactcount; i++) {

		geo::vec ra = man.col.contacts[i].position - man.A->getCOM();
		geo::vec rb = man.col.contacts[i].position - man.B->getCOM();


		geo::vec rv = man.B->trans.velocity - man.A->trans.velocity;								//calculate relative velocity
		rv = rv + cross(man.B->rot.angularVelocity, rb) - cross(man.A->rot.angularVelocity, ra);	//consider angular velocity, too
		double vac = geo::dot(rv, man.col.contacts[i].normal);							//velocity at contact
		if (vac > 0) {
			//don't resolve if objects are moving apart
			return;
		}
		//TODO: maybe use the other formula
		double totalInverseMasses = man.A->trans.inv_mass + man.B->trans.inv_mass;
		double division = totalInverseMasses + (std::pow(cross(ra, man.col.contacts[i].normal), 2) * man.A->rot.inv_MOI)
			+ (std::pow(cross(rb, man.col.contacts[i].normal), 2) * man.B->rot.inv_MOI);

		if (division == 0) {
			return;
		}

		double impulseScalar = -(1 + restitution) * vac;
		impulseScalar /= division;
		impulseScalar /= man.col.contactcount;

		geo::vec impulse = man.col.contacts[i].normal * impulseScalar;

		man.A->applyImpulse(impulse * -1, ra);
		man.B->applyImpulse(impulse, rb);

		//friction:
		rv = man.B->trans.velocity - man.A->trans.velocity;
		rv = rv + cross(man.B->rot.angularVelocity, rb) - cross(man.A->rot.angularVelocity, ra);
		//if (geo::dot(rv, rv) < 0.00000001) {
		//	return;
		//}

		geo::vec t = rv - (man.col.contacts[i].normal * dot(rv, man.col.contacts[i].normal));	//tangent
		//if (geo::dot(t, t) < 0.00000001) { continue; };

		{
			double magnitude = geo::magnitude(t);
			if (magnitude > 0.0001) {
				t = t * (1 / magnitude); //unitTangent
			}
			else break;
		}
		double jt = -geo::dot(rv, t);	//friction
		jt /= totalInverseMasses;
		jt /= (double)man.col.contactcount;

		geo::vec frictionImpulse;
		//std::cout << std::abs(jt) << std::endl << impulseScalar; // staticFriction;

		if (std::abs(jt) < impulseScalar * staticFriction) {
			frictionImpulse = t * jt;
		}
		else {
			frictionImpulse = t * -impulseScalar * dynamicFriction;
		}

		man.A->applyImpulse(frictionImpulse * -1, ra);
		man.B->applyImpulse(frictionImpulse, rb);
		//TODO: this has very strange behaviour when the collision has TWO contact points

	}

}


