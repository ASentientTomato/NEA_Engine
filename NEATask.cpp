// NEATask.cpp : Defines the entry point for the console application.
//
#pragma once
#include "geometrytools.h"
#include <SFML\Graphics.hpp>

#include <cmath>

#define FPS 60
#define SCREEN_X 1400
#define SCREEN_Y 1000
#define MAX_MISSED_TIME 0.05

//#define VERLET_INTEGRATE
#define SYMPLECTIC_EULER

//this goes in physics.cpp/physics.h

struct translation {
	double mass;
	double inv_mass;
	geo::vec accelaration = geo::vec{ 0,0 };
	geo::vec velocity = geo::vec{ 0,0 };
};
struct rotation {
	float momentOfInertia = 10;
	float inv_MOI = 0.1;
	float angularVelocity = 0;//radians per second
	float accelaration = 0;
	float angle = 0;
};

class Rigidbody {
	//note: I like using this-> since it marks things more neatly (in my opinion).
private:


public:
	translation trans;
	rotation rot;
	geo::vec com;
	//center of mass relative to vertex at 0, when shape is not rotated
	geo::convex shape;
	float restitution;

	geo::vec getCOM() {
		float cosine = cos(this->rot.angle);
		float sine = sin(this->rot.angle);
		geo::vec centerOfMass = geo::vec{ cosine * this->com.x - sine * this->com.y, sine * this->com.x + cosine * this->com.y };
		return geo::add(centerOfMass, this->shape.points[0]);
	}
	void calculateCOM()
	{
		geo::vec centroid = { 0, 0 };
		double signedArea = 0.0;
		double x0 = 0.0; // Current vertex X
		double y0 = 0.0; // Current vertex Y
		double x1 = 0.0; // Next vertex X
		double y1 = 0.0; // Next vertex Y
		double a = 0.0;  // Partial signed area

		int i = 0;
		for (i = 0; i < this->shape.points.size() - 1; ++i)
		{
			x0 = this->shape.points[i].x;
			y0 = this->shape.points[i].y;
			x1 = this->shape.points[i + 1].x;
			y1 = this->shape.points[i + 1].y;
			a = x0 * y1 - x1 * y0;
			signedArea += a;
			centroid.x += (x0 + x1)*a;
			centroid.y += (y0 + y1)*a;
		}

		x0 = this->shape.points[i].x;
		y0 = this->shape.points[i].y;
		x1 = this->shape.points[0].x;
		y1 = this->shape.points[0].y;
		a = x0 * y1 - x1 * y0;
		signedArea += a;
		centroid.x += (x0 + x1)*a;
		centroid.y += (y0 + y1)*a;

		signedArea *= 0.5;
		centroid.x /= (6.0*signedArea);
		centroid.y /= (6.0*signedArea);

		this->com = geo::sub(centroid, this->shape.points[0]);
		this->rot.angle = 0;
	}

	double getArea() {
		double area = 0;
		int j = this->shape.points.size() - 1;
		for (int i = 0; i < this->shape.points.size(); i++) {
			area += (this->shape.points[j].x + this->shape.points[i].x) * (this->shape.points[j].y - this->shape.points[i].y);
			j = i;
		}
		return abs(area / 2.0);
	}

	Rigidbody(geo::convex shape, translation trans, rotation rot, float resitution) {
		this->shape = shape;
		this->trans = trans;
		this->rot = rot;
		this->restitution = restitution;
		calculateCOM();
		//infinite mass
		if (trans.mass < 0) {
			trans.inv_mass = 0;
		}
		else {
			trans.inv_mass = 1 / trans.mass;
		}
	}

	Rigidbody() {}



	void translate(geo::vec translationVector) {
		for (geo::vec& i : this->shape.points) {
			i = geo::add(i, translationVector);
		}
	}

	void rotate(float radians, geo::vec centerOfRotation) {
		geo::vec v;
		float cosine = cos(radians);
		float sine = sin(radians);
		for (geo::vec& i : this->shape.points) {
			v = geo::sub(i, centerOfRotation);
			i = geo::add(centerOfRotation, geo::vec{ cosine * v.x - sine * v.y, sine * v.x + cosine * v.y });
		}
		this->rot.angle += radians;
	}
	void rotate(float radians) {
		//c++ is being annoying
		this->rotate(radians, this->getCOM());
	}



	void applyForce(geo::vec toApply) {
		this->trans.accelaration = geo::add(this->trans.accelaration, geo::scale(toApply, this->trans.inv_mass));
	}

	void forceIntegration(float dt) {
		//semi-implicit euler integration//
		//¯\_(ツ)_/¯//
		this->trans.velocity = geo::add(this->trans.velocity, scale(this->trans.accelaration, dt));
		this->translate(geo::scale(this->trans.velocity, dt));
		this->trans.accelaration = geo::scale(this->trans.accelaration, 0);

		this->rot.angularVelocity = this->rot.angularVelocity + (dt * this->rot.accelaration);
		this->rotate(this->rot.angularVelocity * dt);
		this->rot.accelaration = 0;
	}

};


std::vector<Rigidbody> GLOBAL_BODIES;
std::vector<sf::ConvexShape> GLOBAL_SHAPES;


struct manifold {
	int shapes[2];	//this is the index of the two colliding bodies on the global list.
	geo::collision col;
};

geo::vec cross(geo::vec V, double x) {
	return { x*V.y, -x * V.x };
}
geo::vec cross(double x, geo::vec V) {
	return { -x * V.y, x * V.x };
}
double cross(geo::vec A, geo::vec B) {
	return  A.x * B.y - A.y * B.x;
}

//TODO: acknowledge second contact point
geo::vec resolveManifold(manifold man) {
	//check if shapes are moving apart
	//TODO: check the rotation as well
	geo::vec rV = geo::sub(GLOBAL_BODIES[man.shapes[0]].trans.velocity, GLOBAL_BODIES[man.shapes[1]].trans.velocity);	//relative velocity
	bool b = geo::dot(rV, man.col.contacts[0].normal) > 0;
	std::cout << man.col.aIsReference;
	if (!b) { std::cout << "BRUH" << std::endl; return geo::vec{ NULL,NULL }; };
	//(b ^ man.col.aIsReference)
	//use smallest restitution
	double restitution;
	if (GLOBAL_BODIES[man.shapes[0]].restitution < GLOBAL_BODIES[man.shapes[1]].restitution) {
		restitution = GLOBAL_BODIES[man.shapes[0]].restitution;
	}
	else {
		restitution = GLOBAL_BODIES[man.shapes[1]].restitution;
	}

	//calculate impulse
	//TODO: may need to flip rv
	double impulseScalar = (1 + restitution) * geo::dot(rV, man.col.contacts[0].normal);
	double division = GLOBAL_BODIES[man.shapes[0]].trans.inv_mass + GLOBAL_BODIES[man.shapes[1]].trans.inv_mass;
	float& rA = GLOBAL_BODIES[man.shapes[0]].rot.angularVelocity;	//rotation
	float& rB = GLOBAL_BODIES[man.shapes[1]].rot.angularVelocity;
	float& iA = GLOBAL_BODIES[man.shapes[0]].rot.inv_MOI;			//moment of inertia
	float& iB = GLOBAL_BODIES[man.shapes[1]].rot.inv_MOI;
	geo::vec& n = man.col.contacts[0].normal;

	geo::vec side = cross(geo::scale(cross(rA, n), iA), rA);
	side = geo::add(side, cross(geo::scale(cross(rB, n), iB), rB));

	division += geo::dot(side, n);

	geo::vec impulse = geo::scale(n, impulseScalar / division);
	return impulse;
}





using namespace geo;
//-----------------------------------------------GRAPHICS-----------------------------------------------
int main()
{
	system("PAUSE");

	//*****************EFFICIENCY STUFF*****************
	const sf::Time timeStep = sf::seconds(1.0f / FPS);
	sf::Time accumulator;
	sf::Clock clock;
	std::cout << "time step: " << timeStep.asSeconds() << " seconds" << std::endl;
	std::cout << "1 pixel = 1 meter" << std::endl;

	//create props
	sf::CircleShape circle;
	vec circlePoint = { 600,600 };
	circle.setRadius(5);
	circle.setPosition(sf::Vector2f{ (float)circlePoint.x,(float)circlePoint.y });
	circle.setFillColor(sf::Color::Blue);
	//a2 = [0], a1 = [1]
	{
	Rigidbody square = Rigidbody();
	square.trans.mass = 10;
	square.trans.inv_mass = (0.1);
	convex shape2;

	shape2.points = { geo::vec{100, 100} , geo::vec{100,200}, geo::vec{200,200}, geo::vec{200,100} };
	shape2.clockwise = false;
	square.shape = shape2;

	GLOBAL_BODIES.push_back(square);
	GLOBAL_BODIES[0].calculateCOM();
	//GLOBAL_BODIES[0].com = sub(scale(add(GLOBAL_BODIES[0].shape.points[0], GLOBAL_BODIES[0].shape.points[2]), 0.5), GLOBAL_BODIES[0].shape.points[0]);

	Rigidbody rigid = Rigidbody();
	rigid.trans.mass = 10;
	rigid.trans.inv_mass = 0.1;
	rigid.shape.points = { geo::vec{100,100}, geo::vec{200,100}, geo::vec{470,255}, geo::vec{327,800}, geo::vec{157,550}, geo::vec{100,300} };
	rigid.shape.clockwise = true;

	GLOBAL_BODIES.push_back(rigid);
	}

	//initialize global shapes
	for (Rigidbody i : GLOBAL_BODIES) {
		sf::ConvexShape shape;
		shape.setPointCount(i.shape.points.size());
		for (int j = 0; j < i.shape.points.size(); j++) {
			sf::Vector2f bruh = sf::Vector2f{ (float)i.shape.points[j].x ,(float)i.shape.points[j].y };
			shape.setPoint(j, bruh);
		}
		GLOBAL_SHAPES.push_back(shape);
	}


	sf::CircleShape point;
	sf::CircleShape point2;
	point2.setFillColor(sf::Color::Red);
	point2.setRadius(2);
	point2.setPosition(sf::Vector2f(0, 0));
	vec p = { 200, 200 };
	point.setRadius(2);
	point.setPosition(sf::Vector2f((float)p.x, (float)p.y));
	point.setFillColor(sf::Color::Red);

	//*****************GRAPHICS SETUP*****************
	sf::RenderWindow window(sf::VideoMode(SCREEN_X, SCREEN_Y), "test");

	//Rigidbody square;
	//square.rot.angularVelocity = 0;
	//square.trans.accelaration = geo::vec{ 0,0 };
	//square.trans.mass = 10;
	//square.trans.inv_mass = 0.1;
	//square.trans.velocity = geo::vec{ 0,0 };
	
	while (window.isOpen()) {

		//*****************EVENT HANDLING*****************
		sf::Event event;

		while (window.pollEvent(event))
		{
			// "close requested" event: we close the window
			if (event.type == sf::Event::Closed)
				window.close();

			//put event handling here
		}

		if (accumulator.asSeconds() > 0.2f) {
			accumulator = sf::seconds(0.2f);
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		accumulator += clock.getElapsedTime();
		clock.restart();
		//std::cout << accumulator.asSeconds() << std::endl;

		//*****************UPDATE WORLD LOGIC*****************
		//	ONLY APPLY FORCES WITHIN THIS LOOP!
		while (accumulator > timeStep) {


			//convex shape2;
			//shape2.points = { vec { a2.getPoint(0).x , a2.getPoint(0).y }, vec{a2.getPoint(1).x, a2.getPoint(1).y}, vec {a2.getPoint(2).x, a2.getPoint(2).y},
			//vec{ a2.getPoint(3).x , a2.getPoint(3).y } };
			//shape2.clockwise = false;
			//GLOBAL_BODIES[0].shape = shape2;
			//GLOBAL_BODIES[0].calculateCOM();



			//apply forces
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
				//GLOBAL_BODIES[0].translate(vec{ -1,0 });
				GLOBAL_BODIES[0].applyForce(vec{ -1000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
				//square.translate(vec{ 1,0 });
				GLOBAL_BODIES[0].applyForce(vec{ 1000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
				GLOBAL_BODIES[0].applyForce(vec{ 0,-1000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
				GLOBAL_BODIES[0].applyForce(vec{ 0,1000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
				GLOBAL_BODIES[0].rotate(0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
				GLOBAL_BODIES[0].rotate(-0.03);
			}
			geo::printVec(GLOBAL_BODIES[0].trans.accelaration);

			accumulator -= timeStep;
			window.clear(sf::Color::Black);
			//***************************DRAW SHAPES***************************
			GLOBAL_BODIES[0].forceIntegration(timeStep.asSeconds());
			//convex shape;
			//shape.clockwise = true;
			//shape.points = { vec { a1.getPoint(0).x , a1.getPoint(0).y }, vec{a1.getPoint(1).x, a1.getPoint(1).y}, vec {a1.getPoint(2).x, a1.getPoint(2).y},
			//vec{ a1.getPoint(3).x , a1.getPoint(3).y }, vec{ a1.getPoint(4).x , a1.getPoint(4).y }, vec{ a1.getPoint(5).x , a1.getPoint(5).y } };




			sf::Vertex referenceLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };
			sf::Vertex incidentLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };



			collision man;
			line inc;
			line ref;
			vec dist = getCollisionData(GLOBAL_BODIES[0].shape, GLOBAL_BODIES[1].shape, man, inc, ref);
			if (dist.x == 0 && dist.y == 0) {
				std::cout << std::endl;
				printVec(man.contacts[0].normal);
				printVec(man.contacts[1].normal);
				point.setPosition(sf::Vector2f{ (float)man.contacts[0].position.x, (float)man.contacts[0].position.y });
				if (man.contactcount > 1) {
					point2.setPosition(sf::Vector2f{ (float)man.contacts[1].position.x, (float)man.contacts[1].position.y });
				}
				else {
					point2.setPosition(sf::Vector2f{ 0,0 });
				}
				incidentLine[0] = sf::Vector2f((float)inc.start.x, (float)inc.start.y);
				incidentLine[1] = sf::Vector2f((float)add(inc.start, inc.direction).x, (float)add(inc.start, inc.direction).y);
				incidentLine[0].color = sf::Color::Red;
				incidentLine[1].color = sf::Color::Red;

				referenceLine[0] = sf::Vector2f((float)ref.start.x, (float)ref.start.y);
				referenceLine[1] = sf::Vector2f((float)add(ref.start, ref.direction).x, (float)add(ref.start, ref.direction).y);
				referenceLine[0].color = sf::Color::Green;
				referenceLine[1].color = sf::Color::Green;
			}
			else {
				point.setPosition(sf::Vector2f{ 0,0 });
				point2.setPosition(sf::Vector2f{ 0,0 });
				printVec(dist);
			}

			int j = 0;
			for (geo::vec i : GLOBAL_BODIES[0].shape.points) {
				GLOBAL_SHAPES[0].setPoint(j, sf::Vector2f{ (float)i.x, (float)i.y });
				j++;
			}

//			std::cout << isPointInside(vec{ circle.getPosition().x, circle.getPosition().y }, &shape2) << std::endl;
			window.draw(GLOBAL_SHAPES[0]);
			window.draw(circle);
			window.draw(GLOBAL_SHAPES[1]);
			window.draw(referenceLine, 2, sf::Lines);
			window.draw(incidentLine, 2, sf::Lines);
			window.draw(point);
			window.draw(point2);

			if (man.contactcount > 0) {
				sf::Vertex pen1[2];
				pen1[0] = point.getPosition();
				pen1[0].color = sf::Color::Green;
				pen1[1].color = sf::Color::Green;
				vec p1 = scale(man.contacts[0].normal, man.contacts[0].penetration);
				sf::Vector2f sfv = point.getPosition();
				pen1[1] = sf::Vector2f{ (float)sfv.x + (float)p1.x, (float)sfv.y + (float)p1.y };
				window.draw(pen1, 2, sf::Lines);
			}
			if (man.contactcount == 2) {
				sf::Vertex pen2[2];
				pen2[0] = point2.getPosition();
				pen2[0].color = sf::Color::Magenta;
				pen2[1].color = sf::Color::Magenta;
				vec p1 = scale(man.contacts[1].normal, man.contacts[1].penetration);
				sf::Vector2f sfv = point2.getPosition();
				pen2[1] = sf::Vector2f{ (float)sfv.x + (float)p1.x, (float)sfv.y + (float)p1.y };
				window.draw(pen2, 2, sf::Lines);
			}

			window.display();
		}

	}
	//
	return 0;
}
