// NEATask.cpp : Defines the entry point for the console application.
//
#pragma once
#include "geometrytools.h"
#include <SFML\Graphics.hpp>

#include <cmath>

#define FPS 60
#define SCREEN_X 1400
#define SCREEN_Y 1000



//this goes in physics.cpp/physics.h

struct translation{
	double mass;
	double inv_mass;
	geo::vec accelaration;
	geo::vec velocity;
};
struct rotation {
	float momentOfInertia;
	float angularVelocity;//radians per second
};
struct centerOfMass {
	geo::vec current;
	geo::vec previous;

	//true if object is behaving properly, false if it disobeyed the laws of physics in the last frame
	//(i.e. teleportation, appearing out of nowhere, and any other interaction not governed by forces)
	bool consistent;
};

class rigidbody {
	//note: I like using this-> since it marks things more neatly (in my opinion).
private:

	void force_translate(geo::vec translationVector) {
		for (geo::vec& i : this->shape.points) {
			i = geo::add(i, translationVector);
		}
		this->com.current = geo::add(this->com.current, translationVector);
	}

	void force_rotate(float radians, geo::vec centerOfRotation) {
		geo::vec v;
		float cosine = cos(radians);
		float sine = sin(radians);
		for (geo::vec& i : this->shape.points) {
			v = geo::sub(i, centerOfRotation);
			i = geo::add(centerOfRotation, geo::vec{ cosine * v.x - sine * v.y, sine * v.x + cosine * v.y });
		}
		v = sub(this->com.current, centerOfRotation);
		this->com.current = geo::add(centerOfRotation, geo::vec{ cosine * v.x - sine * v.y, sine * v.x + cosine * v.y });
	}
	void force_rotate(float radians) {
		//c++ is being annoying
		this->rotate(radians, this->com.current);
	}


public:
	translation trans;
	rotation rot;
	centerOfMass com;

	geo::convex shape;
	float restitution;
		//two needed for velocity verlet integration.


	void rotate(float radians, geo::vec centerOfRotation) {
		this->force_rotate(radians, centerOfRotation);
		this->com.consistent = false;
	}
	void rotate(float radians) {
		this->force_rotate(radians);
		this->com.consistent = false;
	}
	void translate(geo::vec translationVector) {
		this->force_translate(translationVector);
		this->com.consistent = false;
	}


	void applyForce(geo::vec toApply) {
		geo::add(this->trans.accelaration, geo::scale(toApply, this->trans.inv_mass));
	}

	void forceIntegration(float dt) {
		dt *= dt;

		if (this->com.consistent) {
			geo::vec newCOM = sub(scale(this->com.current,2), this->com.previous);
			newCOM = add(newCOM, scale(this->trans.accelaration, dt));
		}
		else {
			this->com.previous = this->com.current;
			geo::vec newCOM = add(this->com.previous, scale(this->trans.velocity, dt));
			newCOM = add(newCOM, scale(this->trans.accelaration, 0.5*dt));
			this->com.current = newCOM;
		}
		this->force_translate(sub(this->com.current, this->com.previous));
		this->com.consistent = true;
		geo::scale(this->trans.accelaration, 0);
	}
};


struct manifold {
	int shapes[2];	//this is the index of the two colliding bodies on the global list.
	geo::collision col;
};







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

	sf::ConvexShape a1;
	a1.setPointCount(6);
	a1.setPoint(0, sf::Vector2f(100, 100));
	a1.setPoint(1, sf::Vector2f(200, 100));
	a1.setPoint(2, sf::Vector2f(470, 255));
	a1.setPoint(3, sf::Vector2f(327, 800));
	a1.setPoint(4, sf::Vector2f(157, 550));
	a1.setPoint(5, sf::Vector2f(100, 300));
	//clockwise

	sf::ConvexShape a2;
	a2.setPointCount(4);
	/*a2.setPoint(0, sf::Vector2f(700, 100));
	a2.setPoint(1, sf::Vector2f(943, 150));
	a2.setPoint(2, sf::Vector2f(1070, 255));
	a2.setPoint(3, sf::Vector2f(927, 570));
	a2.setPoint(4, sf::Vector2f(757, 500));
	*/
	a2.setPoint(0, sf::Vector2f(100, 100));
	a2.setPoint(1, sf::Vector2f(100, 200));
	a2.setPoint(2, sf::Vector2f(200, 200));
	a2.setPoint(3, sf::Vector2f(200, 100));
	//anticlockwise
	


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

		rigidbody square;
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		accumulator += clock.getElapsedTime();
		clock.restart();
		//std::cout << accumulator.asSeconds() << std::endl;

		//*****************UPDATE WORLD LOGIC*****************
		//	ONLY APPLY FORCES WITHIN THIS LOOP!
		while (accumulator > timeStep) {


			convex shape2;
			shape2.points = { vec { a2.getPoint(0).x , a2.getPoint(0).y }, vec{a2.getPoint(1).x, a2.getPoint(1).y}, vec {a2.getPoint(2).x, a2.getPoint(2).y},
			vec{ a2.getPoint(3).x , a2.getPoint(3).y } };
			shape2.clockwise = false;
			square.shape = shape2;
			square.com.current = scale(add(square.shape.points[0], square.shape.points[2]), 0.5);



			//apply forces
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
				square.translate(vec{ -1,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
				square.translate(vec{ 1,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
				square.translate(vec{ 0,-1 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
				square.translate(vec{ 0,1 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
				square.rotate(0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
				square.rotate(-0.03);
			}


			accumulator -= timeStep;
			window.clear(sf::Color::Black);
			//***************************DRAW SHAPES***************************

			convex shape;
			shape.clockwise = true;
			shape.points = { vec { a1.getPoint(0).x , a1.getPoint(0).y }, vec{a1.getPoint(1).x, a1.getPoint(1).y}, vec {a1.getPoint(2).x, a1.getPoint(2).y},
			vec{ a1.getPoint(3).x , a1.getPoint(3).y }, vec{ a1.getPoint(4).x , a1.getPoint(4).y }, vec{ a1.getPoint(5).x , a1.getPoint(5).y } };




			sf::Vertex referenceLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };
			sf::Vertex incidentLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };



			collision man;
			line inc;
			line ref;
			vec dist = getCollisionData(shape, shape2, man, inc, ref);
			if (dist.x == 0 && dist.y == 0) {
				std::cout << std::endl;
				printVec(man.contacts[0].normal);
				printVec(man.contacts[1].normal);
				point.setPosition(sf::Vector2f{ (float)man.contacts[0].position.x, (float)man.contacts[0].position.y });
				if(man.contactcount > 1){
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
			for (geo::vec i : square.shape.points) {
				a2.setPoint(j, sf::Vector2f{ (float)i.x, (float)i.y });
				j++;
			}

			std::cout << isPointInside(vec{ circle.getPosition().x, circle.getPosition().y }, &shape2)<< std::endl;

			window.draw(a1);
			window.draw(circle);
			window.draw(a2);
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

	return 0;
}

