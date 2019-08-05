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
};
struct centerOfMass {
	geo::vec current;
	geo::vec previous;

	//true if object is behaving properly, false if it disobeyed the laws of physics in the last frame
	//(i.e. teleportation, appearing out of nowhere, and any other interaction not governed by forces)
	bool consistent;
};

class Rigidbody {
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
	//two needed for verlet integration.

	centerOfMass com;

	geo::convex shape;
	float restitution;


	Rigidbody(geo::convex shape, translation trans, rotation rot, centerOfMass com, float resitution) {
		this->shape = shape;
		this->trans = trans;
		this->rot = rot;
		this->com = com;
		this->restitution = restitution;
	}

	Rigidbody() {
	}


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
		this->trans.accelaration = geo::add(this->trans.accelaration, geo::scale(toApply, this->trans.inv_mass));
	}

	void forceIntegration(float dt) {

#ifdef VERLET_INTEGRATE
		//this code is completely wrong
		dt *= dt;
		if (!this->com.consistent) {
			geo::vec newCOM = sub(scale(this->com.current, 2), this->com.previous);
			newCOM = add(newCOM, scale(this->trans.accelaration, dt));
		}
		else {
			this->trans.velocity = scale(geo::sub(this->com.current, this->com.previous), 1 / (2 * sqrt(dt)));
			this->com.previous = this->com.current;
			geo::vec newCOM = add(this->com.previous, scale(this->trans.velocity, dt));
			newCOM = add(newCOM, scale(this->trans.accelaration, 0.5*dt));
			this->com.current = newCOM;
		}
		geo::printVec(sub(this->com.current, com.previous));

		geo::vec a = sub(this->com.current, this->com.previous);
		this->force_translate(a);
		this->trans.velocity = scale(a, (float)1 / (float)60);
		this->com.consistent = true;
		geo::scale(this->trans.accelaration, 0);
#endif	


#ifdef SYMPLECTIC_EULER
		//¯\_(ツ)_/¯//
		this->trans.velocity = geo::add(this->trans.velocity, scale(this->trans.accelaration, dt));
		this->force_translate(geo::scale(this->trans.velocity, dt));
		this->trans.accelaration = geo::scale(this->trans.accelaration, 0);

		this->rot.angularVelocity = this->rot.angularVelocity + (dt * this->rot.accelaration);
		this->force_rotate(this->rot.angularVelocity * dt);
		this->rot.accelaration = 0;
#endif

	}

};


struct manifold {
	int shapes[2];	//this is the index of the two colliding bodies on the global list.
	geo::collision col;
};






std::vector<Rigidbody> GLOBAL_BODIES;
std::vector<sf::ConvexShape> GLOBAL_SHAPES;


using namespace geo;
//-----------------------------------------------GRAPHICS-----------------------------------------------
int main()
{
	//TODO: initialise global bodies by reading from file
	{
		Rigidbody square = Rigidbody();
		square.trans.mass = 10;
		square.trans.inv_mass = (0.1);
		convex shape2;

		shape2.points = { geo::vec{100, 100} , geo::vec{100,200}, geo::vec{200,200}, geo::vec{200,100} };
		shape2.clockwise = false;
		square.shape = shape2;
		square.com.previous = scale(add(square.shape.points[0], square.shape.points[2]), 0.5);

		GLOBAL_BODIES.push_back(square);


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

	//clockwise




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

		if (accumulator.asSeconds() > MAX_MISSED_TIME) {
			accumulator = sf::seconds(MAX_MISSED_TIME);
		}

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



			GLOBAL_BODIES[0].com.current = scale(add(GLOBAL_BODIES[0].shape.points[0], GLOBAL_BODIES[0].shape.points[2]), 0.5);


			//apply forces
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
				//GLOBAL_BODIES[0].translate(vec{ -1,0 });
				GLOBAL_BODIES[0].applyForce(vec{ -1000, 0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
				//GLOBAL_BODIES[0].translate(vec{ 1,0 });
				GLOBAL_BODIES[0].applyForce(vec{ 1000, 0 });

			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
				//GLOBAL_BODIES[0].translate(vec{ 0,-1 });
				GLOBAL_BODIES[0].applyForce(vec{ 0, -1000 });

			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
				//GLOBAL_BODIES[0].translate(vec{ 0,1 });
				GLOBAL_BODIES[0].applyForce(vec{ 0, 1000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
				GLOBAL_BODIES[0].rotate(0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
				GLOBAL_BODIES[0].rotate(-0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::C)) {
			}


			accumulator -= timeStep;
			window.clear(sf::Color::Black);
			//***************************DRAW SHAPES***************************

			sf::Vertex referenceLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };
			sf::Vertex incidentLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };



			GLOBAL_BODIES[0].forceIntegration((float)1 / 60);
			geo::printVec(GLOBAL_BODIES[0].shape.points[0]);
			std::cout << std::endl;
			geo::printVec(GLOBAL_BODIES[0].trans.velocity);

			for (int i = 0; i < GLOBAL_BODIES.size(); ++i) {
				for (int j = 0; j < GLOBAL_BODIES[i].shape.points.size(); ++j) {
					GLOBAL_SHAPES[i].setPoint(j, sf::Vector2f((float)GLOBAL_BODIES[i].shape.points[j].x, (float)GLOBAL_BODIES[i].shape.points[j].y));
				}
			}


			window.draw(circle);
			window.draw(referenceLine, 2, sf::Lines);
			window.draw(incidentLine, 2, sf::Lines);
			window.draw(point);
			window.draw(point2);
			for (sf::ConvexShape con : GLOBAL_SHAPES) {
				window.draw(con);
			}
			window.display();
		}

	}
	return 0;
}