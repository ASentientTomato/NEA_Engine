// NEATask.cpp : Defines the entry point for the console application.
//
#pragma once
#include "Game.h"
#include "geometrytools.h"
#include <SFML\Graphics.hpp>

#include <cmath>

#define FPS 60
#define SCREEN_X 1400
#define SCREEN_Y 1000
#define MAX_MISSED_TIME 0.05


#include "World.h"
#include "FileHandling.h"

int main() {
	World* world = loadGameState("data/save.xml");
	if (world == NULL) {	//I'll make it return NULL in the case of an error.
		std::cout << "failed to load";
		exit(-1);
	}

	std::cout << "loaded successfully!\n";
	world->PrepareCamera({ 1920,1080 });


	saveGameState(world, "data/save2.xml");

	delete world;

	system("PAUSE");
}


/*
std::vector<Rigidbody> GLOBAL_BODIES;
std::vector<sf::ConvexShape> GLOBAL_SHAPES;

struct sfShape {
	Rigidbody* body;
	sf::ConvexShape convex;
};

sf::Vector2f convertVector(geo::vec vector) {
	return sf::Vector2f((float)vector.x, (float)vector.y);
}

using namespace geo;
//-----------------------------------------------GRAPHICS-----------------------------------------------
int main()
{
	Camera camera = Camera(&GLOBAL_BODIES, geo::vec{ SCREEN_X,SCREEN_Y });
	float floats[9] = { 1,0,0,0,1,0,0,0,1 };
	float floats2[9] = { 0,1,2,3,4,5,6,7,8 };

	Matrix3D <float> matrix(floats);
	Matrix3D <float> matrix2(floats2);

	matrix *= matrix2;

	matrix.print();

	vec bruh = { 12,13 };

	geo::printVec(matrix * bruh);

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
		convex shape2;
		square.trans.mass = 10;
		square.trans.inv_mass = 0.1;
		shape2.points = { geo::vec{100, 100} , geo::vec{100,200}, geo::vec{200,200}, geo::vec{200,100} };
		shape2.clockwise = false;
		square.shape = shape2;

		GLOBAL_BODIES.push_back(square);
		GLOBAL_BODIES[0].calculateCOM();
		//GLOBAL_BODIES[0].com = sub(scale(add(GLOBAL_BODIES[0].shape.points[0], GLOBAL_BODIES[0].shape.points[2]), 0.5), GLOBAL_BODIES[0].shape.points[0]);

		Rigidbody rigid = Rigidbody();
		rigid.trans.mass = DBL_MAX;
		rigid.trans.inv_mass = 0;//.1;
		rigid.shape.points = { geo::vec{100,100}, geo::vec{200,100}, geo::vec{470,255}, geo::vec{327,800}, geo::vec{157,550}, geo::vec{100,300} };
		rigid.shape.clockwise = true;
		rigid.rot.inv_MOI = 0;
		rigid.rot.momentOfInertia = 999999999999999999;



		GLOBAL_BODIES.push_back(rigid);
		GLOBAL_BODIES[1].calculateCOM();
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

	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;
	sf::RenderWindow window(sf::VideoMode(SCREEN_X, SCREEN_Y), "test", sf::Style::Fullscreen, settings);

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
				GLOBAL_BODIES[0].applyForce(vec{ -3000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
				//square.translate(vec{ 1,0 });
				GLOBAL_BODIES[0].applyForce(vec{ 3000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
				GLOBAL_BODIES[0].applyForce(vec{ 0,-3000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
				GLOBAL_BODIES[0].applyForce(vec{ 0,3000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
				GLOBAL_BODIES[0].rotate(0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
				GLOBAL_BODIES[0].rotate(-0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::X)) {
				GLOBAL_BODIES[0].trans.velocity = { 0,0 };
				GLOBAL_BODIES[0].trans.accelaration = { 0,0 };
			}
			bool physics = false;
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z)) {
				physics = true;
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
				camera.objectiveTranslate(geo::vec{ 0, 2 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
				camera.objectiveTranslate(geo::vec{ 2, 0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
				camera.objectiveTranslate(geo::vec{ 0, -2 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
				camera.objectiveTranslate(geo::vec{ -2, 0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
				camera.rotate(0.05);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::T)) {
				camera.rotate(-0.05);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::N)) {
				camera.zoom(0.01);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::B)) {
				camera.zoom(-0.01);
			}
			//geo::printVec(GLOBAL_BODIES[0].trans.accelaration);

			accumulator -= timeStep;
			window.clear(sf::Color::Black);
			//***************************DRAW SHAPES***************************
			GLOBAL_BODIES[0].forceIntegration(timeStep.asSeconds());
			GLOBAL_BODIES[1].forceIntegration(timeStep.asSeconds());

			//convex shape;
			//shape.clockwise = true;
			//shape.points = { vec { a1.getPoint(0).x , a1.getPoint(0).y }, vec{a1.getPoint(1).x, a1.getPoint(1).y}, vec {a1.getPoint(2).x, a1.getPoint(2).y},
			//vec{ a1.getPoint(3).x , a1.getPoint(3).y }, vec{ a1.getPoint(4).x , a1.getPoint(4).y }, vec{ a1.getPoint(5).x , a1.getPoint(5).y } };




			sf::Vertex referenceLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };
			sf::Vertex incidentLine[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };
			sf::Vertex impulse[2] = { sf::Vector2f {0, 0}, sf::Vector2f{0,0} };



			manifold myManifold;

			collision man;
			line inc;
			line ref;
			vec dist = getCollisionData(GLOBAL_BODIES[1].shape, GLOBAL_BODIES[0].shape, man, inc, ref);
			if (dist.x == 0 && dist.y == 0) {
				myManifold.col = man;
				myManifold.A = &GLOBAL_BODIES[1];
				myManifold.B = &GLOBAL_BODIES[0];
				//vec bruh = 
				if (physics) {
					resolveManifold(myManifold);
				}
				//impulse[0].position = { (float)myManifold.col.contacts[0].position.x, (float)myManifold.col.contacts[0].position.y };
				//bruh = geo::add(myManifold.col.contacts[0].position, bruh);
				//impulse[0].position = { (float)bruh.x, (float)bruh.y };
				//impulse[0].color = sf::Color::Blue;
				//impulse[1].color = sf::Color::Blue;


				std::cout << std::endl;
				//printVec(man.contacts[0].normal);
				//printVec(man.contacts[1].normal);
				point.setPosition(sf::Vector2f{ (float)man.contacts[0].position.x, (float)man.contacts[0].position.y });
				if (man.contactcount > 1) {
					point2.setPosition(sf::Vector2f{ (float)man.contacts[1].position.x, (float)man.contacts[1].position.y });
				}
				else {
					point2.setPosition(sf::Vector2f{ 0,0 });
				}
				incidentLine[0] = sf::Vector2f((float)inc.start.x, (float)inc.start.y);
				incidentLine[1] = sf::Vector2f((float)(inc.start + inc.direction).x, (float)(inc.start + inc.direction).y);
				incidentLine[0].color = sf::Color::Red;
				incidentLine[1].color = sf::Color::Red;

				referenceLine[0] = sf::Vector2f((float)ref.start.x, (float)ref.start.y);
				referenceLine[1] = sf::Vector2f((float)(ref.start + ref.direction).x, (float)(ref.start + ref.direction).y);
				referenceLine[0].color = sf::Color::Green;
				referenceLine[1].color = sf::Color::Green;
			}
			else {
				point.setPosition(sf::Vector2f{ 0,0 });
				point2.setPosition(sf::Vector2f{ 0,0 });
			}
			for (int k = 0; k < GLOBAL_BODIES.size(); k++) {
				int j = 0;
				for (geo::vec i : GLOBAL_BODIES[k].shape.points) {
					GLOBAL_SHAPES[k].setPoint(j, sf::Vector2f{ (float)i.x, (float)i.y });
					j++;
				}
			}

			//draw shapes
			camera.prepare();
			sf::ConvexShape convex;
			for (Shape i : camera.displayable) {
				convex.setPointCount(i.shape.points.size());
				for (int j = 0; j < i.shape.points.size(); j++) {
					convex.setPoint(j, convertVector(i.shape.points[j]));
				}
				window.draw(convex);
			}

			circle.setPosition(convertVector(camera.center));//temporary
			window.draw(circle);
			window.draw(referenceLine, 2, sf::Lines);
			window.draw(incidentLine, 2, sf::Lines);
			window.draw(point);
			window.draw(point2);
			window.draw(impulse, 2, sf::Lines);

			if (man.contactcount > 0) {
				sf::Vertex pen1[2];
				pen1[0] = point.getPosition();
				pen1[0].color = sf::Color::Green;
				pen1[1].color = sf::Color::Green;
				vec p1 = man.contacts[0].normal * man.contacts[0].penetration;
				sf::Vector2f sfv = point.getPosition();
				pen1[1] = sf::Vector2f{ (float)sfv.x + (float)p1.x, (float)sfv.y + (float)p1.y };
				window.draw(pen1, 2, sf::Lines);
			}
			if (man.contactcount == 2) {
				sf::Vertex pen2[2];
				pen2[0] = point2.getPosition();
				pen2[0].color = sf::Color::Magenta;
				pen2[1].color = sf::Color::Magenta;
				vec p1 = man.contacts[1].normal * man.contacts[1].penetration;
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
*/