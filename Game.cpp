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

/*
inv_MOI calculated wrong
Added "dropped frames" warning to check if I'm being too slow
*/
//n must be even
geo::convex generatePolygon(geo::vec side, int n) {
	float internalAngle = (n - 2) * 3.14f;
	internalAngle *= (1 / (float)n);
	float sine = std::sin(internalAngle);
	float cosine = std::cos(internalAngle);
	geo::convex shape;
	shape.points.resize(n);
	shape.points[0] = { 0,0 };
	shape.points[1] = side;
	for (int i = 2; i < n; i++) {
		side = shape.points[i - 2] - shape.points[i - 1];
		side = shape.points[i - 1] + geo::vec{ cosine * side.x - sine * side.y, sine * side.x + cosine * side.y };
		shape.points[i] = (side);
	}
	shape.clockwise = true;
	return shape;
}

void applyGravity(World& world) {
	for (int i = 0; i < world.size(); i++) {
		//TODO: only perform this on large bodies, by providing a list instead of the whole world
		//alternatively, extend the world class to implement this method.
		//also, there's a more complicated equation for gravity which I don't remember - this is just the proportionality.
		for (int j = i + 1; j < world.size(); j++) {
			geo::vec dist = world[i].getCOM() - world[j].getCOM();
			//j->i
			double magnitude = geo::magnitude(dist);
			double mag = magnitude;
			magnitude = 1 / magnitude;
			//dist = dist * magnitude;
			//magnitude = magnitude * magnitude;
			//magnitude *= (world[i].trans.mass + world[j].trans.mass);
			mag = mag * mag;
			mag = (world[i].trans.mass + world[j].trans.mass) / mag;

			dist = dist * mag;
			geo::printVec(dist);
			world[j].applyForce(dist);
			world[i].applyForce(dist * -1);
		}
	}
}


sf::Vector2f convertVector(geo::vec vector) {
	return sf::Vector2f((float)vector.x, (float)vector.y);
}

using namespace geo;
//-----------------------------------------------GRAPHICS-----------------------------------------------
int main()
{
	//load world
	World* world = loadGameState("data/save.xml");
	if (world == NULL) {	//I'll make it return NULL in the case of an error.
		std::cout << "failed to load";
		exit(-1);
	}
	std::cout << "loaded successfully!\n";
	world->PrepareCamera({ 1920,1080 });


	//*****************EFFICIENCY STUFF*****************
	const sf::Time timeStep = sf::seconds(1.0f / FPS);
	sf::Time accumulator;
	sf::Clock clock;
	std::cout << "time step: " << timeStep.asSeconds() << " seconds" << std::endl;
	std::cout << "1 pixel = 1 meter" << std::endl;

	//*****************GRAPHICS SETUP*****************
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;
	sf::RenderWindow window(sf::VideoMode(SCREEN_X, SCREEN_Y), "test");//, sf::Style::Fullscreen, settings);

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
			std::cout << "dropped frames!\n";
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		accumulator += clock.getElapsedTime();
		clock.restart();
		//std::cout << accumulator.asSeconds() << std::endl;

		//*****************UPDATE WORLD LOGIC*****************
		//	ONLY APPLY FORCES WITHIN THIS LOOP!
		while (accumulator > timeStep) {

			//apply forces
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
				//GLOBAL_BODIES[0].translate(vec{ -1,0 });
				(*world)[0].applyForce(vec{ -3000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
				//square.translate(vec{ 1,0 });
				(*world)[0].applyForce(vec{ 3000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
				(*world)[0].applyForce(vec{ 0,-3000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
				(*world)[0].applyForce(vec{ 0,3000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
				(*world)[0].rotate(0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
				(*world)[0].rotate(-0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::X)) {
				(*world)[0].trans.velocity = { 0,0 };
				(*world)[0].trans.accelaration = { 0,0 };
			}
			bool physics = false;
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z)) {
				physics = true;
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
				world->camera.objectiveTranslate(geo::vec{ 0, 2 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
				world->camera.objectiveTranslate(geo::vec{ 2, 0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
				world->camera.objectiveTranslate(geo::vec{ 0, -2 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
				world->camera.objectiveTranslate(geo::vec{ -2, 0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
				world->camera.rotate(0.05);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::T)) {
				world->camera.rotate(-0.05);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::N)) {
				world->camera.zoom(0.01);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::B)) {
				world->camera.zoom(-0.01);
			}

			accumulator -= timeStep;
			window.clear(sf::Color::Black);


			//time integration
			world->Integrate(timeStep.asSeconds());



			//**************************	PHYSICS		***************************

			manifold myManifold;

			world->GenerateManifolds();
			if (physics) {
				world->ResolveManifolds();
			}

			//**************************	DRAW SHAPES		***************************


			world->camera.prepare();
			for (const Shape& i : world->camera.displayable) {
				window.draw(i.displayable.convex);
			}

			window.display();
		}

	}
	//
	return 0;
}