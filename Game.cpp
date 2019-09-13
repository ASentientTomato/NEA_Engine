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
add "int situation" in world - only things in the same situation need to have collision detection run on them. This can be controlled by an enum.
remove camera from world.
add "string tosave" to world for saving additional data.
remove "displayable" from world/rigidbody. This should be up for the user to render.
add world creation tools.
*/
//n must be even
geo::convex generatePolygon(geo::vec side, int n) {
	float internalAngle = (n - 2) * 3.14159265358979323846f;
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
		//TODO: only perform this on large bodies w/ small bodies, by providing a list instead of the whole world
		//alternatively, extend the world class to implement this method.
		for (int j = i + 1; j < world.size(); j++) {
			geo::vec dist = world[i].getCOM() - world[j].getCOM();
			//j->i
			double magnitude = geo::magnitude(dist);
			magnitude = 1 / magnitude;
			dist = dist * magnitude;
			magnitude *= magnitude;

			double top = world[i].trans.mass * world[j].trans.mass * 6.67408e-11;	//gravitational constant

			magnitude = top * magnitude;
			dist = dist * magnitude;

			geo::printVec(dist);
			world[j].applyForce(dist);
			world[i].applyForce(dist * -1);
		}
	}
}

geo::vec getClickPosition(sf::Window* window, Camera* cam, sf::Mouse::Button button) {
	if (sf::Mouse::isButtonPressed(button)) {
		sf::Vector2i position = sf::Mouse::getPosition(*window);
		geo::vec pos = cam->inverse({ (double)position.x, (double)position.y });
		return pos;
	}
	else {
		return { NULL, NULL };
	}
}


sf::Vector2f convertVector(geo::vec vector) {
	return sf::Vector2f((float)vector.x, (float)vector.y);
}

using namespace geo;
//-----------------------------------------------GRAPHICS-----------------------------------------------
int main()
{
	geo::vec screenDimensions = { SCREEN_X, SCREEN_Y };


	//load world
	World* world = loadGameState("data/save.xml");
	if (world == NULL) {	//I'll make it return NULL in the case of an error.
		std::cout << "failed to load";
		exit(-1);
	}
	std::cout << "loaded successfully!\n";
	Camera camera = loadCamera("data/save.xml", world->getDataAddress());
	camera.resize(screenDimensions);

	sf::Font font;
	font.loadFromFile("/data/fonts/AGENCYR.TTF");
	std::vector<sf::Text> text;

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
				(*world)[0].applyForce(vec{ -6000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
				//square.translate(vec{ 1,0 });
				(*world)[0].applyForce(vec{ 6000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
				(*world)[0].applyForce(vec{ 0,-6000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
				(*world)[0].applyForce(vec{ 0,6000 });
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
			
			
			//world editor
			static Rigidbody sh;
			static bool enter_down = false;

			if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
				sf::Vector2i position = sf::Mouse::getPosition(window);
				//sf::Vector2i position = sf::Mouse::getPosition();
				geo::vec vec = { position.x,position.y };
				vec = camera.inverse(vec);
				//TODO: handle concave shapes properly.
				bool found = false;
				for (geo::vec& v : sh.points) {
					if (v == vec) {
						found = true;
						break;
					}
				}
				if(!found){
					sh.points.push_back(vec);
				}
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Enter)) {
				if (!enter_down) {
					enter_down = true;

					if(sh.getArea() > 0){
						sh.trans.mass = DBL_MAX;
						sh.trans.inv_mass = 0;
						sh.rot.momentOfInertia = FLT_MAX;
						sh.rot.inv_MOI = 0;
						sh.calculateCOM();
						sh.clockwise = isClockwise(sh);

						sh.displayable.set_sides(sh.points);

						world->push_back(sh);

						sh = Rigidbody();
					}
				}
			}
			else {
				enter_down = false;
			}

			//Allow a shape to be selected
			static Rigidbody* selected = NULL;
			{
				geo::vec v;
				v = getClickPosition(&window, &camera, sf::Mouse::Button::Right);
				if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)) {
					for (int i = 0; i < world->size(); i++) {
						(*world)[i].displayable.convex.setFillColor(sf::Color::White);
					}
					selected = NULL;
					for (int i = 0; i < world->size(); i++) {
						if (isPointInside(v, &world->operator[](i))) {
							selected = &(*world)[i];
							break;
						}
					}
				}
				else {
					selected = NULL;
				}
			}

			accumulator -= timeStep;
			window.clear(sf::Color::Black);


			//time integration
			world->Integrate(timeStep.asSeconds());



			//**************************	PHYSICS		***************************




			if (text.size() < 1) {
				sf::Text t;
				t.setFont(font);
				t.setString("testing testing 123");
				t.setCharacterSize(240);
				t.setFillColor(sf::Color::Red);
				text.push_back(t);
				std::cout << std::endl <<"HELLO" << std::endl;
			}


			manifold myManifold;

			if(physics){
				applyGravity(*world);
			}

			world->GenerateManifolds();
			world->ResolveManifolds();

			//**************************	DRAW SHAPES		***************************


			camera.prepare();
			for (const Shape& i : camera.displayable) {
				window.draw(i.displayable.convex);
			}
			for (const sf::Text& i : text) {
				window.draw(i);
			}
			
			if (selected != NULL) {
				selected->displayable.convex.setFillColor(sf::Color::Green);
			}

			window.display();
		}

	}
	//
	return 0;
}