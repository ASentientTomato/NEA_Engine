// NEATask.cpp : Defines the entry point for the console application.
//
#pragma once
#include "Game.h"
#include "geometrytools.h"
#include <SFML\Graphics.hpp>

#include <cmath>
#include <fstream>

#define FPS 60
#define SCREEN_X 1400
#define SCREEN_Y 1000
#define MAX_MISSED_TIME 0.05


#include "World.h"
#include "FileHandling.h"

bool isConvex(Shape sh) {
	//A shape is convex if and only if the cross products of every two consecutive sides are either all positive or all negative.
	//side1, side2
	bool positive = (cross(sh.points[0] - sh.points[sh.points.size()-1],sh.points[1]-sh.points[0]) >= 0); //is the cross product positive?

	geo::vec side1;
	geo::vec side2;
	//The two sides that we consider

	bool newCrossIsPositive; //is the new cross product positive?

	for (int i = 2; i < sh.points.size(); i++) {
		side2 = sh.points[i] - sh.points[i - 1];
		side1 = sh.points[i - 1] - sh.points[i - 2];
		newCrossIsPositive = (cross(side1, side2) >= 0);
		if (positive != newCrossIsPositive) {	//They have different signs
			return false;
		}
	}
	return true;
}

/*
Later versions of the code may implement these improvements:
add "int situation" in world - only things in the same situation need to have collision detection run on them. This can be controlled by an enum.
remove camera from world.
add "std::String tosave" to world for saving additional data.
remove "displayable" from world/rigidbody. This should be up for the user to render.
add better world creation tools.
*/


geo::convex generatePolygon(geo::vec side, int n) {
	//Generates an n-sided polygon, where each side has the length of "side".
	/*This is achieved by calculating the internal angle of the desired polygon, then:
	-drawing "side"
	-rotating "side" by that angle
	-drawing "side" again, connected to the end of the previous side.
	*/
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

void applyGravity(World* world) {

	//applies gravity to all shape. Currently, this achieves an unnecessarily high level of realism by considering all objects.
	//This makes it O(n^2) where n is the number of objects. This can be changed in the future.

	for (int i = 0; i < world->size(); i++) {
		//TODO: only perform this on large bodies w/ small bodies, by providing a list instead of the whole world
		//alternatively, extend the world class to implement this method.
		for (int j = i + 1; j < world->size(); j++) {
			geo::vec dist = world->operator[](i).getCOM() - world->operator[](j).getCOM();
			//j->i
			double magnitude = geo::magnitude(dist);
			magnitude = 1 / magnitude;
			dist = dist * magnitude;
			magnitude *= magnitude;

			double top = world->operator[](i).trans.mass * world->operator[](j).trans.mass * 6.67408e-11;	//gravitational constant

			magnitude = top * magnitude;
			dist = dist * magnitude;

			world->operator[](j).applyForce(dist);
			world->operator[](i).applyForce(dist * -1);
		}
	}
}

geo::vec getClickPosition(sf::Window* window, Camera* cam, sf::Mouse::Button button) {

	//Gets the position of a click. Just a utility method.

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
	//Utility method for converting my type of vector to the SFML type of vector.
	return sf::Vector2f((float)vector.x, (float)vector.y);
}

using namespace geo;
//-----------------------------------------------GRAPHICS-----------------------------------------------
int main()
{
	geo::vec screenDimensions = { SCREEN_X, SCREEN_Y };

	//**************LOAD WORLD AND CAMERA**************//
	//load world
	World* world = loadGameState("data/test.xml");
	if (world == NULL) {	//loadGameState() returns NULL in the case of an error.
		std::cout << "failed to load";
		exit(-1);
	}
	std::cout << "world loaded successfully!\n";
	//load camera
	Camera camera = loadCamera("data/test.xml", world->getDataAddress());
	camera.resize(screenDimensions);

	//*****************EFFICIENCY STUFF*****************//
	const sf::Time timeStep = sf::seconds(1.0f / FPS);
	sf::Time accumulator;
	sf::Clock clock;
	std::cout << "time step: " << timeStep.asSeconds() << " seconds" << std::endl;
	std::cout << "1 pixel = 1 meter" << std::endl;

	//*****************GRAPHICS SETUP*****************//
	sf::ContextSettings settings;
	settings.antialiasingLevel = 100;
	sf::RenderWindow window(sf::VideoMode(SCREEN_X, SCREEN_Y), "test", sf::Style::Default, settings);

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

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//this allows measurement of startup time.
		accumulator += clock.getElapsedTime();
		clock.restart();
		//the clock and the accumulator are used to measure framerate and handle dropping frames.

		//*****************UPDATE WORLD LOGIC*****************
		//	ONLY APPLY FORCES WITHIN THIS LOOP!
		while (accumulator > timeStep) {	//This means that if there are multiple frames that haven't yet been processed
											//we should process them all, one after the other.


			if (accumulator.asSeconds() > MAX_MISSED_TIME) {
				std::cout << "dropped " << accumulator.asSeconds() << "seconds!\n";
				accumulator = sf::seconds(MAX_MISSED_TIME);
				//This allows us to drop frames in case the program is already far behind.
			}


			static bool physics = true;	//Should gravity be applied?
			static bool paused = true;	//Should the simulation be paused?

			//set up some rest frames
			static int physicsRestFrames = 0;
			//apply forces (gravity) if Z is pressed.
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z) && physicsRestFrames == 0) {
				physicsRestFrames++;
				physics = !physics;
			}
			if(physicsRestFrames != 0){
				physicsRestFrames++;
			}
			//ensure that physics won't keep triggering itself on and off
			if (physicsRestFrames == 60) {
				physicsRestFrames = 0;
			}


			//Camera controls:translation
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
				camera.objectiveTranslate(geo::vec{ 0, 10 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
				camera.objectiveTranslate(geo::vec{ 10, 0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
				camera.objectiveTranslate(geo::vec{ 0, -10 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
				camera.objectiveTranslate(geo::vec{ -10, 0 });
			}
			
			//Camera controls:rotation
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::T)) {
				camera.rotate(0.025);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
				camera.rotate(-0.025);
			}

			//Camera controls:zoom
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::N)) {
				camera.zoom(0.01);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::B)) {
				camera.zoom(-0.01);
			}

			static bool pPressed = false;
			//Create a regular polygon around the mouse's position.
			//For a sufficiently high point count, or a sufficiently small polygon, this can be really good for approximating a circle.
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::P)) {
				if (!pPressed) {
					//loop left like this in case you want to stress-test the system by making many shapes at once:
					//just change "i<1" to "i<100".
					for (int i = 0; i < 1; i++){
					sf::Vector2i position = sf::Mouse::getPosition(window);
					geo::vec vec = { position.x, position.y };
					geo::convex c = generatePolygon({ 10,0 }, 25);
					Rigidbody r;
					vec = camera.inverse(vec);

					//default properties for the polygon
					r.trans.mass = 25;
					r.displayable.set_sides(c.points);
					r.points = c.points;
					r.trans.inv_mass = 1/r.trans.mass;
					r.rot.momentOfInertia = 10000;
					r.rot.inv_MOI = 1/r.rot.momentOfInertia;
					r.calculateCOM();
					r.clockwise = isClockwise(r);
					r.translate(vec);

					world->push_back(r);
					pPressed = true;
				}
				}
			}
			else { pPressed = false; }
			
			//world editor
			static Rigidbody sh;
			static bool enter_down = false;

			//detect the presense and position of clicks.
			if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
				sf::Vector2i position = sf::Mouse::getPosition(window);
				geo::vec vec = { position.x,position.y };
				vec = camera.inverse(vec);
				bool found = false;
				//ensure that there weren't two clicks on the same point...
				for (geo::vec& v : sh.points) {
					if (v == vec) {
						found = true;
						break;
					}
				}
				//if the point isn't already present, add it to the pending shape.
				if(!found){
					sh.points.push_back(vec);
				}
			}
			//if Enter is pressed, create the shape.
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Enter)) {
				if (!enter_down) {	//ensure Enter isn't held down from a previous frame
					enter_down = true;

					if(sh.getArea() > 0 && isConvex(sh)){	//ensure the shape isn't concave or a line (or self intersecting-
															//self intersecting shapes result in a negative area. 

						//attributes of the newly-created shape.
						sh.trans.mass = 20;
						sh.trans.inv_mass = 1/sh.trans.mass;
						sh.rot.momentOfInertia = 10000;
						sh.rot.inv_MOI = 1/sh.rot.momentOfInertia;
						sh.calculateCOM();
						sh.clockwise = isClockwise(sh);

						sh.displayable.set_sides(sh.points);

						world->push_back(sh);

						sh = Rigidbody();
					}
					else { sh = Rigidbody(); }
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
						(*world)[i].displayable.convex.setFillColor(sf::Color::White);	//Clear all shapes on a right-click,
																						//then decide if the click was in a shape.s
					}
					selected = NULL; 
					for (int i = 0; i < world->size(); i++) {
						if (isPointInside(v, &world->operator[](i))) {	//check if the right click is within an existing shape.
							selected = &(*world)[i];
							break;
						}
					}
				}
			}

			//Increase or decrease mass (H or G respectively)
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::H) && selected != NULL) {
				selected->trans.mass = selected->trans.mass + 10;
				selected->trans.inv_mass = 1 / selected->trans.mass;
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::G) && selected != NULL) {
				selected->trans.mass -= 10;
				selected->trans.inv_mass = 1 / selected->trans.mass;
			}

			//Applies force to the selected shape in the chosen direction. Used for controlling the shape's position.
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left) && selected != NULL && !paused) {
				//GLOBAL_BODIES[0].translate(vec{ -1,0 });
				selected->applyForce(vec{ -6000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right) && selected != NULL && !paused) {
				//square.translate(vec{ 1,0 });
				selected->applyForce(vec{ 6000,0 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up) && selected != NULL && !paused) {
				selected->applyForce(vec{ 0,-6000 });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down) && selected != NULL && !paused) {
				selected->applyForce(vec{ 0,6000 });
			}

			//Rotation controls for the camera
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q) && selected != NULL) {
				selected->rotate(0.03);
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::E) && selected != NULL) {
				selected->rotate(-0.03);
			}
			
			//Translation controls for the camera
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::X) && selected != NULL) {
				selected->trans.velocity = { 0,0 };
				selected->trans.accelaration = { 0,0 };
			}



			//delete shapes if backspace is pressed
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::BackSpace) && selected != NULL) {
				for (int i = 0; i < world->size(); i++) {
					if ((*world)[i].points == selected->points) {
						world->remove(i);
						selected = NULL;
						break;
					}
				}
			}
			//Save to a file if F1 is pressed
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::F1)) {
				if (saveGameState(world, "data/test.xml", camera) >= 0) {
					std::cout << "world saved successfully!" << std::endl;
					exit(1);
				}
				else {
					std::cout << "failed to save..." << std::endl;
				}
			}

			//allow user to pause simulation by pressing F2.
			static int restFrame;
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::F2) && restFrame == 0) {
				paused = !paused;
				restFrame++;
			}
			if(restFrame != 0){
				restFrame++;
			}
			//ensure that the F2 key won't respond for 1 second after it is pressed.
			if (restFrame == 61) {
				restFrame = 0;
			}


			accumulator -= timeStep;
			window.clear(sf::Color::Black);

			//**************************	PHYSICS		***************************



			manifold myManifold;


			//time integration
			if(!paused){
				if (physics) {
					applyGravity(world);
				}
				world->Integrate(timeStep.asSeconds());
				world->GenerateManifolds();
				world->ResolveManifolds();
			}
			//std::vector<int> bruh = makeConvexz(world->operator[](3), 0.001);
			//**************************	DRAW SHAPES		***************************

			camera.prepare();
			if (selected != NULL) {
				selected->displayable.convex.setFillColor(sf::Color::Green);
			}
			for (const Shape& i : camera.displayable) {
				window.draw(i.displayable.convex);
			}
/*			for (const sf::Text& i : text) {
				window.draw(i);
			}*/
			


			window.display();
		}

	}
	//
	return 0;
}