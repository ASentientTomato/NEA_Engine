#pragma once
#include "Physics.h"

class Camera {
private:
	std::vector<Rigidbody>* world_state;
public:
	geo::vec totalTranslation = { 0,0 };
	float totalRotation = 0;
	//initial zoom must be set!
	float totalZoom = 0;
	geo::vec center;//should be private (temporary)
	std::vector<Shape> displayable;

	//initializer
	Camera(std::vector<Rigidbody>* world_state, geo::vec screen_size);
	Camera() {}	//TODO: remove this.

	//change world camera points to
	void rebind(std::vector<Rigidbody>* world_state);

	//set screen size
	void resize(geo::vec screenSize);

	//
	void prepare();

	//project a single shape to where it would appear from the camera's point of view
	Shape project(const Shape& rigid);

	//translate relative to the position of the camera
	void relativeTranslate(geo::vec vector);

	//translate relative to the position of the world (recommended)
	void objectiveTranslate(geo::vec vector);

	//rotate camera about the center of the screen
	void rotate(float theta);

	//zoom camera to the center of the screen
	void zoom(float zoomFactor);

	//getters for some reason
	geo::vec getTrans() { return totalTranslation; }
	float getRot() { return totalRotation; }
	float getZoom() { return totalZoom; }

	//get original position from translated camera position
	Shape inverse(Shape shape);

	geo::vec inverse(geo::vec vec);
};