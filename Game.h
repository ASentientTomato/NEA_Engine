#pragma once
#include "Physics.h"

class Camera {
private:
	std::vector<Rigidbody>* world_state;

	geo::vec totalTranslation = { 0,0 };
	float totalRotation = 0;
	float totalZoom = 1;
public:
	geo::vec center;//should be private (temporary)
	std::vector<Shape> displayable;

	//initializer
	Camera(std::vector<Rigidbody>* world_state, geo::vec screen_size);
	Camera() {}
	void rebind(std::vector<Rigidbody>* world_state);
	void resize(geo::vec screenSize);
	void prepare();
	Shape project(const Shape& rigid);
	void relativeTranslate(geo::vec vector);
	void objectiveTranslate(geo::vec vector);
	void rotate(float theta);
	void zoom(float zoomFactor);
};