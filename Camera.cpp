#pragma once
#include "Game.h"




	Camera::Camera(std::vector<Rigidbody>* world_state, geo::vec screen_size) {
		this->rebind(world_state);
		this->resize(screen_size);
	}

	//bind camera to new world (just in case I have those)
	void Camera::rebind(std::vector<Rigidbody>* world_state) {
		this->world_state = world_state;
	}

	//call this whenever the screen is resized
	void Camera::resize(geo::vec screenSize) {
		this->center = { screenSize.x / 2, screenSize.y / 2 };
	}

	void Camera::prepare() {
		this->displayable.clear();
		Shape shape;
		for (const Rigidbody& i : (*world_state)) {
			shape.points = i.points;
			shape.displayable = i.displayable;
			displayable.push_back(shape);
			//TOOD: implement better selection (far-away shapes need not be pushed here).
		}
		//Order of operations: Rotate->Translate->Zoom.
		for (Shape& i : displayable) {
			i = project(i);
		}

	}

	Shape Camera::project(const Shape& rigid) {
		Shape shape = rigid;

		shape.translate(this->totalTranslation);
		shape.zoom(this->totalZoom, this->center);

		shape.rotate(this->totalRotation, this->center);

		shape.displayable.set_sides(shape.points);

		return shape;
	}

	
	Shape Camera::inverse(Shape shape) {

		shape.rotate(totalRotation * -1, center);
		shape.zoom(1 / totalZoom, center);

		//this will probably not work...
		shape.translate(totalTranslation * -1);
	
		return shape;
	}

	geo::vec Camera::inverse(geo::vec vec) {
		Shape s;
		s.points.push_back(vec);
		s = this->inverse(s);
		return s.points[0];

		//TODO: this is a bit of a silly method. Change it.
	}


	//pan camera relative to the camera
	void Camera::relativeTranslate(geo::vec vector) {
		double magnitude = geo::magnitude(vector);
		float cosine = cos(-this->totalRotation);
		float sine = sin(-this->totalRotation);
		vector = geo::vec{ cosine * vector.x - sine * vector.y, sine * vector.x + cosine * vector.y };
		this->totalTranslation += vector;
		//TODO: translate backgrounds
	}

	//pan camera relative to the world
	void Camera::objectiveTranslate(geo::vec vector) {
		this->totalTranslation += vector;
		//TODO: translate backgrounds
	}

	//rotate camera about screen's center
	void Camera::rotate(float theta) {
		this->totalRotation += theta;
		if (this->totalRotation > 360) {
			this->totalRotation -= 360;
		}
		//TODO: rotate backgrounds
	}

	//zoom camera to center of screen
	void Camera::zoom(float zoomFactor) {
		this->totalZoom += zoomFactor;
		if (this->totalZoom < 0) {
			this->totalZoom = 0;
		}
		//TODO: zoom backgrounds
	}



//TODO: rotation not working.


//TODO: rotation not working.
