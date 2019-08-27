#pragma once
#include "Physics.h"
#include "Game.h"
class World : public std::vector<Rigidbody> {	//ok, apparently this is very, very bad practice.
public:
	Camera camera;

	//Advance the simulation by one timestep of length dt
	void Integrate(float dt);
	
	//Set camera
	void Bind_Camera(Camera& cam);

	//Run collision detection on all shapes in the world
	void GenerateManifolds();

	//WORLD_SHAPES setter
	void SetShapes(std::vector<Rigidbody> shapes);

	//Initialise world with shape list and camera
	World(std::vector<Rigidbody> shapes, Camera cam);

	//Initialise camera
	void PrepareCamera(geo::vec screenDimensions);
};