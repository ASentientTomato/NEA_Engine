#pragma once
#include "Physics.h"
#include "Game.h"
class World{
	std::vector<Rigidbody> WORLD_SHAPES;
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

	//Get access to WORLD_SHAPES
	Rigidbody& operator [] (int position);

	//Initialise world with shape list and camera
	World(std::vector<Rigidbody> shapes, Camera cam);

	//Initialise camera
	void PrepareCamera(geo::vec screenDimensions);
};