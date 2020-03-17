#pragma once
#include "Physics.h"
#include "Game.h"
class World {
	std::vector<Rigidbody> WORLD_SHAPES;
public:
	std::vector<manifold> manifolds;

	//Advance the simulation by one timestep of length dt
	void Integrate(float dt);
	
	//Run collision detection on all shapes in the world
	void GenerateManifolds();

	//Run impulse resolution on all manifolds
	void ResolveManifolds();

	//WORLD_SHAPES setter
	void SetShapes(std::vector<Rigidbody> shapes);

	//Get access to WORLD_SHAPES
	Rigidbody& operator [] (int position);

	//Get address of WORLD_SHAPES
	std::vector<Rigidbody>* getDataAddress();

	void push_back(Rigidbody r);

	//Get access to WORLD_SHAPES.size()
	size_t size();

	void remove(int i);

	//Initialise world with shape list
	World(std::vector<Rigidbody> shapes);
};

