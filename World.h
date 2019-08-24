#pragma once
#include "Physics.h"
#include "Game.h"
class World {
	std::vector<Rigidbody> WORLD_SHAPES;
	Camera camera;

public:
	void Integrate(float dt);
	void Bind_Camera(Camera& cam);
	void GenerateManifolds();
	void SetShapes(std::vector<Rigidbody> shapes);
	std::vector<Rigidbody>* GetShapes();
	World(std::vector<Rigidbody> shapes, Camera cam);
//	void Prepare();
};