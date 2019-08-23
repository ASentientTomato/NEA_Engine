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

};