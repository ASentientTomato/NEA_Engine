#include "World.h"

void World::Integrate(float dt){
	for (Rigidbody& i : this->WORLD_SHAPES) {
		i.forceIntegration(dt);
	}
}

void World::Bind_Camera(Camera& cam) {
	this->camera = cam;
}

void World::GenerateManifolds() {
	//TODO: perform collision detection
}

void World::SetShapes(std::vector<Rigidbody> shapes) {
	this->WORLD_SHAPES = shapes;
}


World::World(std::vector<Rigidbody> shapes, Camera cam) {
	this->WORLD_SHAPES = shapes;
	this->Bind_Camera(cam);
}

void World::PrepareCamera(geo::vec screenDimensions){
	camera.resize(screenDimensions);
	camera.rebind((&WORLD_SHAPES));
}

Rigidbody& World::operator [] (int position) {
	return WORLD_SHAPES[position];
}

//void World::Prepare() {
//	this->camera.rebind(&this->WORLD_SHAPES);
//}