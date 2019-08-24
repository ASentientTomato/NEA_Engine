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

}

void World::SetShapes(std::vector<Rigidbody> shapes) {
	this->WORLD_SHAPES = shapes;
}

std::vector<Rigidbody>* World::GetShapes() {
	return &this->WORLD_SHAPES;
}

World::World(std::vector<Rigidbody> shapes, Camera cam) {
	this->WORLD_SHAPES = shapes;
	this->Bind_Camera(cam);
}

void World::PrepareCamera(geo::vec screenDimensions){
	camera.resize(screenDimensions);
	camera.rebind((&WORLD_SHAPES));
}

//void World::Prepare() {
//	this->camera.rebind(&this->WORLD_SHAPES);
//}