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
	//TODO: Get an algorithm that isn't O(n^2) - this one is only for testing
	//TODO: add a pointer to the manifold in the rigidbody class - sometimes it is needed to do something special in the case of a collision (for instance, in a game health might be lost when colliding with a bullet).

	geo::collision man;
	geo::line inc;
	geo::line ref;
	manifold myManifold;
	std::vector<manifold> manifolds;
	
	for (int i = 0; i < WORLD_SHAPES.size(); i++) {
		for (int j = i + 1; j < WORLD_SHAPES.size(); j++) {
			geo::vec dist = geo::getCollisionData(WORLD_SHAPES[i].shape, WORLD_SHAPES[j].shape, man, inc, ref);
			if (dist.x == 0 && dist.y == 0) {
				myManifold.col = man;
				myManifold.A = &(WORLD_SHAPES[i]);
				myManifold.B = &(WORLD_SHAPES[j]);
				manifolds.push_back(myManifold);
			}
		}
	}

	this->manifolds = manifolds;

}

void World::ResolveManifolds() {
	for (const manifold& i : this->manifolds) {
		resolveManifold(i);
	}
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

void World::push_back(Rigidbody r) {
	WORLD_SHAPES.push_back(r);
}

size_t World::size(){
	return WORLD_SHAPES.size();
}

//void World::Prepare() {
//	this->camera.rebind(&this->WORLD_SHAPES);
//}