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