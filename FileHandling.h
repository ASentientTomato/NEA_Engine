#pragma once
//external xml library
#include "pugixml.hpp"

#include "World.h"

//#define SAVE_DIRECTORY "resources/saves"
//#define IMAGE_DIRECTORY "resources/images"

//PNG is probably the best texture format
//I will use XML as my format.

//TODO: this should be opened as doubles, not as text!

//plan:
//pugixml -> data, directory of texture/sounds
//sfml -> loading from texture/sound directory

//doubles and floats are to be stored in exponential notation.




//convert string in the format
//		"(x,y)"				
//		into a vector.
geo::vec readVec(const std::string& str) {
	geo::vec vec;

	int i = 0;
	std::string result;
	for (; i < str.length(); i++) {	//I wrote this code at 2am, so it was pretty buggy
		if (str[i] == '(') continue; 
		if (str[i] == ',') break;
		result += str[i];
	}

	vec.x = std::stod(result.c_str());

	result = "";
	for (int j = i; j < str.length(); j++) {
		if (str[j] == ')') { continue; }
		if (str[j] == ',') { continue; }
		result += str[j];
	}
	
	vec.y = std::stod(result.c_str());

	return vec;
}

//convert vector into string in the format 
//				"(x,y)"					//
inline std::string writeVec(const geo::vec& vec) {
	return ("(" + std::to_string(vec.x) + "," + std::to_string(vec.y) + ")");
}

World* loadGameState(std::string location) {
	
	pugi::xml_document save;
	pugi::xml_parse_result result = save.load_file(location.c_str());
	if (result) {
		std::cout << "XML parsed successfully\n";
	}
	else {
		std::cout << "XML parsing error\n";
	}


	//TODO:load background

	//read rigidbodies
	std::vector<Rigidbody> objects;
	for (pugi::xml_node body = save.child("Universe").child("Bodies").child("Body"); body; body = body.next_sibling("Body")) {
		Rigidbody rigid;

		{
			//load mass
			std::string mass = body.attribute("Mass").value();
			if (mass == "INFINITY") {
				rigid.trans.mass = DBL_MAX;	//std::numeric_limits<float>::infinity();	//TODO: test if this raises errors		<- it does
				rigid.trans.inv_mass = 0;
			}
			else {
				rigid.trans.mass = std::stof(mass.c_str());
				rigid.trans.inv_mass = 1 / rigid.trans.mass;
			}
		}

		//load moment of inertia
		{
			std::string momentOfInertia = body.attribute("MOI").value();
			if (momentOfInertia == "INFINITY") {
				//rigid.rot.momentOfInertia = std::numeric_limits<float>::infinity(); <- TODO: This should work. But it doesn't. Tested by changing the Game.cpp rigidbody to have INFINITY mass instead of DBL_MAX. Things teleport.
				rigid.rot.momentOfInertia = FLT_MAX;
				rigid.rot.inv_MOI = 0;
			}
			else {
				rigid.rot.momentOfInertia = std::stof(momentOfInertia.c_str());
				rigid.rot.inv_MOI = 1 / rigid.trans.mass;
			}
		}

		//load angle
		{
			std::string angle = body.attribute("Angle").value();
			rigid.rot.angle = std::stof(angle.c_str());
			//TODO: normalise angle, probably.
		}

		//load restitution
		{
			std::string restitution = body.attribute("Restitution").value();
			rigid.restitution = std::stof(restitution.c_str());
		}
		
		//load friction
		{
			std::string dynamicFriction = body.attribute("DFriction").value();
			std::string staticFriction = body.attribute("SFriction").value();
			rigid.static_friction = std::stof(staticFriction.c_str());
			rigid.dynamic_friction = std::stof(dynamicFriction.c_str());
		}

		//load velocity
		{
			std::string velocity = body.attribute("Velocity").value();
			rigid.trans.velocity = readVec(velocity);
		}

		//load accelaration (less necessary, but why not?)
		{
			std::string acc = body.attribute("Accelaration").value();
			rigid.trans.accelaration = readVec(acc);
		}

		//load (bool)clockwise
		{
			std::string clockwise = body.attribute("Clockwise").value();
			rigid.shape.clockwise = (clockwise == "1");
		}

		//load sides
		{
			std::string sides = body.attribute("Sides").value();
			std::string vec;
			std::vector<geo::vec> vectors;

			int i = 0;
			while (i < sides.length()) {
				vec += sides[i];
				if (sides[i] == ')') {
					vectors.push_back(readVec(vec));
					vec = "";
				}
				i++;
			}
			//update shape points
			rigid.shape.points.clear();
			vectors.resize(vectors.size());
			rigid.shape.points = vectors;
			//update displayable points
			rigid.displayable.set_sides(vectors);
		}

		//load texture
		//{
		//	std::string textureLocation = body.attribute("Texture").value();
		//	rigid.displayable->load_texture(textureLocation);
		//}

		//commit object
		rigid.calculateCOM();	//TODO: maybe store this?
		objects.push_back(rigid);
	}

	//TODO:
	//load sound directory
	//load backgrounds
	//...etc.
	
	//load camera
	Camera camera;
	{
		pugi::xml_node cam = save.child("Universe").child("Camera");
		std::string translate = cam.attribute("Translation").value();
		std::string rotate = cam.attribute("Rotation").value();
		std::string scale = cam.attribute("Zoom").value();
		camera.objectiveTranslate(readVec(translate));
		camera.rotate(std::stof(rotate.c_str()));
		camera.zoom(std::stof(scale.c_str()));
	}

	//create world
	World* world = new World(objects, camera);
	//TODO: don't forget to deallocate!

	return world;
}
//save game state. returns negative number for error.
int saveGameState(World* world, std::string location) {

	//load file
	pugi::xml_document save = pugi::xml_document();
	pugi::xml_node universe = save.append_child("Universe");

	//create fields
	pugi::xml_node camera = universe.append_child("Camera");
	pugi::xml_node bodies = universe.append_child("Bodies");
	
	//save camera
	{
		pugi::xml_attribute attribute = camera.append_attribute("Translation");
		attribute.set_value(writeVec(world->camera.totalTranslation).c_str());
		attribute = camera.append_attribute("Rotation");
		attribute.set_value(world->camera.totalRotation);
		attribute = camera.append_attribute("Zoom");
		attribute.set_value(world->camera.totalZoom);
	}
	//save bodies
	{
		pugi::xml_node body;
		pugi::xml_attribute att;
		for (int i = 0; i < world->size(); i++) {
			body = bodies.append_child("Body");
			//save attributes
			body.append_attribute("Mass").set_value((*world)[i].trans.mass);
			body.append_attribute("MOI").set_value((*world)[i].rot.momentOfInertia);
			body.append_attribute("Angle").set_value((*world)[i].rot.angle);
			body.append_attribute("Restitution").set_value((*world)[i].restitution);
			body.append_attribute("DFriction").set_value((*world)[i].dynamic_friction);
			body.append_attribute("SFriction").set_value((*world)[i].static_friction);
			body.append_attribute("Velocity").set_value(writeVec((*world)[i].trans.velocity).c_str());
			body.append_attribute("Accelaration").set_value(writeVec((*world)[i].trans.accelaration).c_str());
			body.append_attribute("Clockwise").set_value((*world)[i].shape.clockwise);
			//save sides
			std::string sides = "";
			for (const geo::vec& j : (*world)[i].shape.points) {
				sides = sides += writeVec(j);
			}
			body.append_attribute("Sides").set_value(sides.c_str());

		}
	}
	
	if (save.save_file(location.c_str())) return 0;
	else return -1;
}