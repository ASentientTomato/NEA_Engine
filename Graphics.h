#pragma once
#include <SFML/Graphics.hpp>
#include "shapes.h"

namespace Graphics {

	class Displayable {
	public:

		sf::ConvexShape convex;	//TODO: maybe replace with sprite.
		sf::Texture texture;
		//TODO: include texture, sound, etc.

		Displayable(const geo::convex& convex) {
			//initialize shape
			this->convex.setPointCount(convex.points.size());
			for (int i = 0; i < convex.points.size(); i++) {
				this->convex.setPoint(i, { (float)convex.points[i].x, (float)convex.points[i].y });
			}
		}
		Displayable() {};

		//set convex shape's sides to rigidbody's
		void set_sides(const std::vector<geo::vec>& vec) {
			convex.setPointCount(vec.size());
			for (int i = 0; i < vec.size(); i++) {
				convex.setPoint(i, { (float)vec[i].x, (float)vec[i].y });
			}
		}

		void load_texture(std::string location) {
			this->texture.loadFromFile(location);
			this->convex.setTexture(&this->texture);
		}
	};

	//TODO: add Graphics::Displayable creation functions (from file stream).
}
