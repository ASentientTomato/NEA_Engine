#pragma once
#include <SFML/Graphics.hpp>
#include "shapes.h"

namespace Graphics {

	class Displayable {
		sf::ConvexShape convex;
		sf::Texture texture;
		//TODO: include texture, sound, etc.

		Displayable(const geo::convex& convex) {
			//initialize shape
			this->convex.setPointCount(convex.points.size());
			for (int i = 0; i < convex.points.size(); i++) {
				this->convex.setPoint(i, { (float)convex.points[i].x, (float)convex.points[i].y });
			}
		}


	};

	//TODO: add Graphics::Displayable creation functions (from file stream).
}
