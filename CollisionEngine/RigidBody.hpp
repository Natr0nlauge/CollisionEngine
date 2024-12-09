#pragma once

#include<SFML/Graphics.hpp>

class RigidBody : public sf::RectangleShape
{
public:
	RigidBody();
	RigidBody(const sf::Vector2f vec);
	~RigidBody();

	

};

class Polygon : public RigidBody, public sf::ConvexShape
{
public:
	Polygon(sf::Vector2f vec);
	~Polygon();
};


class Ball : public RigidBody, public sf::CircleShape
{
public:
	Ball();
	~Ball();
};


