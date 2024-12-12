#pragma once

#include<SFML/Graphics.hpp>

class RigidBody : public sf::Shape
{
public:
	RigidBody();
	~RigidBody();
	
protected:
	sf::Vector2f transformPointToGlobal(sf::Vector2f i_localPoint);
	sf::Vector2f transformVectorToGlobal(sf::Vector2f i_localVector);

};






