#pragma once

#include<SFML/Graphics.hpp>

class RigidBody : public sf::Shape
{
public:
	RigidBody();
	//RigidBody(const sf::Vector2f vec);
	~RigidBody();
	
	

};

class Polygon : public RigidBody
{
private: 
	std::vector<sf::Vector2f> m_points = {sf::Vector2f(25.0f,25.0f), sf::Vector2f(-25.0f,25.0f), sf::Vector2f(-50.0f,0.0f),sf::Vector2f(-25.0f,-25.0f), sf::Vector2f(25.0f,-25.0f)};
	sf::Vector2f m_origin = sf::Vector2f(0.0f,0.0f);
	sf::Vector2f m_position = sf::Vector2f(0.0f, 0.0f);
	sf::Texture m_texture;
public:
	Polygon();
	Polygon(std::vector<sf::Vector2f> vertices);
	~Polygon();
	std::vector<sf::Vector2f> getPoints();
	//void;
	sf::Vector2f getPoint(std::size_t index) const;
	std::size_t getPointCount() const;

};



class Ball : public RigidBody
{
public:
	Ball();
	~Ball();
};


