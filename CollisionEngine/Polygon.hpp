#pragma once

#include "RigidBody.hpp"

class Polygon : public RigidBody
{
private:
	std::vector<sf::Vector2f> m_points = { sf::Vector2f(25.0f,25.0f), sf::Vector2f(-25.0f,25.0f), sf::Vector2f(-50.0f,0.0f),sf::Vector2f(-25.0f,-25.0f), sf::Vector2f(25.0f,-25.0f) };
	sf::Vector2f m_origin = sf::Vector2f(0.0f, 0.0f);
	sf::Vector2f m_position = sf::Vector2f(0.0f, 0.0f);
	float calculateSignedArea();
	float calculateArea() override;
	float calculateMomentOfInertia();
	sf::Vector2f calculateCenterOfMass();


public:
	//Polygon(float i_mass);
	Polygon(float i_mass, std::vector<sf::Vector2f> vertices);
	~Polygon();
	sf::Vector2f getPoint(std::size_t index) const;
	std::size_t getPointCount() const;
	std::vector<sf::Vector2f> getPoints();
	sf::Vector2f getGlobalPoint(int i_index);
	sf::Vector2f getNormal(int i_index); //TODO is this necessary?
	sf::Vector2f getGlobalNormal(int i_index);

};

