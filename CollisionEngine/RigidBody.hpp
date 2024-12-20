#pragma once

#include<SFML/Graphics.hpp>

class RigidBody : public sf::Shape
{
public:
	RigidBody(float i_mass);
	~RigidBody();
	float getMass() const;
	float getMomentOfInertia() const;
	void setVelocity(sf::Vector2f i_newVel);
	void setAngularVelocity(float i_newAngVel);

	
protected:
	sf::Vector2f transformPointToGlobal(sf::Vector2f i_localPoint);
	sf::Vector2f transformVectorToGlobal(sf::Vector2f i_localVector);
	virtual sf::Vector2f calculateCenterOfMass() = 0;
	virtual float calculateMomentOfInertia() = 0;
	float calculateDensity() const;
	virtual float calculateArea() = 0;
	float mass; //center of mass is in origin
	float momentOfInertia = 1.0f;
	float area = 1.0f;
	sf::Vector2f velocity = sf::Vector2f(0.0f,0.0f);
	float angularVelocity = 0.0f;

};






