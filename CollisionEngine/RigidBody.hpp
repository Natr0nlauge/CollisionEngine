#pragma once

#include<SFML/Graphics.hpp>

class RigidBody : public sf::Shape
{
public:
	//sf::Vector2f m_origin = sf::Vector2f(0.0f, 0.0f);
	//sf::Vector2f m_position = sf::Vector2f(0.0f, 0.0f);
	RigidBody(float i_mass);
	~RigidBody();
	float getInverseMass() const;
	float getInverseMomentOfInertia() const;
	void setVelocity(sf::Vector2f i_newVel);
	void setAngularVelocity(float i_newAngVel);
	sf::Vector2f getVelocity();
	float getAngularVelocity();
	void updatePositionAndAngle(float i_dT); //TODO check if there is a more elegant way for dt

	
protected:
	sf::Vector2f transformPointToGlobal(sf::Vector2f i_localPoint);
	sf::Vector2f transformVectorToGlobal(sf::Vector2f i_localVector);
	virtual sf::Vector2f calculateCenterOfMass() = 0;
	virtual float calculateInverseMomentOfInertia() = 0;
	float calculateInverseDensity() const;
	virtual float calculateArea() = 0;
	//inverse mass is more useful (infinite mass for immovable objects is easier to implement)
	float m_inverseMass; //center of mass is in origin
	float m_inverseMomentOfInertia = 1.0f;
	float m_area = 1.0f;
	sf::Vector2f m_velocity = sf::Vector2f(0.0f,0.0f);
	float m_angularVelocity = 0.0f;

};






