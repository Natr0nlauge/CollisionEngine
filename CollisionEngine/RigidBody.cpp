#include "RigidBody.hpp"
#include <iostream> //TODO remove this after debugging
#include "sfmlUtility.hpp"

const float PI = 3.14159265358979323846;

RigidBody::RigidBody(float i_inverseMass) : Shape(), m_inverseMass(i_inverseMass) {

}

RigidBody::~RigidBody() {

}

float RigidBody::getInverseMass() const
{
    return m_inverseMass;
}

float RigidBody::getInverseMomentOfInertia() const
{
    return m_inverseMomentOfInertia;
}

void RigidBody::setVelocity(sf::Vector2f i_newVel) 
{
    m_velocity = i_newVel;
}

void RigidBody::setAngularVelocity(float i_newAngVel) 
{
    m_angularVelocity = i_newAngVel;
}

sf::Vector2f RigidBody::getVelocity()
{
    return m_velocity;
}

float RigidBody::getAngularVelocity()
{
    return m_angularVelocity;
}

void RigidBody::updatePositionAndAngle(float i_dT)
{
    move(m_velocity.x * i_dT, m_velocity.y * i_dT);
    rotate(m_angularVelocity * i_dT);
    //std::cout << transformPointToGlobal(getPoint(0)).x << ", " << transformPointToGlobal(getPoint(0)).x << "\n";
 }

// Local to global
sf::Vector2f RigidBody::transformPointToGlobal(sf::Vector2f i_localPoint){   
    // Body position will be the new origin
    sf::Vector2f localOrigin = getPosition();
    float angle = getRotation() /* /*PI / 180*/  ;
    return sfu::transformPoint(i_localPoint, localOrigin, angle);
}

// Local to global
sf::Vector2f RigidBody::transformVectorToGlobal(sf::Vector2f i_localVector) {
    // Multiply with rotation matrix
    float angle = getRotation() /** PI / 180*/;
    return sfu::rotateVector(i_localVector, angle);
}

float RigidBody::calculateInverseDensity() const
{
    return m_area*m_inverseMass;
}









