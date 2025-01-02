#include "RigidBody.hpp"
#include <iostream> //TODO remove this after debugging

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

void RigidBody::updatePositionAndAngle(float i_dT)
{
    move(m_velocity.x * i_dT, m_velocity.y * i_dT);
    rotate(m_angularVelocity * i_dT);
    //std::cout << transformPointToGlobal(getPoint(0)).x << ", " << transformPointToGlobal(getPoint(0)).x << "\n";
 }

//void RigidBody::updatePositionAndAngle(float i_dT) {
//
//}



sf::Vector2f RigidBody::transformPointToGlobal(sf::Vector2f i_localPoint)
{
    sf::Vector2f globalPoint;
    
    globalPoint = transformVectorToGlobal(i_localPoint);

    //add position to account for translation
    globalPoint.x += getPosition().x;
    globalPoint.y += getPosition().y;
    
    return globalPoint;
}

sf::Vector2f RigidBody::transformVectorToGlobal(sf::Vector2f i_localVector) {
    //multiply with rotation matrix
    sf::Vector2f globalVector;
    float theta = getRotation() * PI / 180;
    globalVector.x = i_localVector.x * cos(theta) - i_localVector.y * sin(theta);
    globalVector.y = i_localVector.x * sin(theta) + i_localVector.y * cos(theta);
    return globalVector;
}

float RigidBody::calculateInverseDensity() const
{
    return m_area*m_inverseMass;
}









