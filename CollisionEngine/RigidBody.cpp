#include "RigidBody.hpp"

#define PI  3.14159265358979323846

RigidBody::RigidBody(float i_mass) : Shape(), mass(i_mass) {

}

RigidBody::~RigidBody() {

}

float RigidBody::getMass() const
{
    return mass;
}

float RigidBody::getMomentOfInertia() const
{
    return momentOfInertia;
}

void RigidBody::setVelocity(sf::Vector2f i_newVel) 
{
    velocity = i_newVel;
}

void RigidBody::setAngularVelocity(float i_newAngVel) 
{
    angularVelocity = i_newAngVel;
}



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

float RigidBody::calculateDensity() const
{
    return mass/area;
}







