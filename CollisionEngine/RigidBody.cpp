#include "RigidBody.hpp"



RigidBody::RigidBody(sf::Vector2f vec) : RectangleShape(vec) {

}

RigidBody::RigidBody() : RectangleShape(sf::Vector2f(100.0f, 100.0f)) {

}

RigidBody::~RigidBody() {

}




Polygon::Polygon(sf::Vector2f vec) : RigidBody(vec) {

}

Polygon::~Polygon() {

}


Ball::Ball() {

}

Ball::~Ball() {

}