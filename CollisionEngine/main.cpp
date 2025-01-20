#include <iostream>
#include <SFML/Graphics.hpp>
#include "Simulation.hpp"

int main() {
    std::vector<RigidBody *> collisionPartners;

	// std::vector<sf::Vector2f> exampleVertices = { sf::Vector2f(25.0f, -50.0f), sf::Vector2f(-25.0f, -50.0f),sf::Vector2f(-50.0f, 0.0f),
    // sf::Vector2f(-50.0f, 25.0f), sf::Vector2f(25.0f, 25.0f)     };
    std::vector<sf::Vector2f> exampleVertices = {sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-25.0f, 25.0f),
            sf::Vector2f(25.0f, 25.0f)};
    collisionPartners.push_back(new Polygon(0.1, exampleVertices));
    // std::vector<sf::Vector2f> exampleVertices2 = { sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-50.0f, 0.0f),
    // sf::Vector2f(-75.0f, 225.0f), sf::Vector2f(25.0f, 25.0f)     };
    std::vector<sf::Vector2f> exampleVertices2 = {sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-25.0f, 25.0f),
            sf::Vector2f(25.0f, 25.0f)};
    collisionPartners.push_back(new Circle(0.1, 25));

    // Simulation border TODO: Replace with a seperate class
    std::vector<sf::Vector2f> borderVertices3 = {sf::Vector2f(0.0f, 50.0f), sf::Vector2f(450.0f, 50.0f), sf::Vector2f(450.0f, 0.0f),
            sf::Vector2f(0.0f, 0.0f)};
    collisionPartners.push_back(new Polygon(0.0, borderVertices3));
    std::vector<sf::Vector2f> borderVertices4 = {sf::Vector2f(0.0f, 50.0f), sf::Vector2f(450.0f, 50.0f), sf::Vector2f(450.0f, 0.0f),
            sf::Vector2f(0.0f, 0.0f)};
    collisionPartners.push_back(new Polygon(0.0, borderVertices4));
    std::vector<sf::Vector2f> borderVertices5 = {sf::Vector2f(0.0f, 450.0f), sf::Vector2f(50.0f, 450.0f), sf::Vector2f(50.0f, 0.0f),
            sf::Vector2f(0.0f, 0.0f)};
    collisionPartners.push_back(new Polygon(0.0, borderVertices5));
    std::vector<sf::Vector2f> borderVertices6 = {sf::Vector2f(0.0f, 450.0f), sf::Vector2f(50.0f, 450.0f), sf::Vector2f(50.0f, 0.0f),
            sf::Vector2f(0.0f, 0.0f)};
    collisionPartners.push_back(new Polygon(0.0, borderVertices6));

    collisionPartners[0]->setPosition(200.0f, 160.0f);
    collisionPartners[0]->setVelocity(sf::Vector2f(0.0f, 0.0f));
    collisionPartners[0]->setAngularVelocity(0.0f);
    collisionPartners[0]->setRotation(0.0f);
    collisionPartners[1]->setPosition(300.0f, 200.0f);
    collisionPartners[1]->setVelocity(sf::Vector2f(0.0f, 0.0f));
    collisionPartners[1]->setAngularVelocity(0.0f);

    collisionPartners[2]->setPosition(256.0f, 5.0f);
    collisionPartners[3]->setPosition(256.0f, 507.0f);
    collisionPartners[4]->setPosition(5.0f, 256.0f);
    collisionPartners[5]->setPosition(507.0f, 256.0f);

	Simulation & simRef = Simulation::getInstance();
    simRef.initBodies(collisionPartners);
    simRef.initWindow();
	simRef.run();
	
}