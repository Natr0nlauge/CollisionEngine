#include <SFML/Graphics.hpp>
#include <iostream>

#include "Simulation.hpp"

// TODO: fix all the warnings

// TODO: implement proper tests

// TODO: use doxygen

// TODO: Replace simulation border with a seperate class

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main() {
    // TODO why is the indent weird?
    // Example program
    std::vector<RigidBody *> collisionPartners;
    std::vector<BoundaryElement *> boundaryElements;
    Simulation & simRef = Simulation::getInstance();
    // std::vector<sf::Vector2f> exampleVertices = { sf::Vector2f(25.0f, -50.0f),
    // sf::Vector2f(-25.0f, -50.0f),sf::Vector2f(-50.0f, 0.0f),
    // sf::Vector2f(-50.0f, 25.0f), sf::Vector2f(25.0f, 25.0f)     };
    std::vector<sf::Vector2f> exampleVertices = {sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -50.0f), sf::Vector2f(-25.0f, 25.0f),
            sf::Vector2f(-20.0f, 30.0f), sf::Vector2f(25.0f, 25.0f)};
    // collisionPartners.push_back(new Polygon(0.1, exampleVertices));
    Polygon * playerPtr = new Polygon(0.1, exampleVertices);
    simRef.addPlayer(playerPtr);
    // std::vector<sf::Vector2f> exampleVertices2 = { sf::Vector2f(25.0f, -25.0f),
    //// sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-50.0f, 0.0f),
    //// sf::Vector2f(-75.0f, 225.0f), sf::Vector2f(25.0f, 25.0f)     };
    std::vector<sf::Vector2f> exampleVertices2 = {sf::Vector2f(100.0f, -100.0f), sf::Vector2f(-100.0f, -100.0f),
            sf::Vector2f(-100.0f, 100.0f), sf::Vector2f(100.0f, 100.0f)};
    /* std::vector<sf::Vector2f> exampleVertices3 = {
        {21.6506271f, -12.5000153f},      {12.4999876f, -21.6506424f},
        {-1.16228075e-05f, -25.0000000f}, {-12.5000076f, -21.6506310f},
        {-21.650636f, -12.4999952f},      {-25.0000000f, 3.77489505e-06f},
        {-21.6506348f, 12.5000109f},      {-12.500019f, 21.6506348f},
        {1.0927847e-06f, 25.0000000f},    {12.499990f, 21.6506367f},
        {21.6506348f, 12.5000000f},       {25.0f, 0.0f}};*/
    // collisionPartners.push_back(new Polygon(0.1, exampleVertices2));

    // Test bodies
    collisionPartners.push_back(new Polygon(0.1, exampleVertices));
    collisionPartners[0]->setPosition({100.0f, 400.0f});
    collisionPartners[0]->setVelocity(sf::Vector2f(0, -50));
    collisionPartners[0]->setAngularVelocity(-50);

    collisionPartners.push_back(new Circle(0.1));
    collisionPartners[1]->setPosition({100.0f, 300.0f});

    playerPtr->setRotation(180.0f);
    playerPtr->setPosition(340.0f, 200.0f);
    playerPtr->setVelocity(sf::Vector2f(0, 0.0f));
    // collisionPartners[0]->setVelocity(sf::Vector2f(0.0f, -40.0f));
    playerPtr->setAngularVelocity(0);

    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[0]->setPosition(256, 0);
    boundaryElements[0]->setRotation(180);
    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[1]->setPosition(256, 512);
    boundaryElements[1]->setRotation(0);
    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[2]->setPosition(0, 256);
    boundaryElements[2]->setRotation(90);
    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[3]->setPosition(512, 256);
    boundaryElements[3]->setRotation(270);

    // borders
    // int n = collisionPartners.size();
    /*collisionPartners[n - 4]->setPosition(256.0f, 5.0f);
    collisionPartners[n - 3]->setPosition(256.0f, 507.0f);
    collisionPartners[n - 2]->setPosition(5.0f, 256.0f);
    collisionPartners[n - 1]->setPosition(507.0f, 256.0f);*/

    simRef.initWindow();
    simRef.initBodies(collisionPartners);
    simRef.initBoundaries(boundaryElements);
    simRef.run();

    return EXIT_SUCCESS;
}