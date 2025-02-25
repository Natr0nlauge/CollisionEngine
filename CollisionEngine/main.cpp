#include <SFML/Graphics.hpp>
#include <iostream>

#include "Simulation.hpp"

int main() {
    // Example program
    std::vector<RigidBody *> collisionPartners;
    std::vector<BoundaryElement *> boundaryElements;
    Simulation & simRef = Simulation::getInstance();
    std::vector<sf::Vector2f> exampleVertices = {sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -50.0f), sf::Vector2f(-25.0f, 25.0f),
            sf::Vector2f(-20.0f, 30.0f), sf::Vector2f(25.0f, 25.0f)};
    Polygon * polygonPtr = new Polygon(0.1f /*, exampleVertices */);
    // Circle * circlePtr = new Circle(0.0f);
    PlayerController * playerPtr = new PlayerController(polygonPtr);
    simRef.addPlayer(playerPtr);

    std::vector<sf::Vector2f> exampleVertices2 = {sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-25.0f, 25.0f),
            sf::Vector2f(25.0f, 25.0f)};

    Circle * testCircle = new Circle(0.1);
    collisionPartners.push_back(testCircle);
    testCircle->setPosition({100.0f, 300.0f});

    // Test bodies
    Polygon * testPolygon = new Polygon(0.1, exampleVertices2);
    collisionPartners.push_back(testPolygon);
    testPolygon->setPosition({100.0f, 400.0f});
    testPolygon->setVelocity(sf::Vector2f(0, 0));
    testPolygon->setRotation(90.0f);
    //testPolygon->setAngularVelocity(-50);

    Polygon * testPolygon2 = new Polygon(0.1);
    collisionPartners.push_back(testPolygon2);
    testPolygon2->setPosition({300.0f, 400.0f});
    testPolygon2->setVelocity(sf::Vector2f(0, 0));
    testPolygon2->setRotation(90.0f);
    //testPolygon2->setAngularVelocity(-50);

    playerPtr->getPlayerBody()->setRotation(180.0f);
    playerPtr->getPlayerBody()->setPosition(25.0f, 200.0f);
    //playerPtr->getPlayerBody()->setVelocity(sf::Vector2f(0, -10.0f));
    playerPtr->getPlayerBody()->setAngularVelocity(0);

    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[0]->setPosition(256, 0);
    boundaryElements[0]->setRotation(0);
    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[1]->setPosition(256, 512);
    boundaryElements[1]->setRotation(180);
    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[2]->setPosition(0, 256);
    boundaryElements[2]->setRotation(270);
    boundaryElements.push_back(new BoundaryElement(510));
    boundaryElements[3]->setPosition(512, 256);
    boundaryElements[3]->setRotation(90);

    for (RigidBody * body : collisionPartners) {
        simRef.addCollisionPartner(body);
    }

    for (BoundaryElement * bElem : boundaryElements) {
        simRef.addBoundaryElement(bElem);
    }

    simRef.initWindow();
    simRef.run();

    return EXIT_SUCCESS;
}