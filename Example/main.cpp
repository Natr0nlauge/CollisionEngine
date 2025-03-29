#include "Simulation.hpp"


//*** Example program ***
//
//Controls: WASD for accelerating the body; LMB for teleporting the body somewhere
//
//
int main() {
    
    std::vector<RigidBody *> collisionPartners; // Vector to hold bodies to simulate
    
    Simulation & simRef = Simulation::getInstance(); // Get a Simulation instance

    // Vertices defining the geometry of example bodies
    std::vector<sf::Vector2f> examplePentagon = {sf::Vector2f(25.0f, -40.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-25.0f, 25.0f),
            sf::Vector2f(25.0f, 40.0f), sf::Vector2f(50.0f, 0.0f)};
    std::vector<sf::Vector2f> exampleTriangleVertices = {sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-25.0f, 25.0f)};

    // Add a body to be controller by the user
    Circle * circlePtr = new Circle(0.1f);
    PlayerController * playerPtr = new PlayerController(circlePtr);
    playerPtr->getPlayerBody()->setPosition(256.0f, 256.0f);


    // Initialize example bodies
    Circle * testCircle = new Circle();
    collisionPartners.push_back(testCircle);
    testCircle->setPosition({156.0f, 156.0f});
    Polygon * testSquare = new Polygon(); // Default Polygon is a square
    collisionPartners.push_back(testSquare);
    testSquare->setPosition({356.0f, 156.0f});
    testSquare->setVelocity(sf::Vector2f(0, 0));
    testSquare->setRotation(90.0f);
    Polygon * testPentagon = new Polygon(0.05f, examplePentagon);
    collisionPartners.push_back(testPentagon);
    testPentagon->setPosition({356.0f, 356.0f});
    testPentagon->setVelocity(sf::Vector2f(0, 0));
    testPentagon->setRotation(90.0f);
    Polygon * testTriangle = new Polygon(0.2f, exampleTriangleVertices);
    collisionPartners.push_back(testTriangle);
    testTriangle->setPosition({156.0f, 356.0f});
    testTriangle->setVelocity(sf::Vector2f(0, 0));
    testTriangle->setRotation(90.0f);


    // Initialize boundary elements (to be positioned at the edges of the example Simulation window)
    std::vector<BoundaryElement *> boundaryElements;
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

    // Add all the bodies to the simulation
    simRef.addPlayer(playerPtr);
    for (RigidBody * body : collisionPartners) {
        simRef.addCollisionPartner(body);
    }
    for (BoundaryElement * bElem : boundaryElements) {
        simRef.addBoundaryElement(bElem);
    }
    // Turn off contact geometry indicators
    simRef.m_showCollisionMarkers = false;

    // Optional: Change framerate and window dimensions here.
    simRef.initWindow(); 
    // Run simulation
    simRef.run();

    return EXIT_SUCCESS;
}

