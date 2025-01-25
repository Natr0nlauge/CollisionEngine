#include "CollisionDetector.hpp"
#include <iostream>
#include <array>
#include <algorithm>
#include <vector>
#include "sfml_utility.hpp"

std::unique_ptr<CollisionDetector> CollisionDetector::s_instance = nullptr;
std::mutex CollisionDetector::mtx;

// In case of edge-edge-collision: Find the center of the collision area
sf::Vector2f CollisionDetector::findCenterOfContact(edgeStructureSeparationData_type & i_sepData1,
        edgeStructureSeparationData_type & i_sepData2, EdgeStructure & i_body1, EdgeStructure & i_body2) {
    const int numberOfPoints = 4;
    sf::Vector2f vertices[numberOfPoints];
    std::array<float, numberOfPoints> xValues = {0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, numberOfPoints> yValues = {0.0f, 0.0f, 0.0f, 0.0f};

    vertices[0] = i_body2.getGlobalPoint(i_sepData2.indexVec[0]);
    vertices[1] = i_body2.getGlobalPoint(i_sepData2.indexVec[1]);
    vertices[2] = i_body1.getGlobalPoint(i_sepData1.indexVec[0]);
    vertices[3] = i_body1.getGlobalPoint(i_sepData1.indexVec[1]);

    for (int i = 0; i < numberOfPoints; i++) {
        xValues[i] = vertices[i].x;
        yValues[i] = vertices[i].y;
    }

    return sf::Vector2f(sfu::computeMedian(xValues), sfu::computeMedian(yValues));
}

// Detect if two EdgeStructures are colliding and write collision data to a c_collisionEvent
// TODO: USE OVERLOADS FOR THESE, give only 2 body pointers as arguments, return collisionGeometry_type
collisionGeometry_type CollisionDetector::determineCollisionGeometry(EdgeStructure * firstBody, EdgeStructure * secondBody) {
    collisionGeometry_type collisionGeometry;

    // Do the actual calculations
    edgeStructureSeparationData_type collData2 = calculateMinEdgeStructureSeparation(*firstBody, *secondBody);
    edgeStructureSeparationData_type collData1 = calculateMinEdgeStructureSeparation(*secondBody, *firstBody);

    //  If one of the separation values is greater than 0, the bodies are not colliding
    // if (collData1.separation <= 0 && collData2.separation <= 0) /*(collIndexVec2.size()>0 && collIndexVec1.size()>0)*/ {
    if (collData1.indexVec.size() > 1 && collData2.indexVec.size() > 1) {
        // Two values in each vector indicate an edge-to-edge collision
        collisionGeometry.separation = collData2.separation;
        collisionGeometry.location = findCenterOfContact(collData1, collData2, *firstBody, *secondBody);
        collisionGeometry.normals[0] = collData2.normal;
        collisionGeometry.normals[1] = collData1.normal;
    } else if (collData2.separation < collData1.separation) {
        // Vertex of body 1 hits edge of body 2
        collisionGeometry.separation = collData1.separation;
        collisionGeometry.location = firstBody->getGlobalPoint(collData1.indexVec[0]);
        collisionGeometry.normals[0] = sfu::scaleVector(collData1.normal, -1.0f);
        collisionGeometry.normals[1] = sfu::scaleVector(collData1.normal, 1.0f);
        // assignNormal(c_collisionEvent, collData1);
    } else /*if (collData2.separation > collData1.separation)*/ {
        // Vertex of body 2 hits edge of body 1
        collisionGeometry.separation = collData2.separation;
        collisionGeometry.location = secondBody->getGlobalPoint(collData2.indexVec[0]);
        collisionGeometry.normals[0] = sfu::scaleVector(collData2.normal, 1.0f);
        collisionGeometry.normals[1] = sfu::scaleVector(collData2.normal, -1.0f);
    }
    //}

    return collisionGeometry;
}

// Detect if a Circle and an EdgeStructure are colliding and write collision data to a c_collisionEvent
collisionGeometry_type CollisionDetector::determineCollisionGeometry(EdgeStructure * firstBody, Circle * secondBody) {
    collisionGeometry_type collisionGeometry = determineEdgeAndCircleGeometry(firstBody, secondBody);

    return collisionGeometry;
}

collisionGeometry_type CollisionDetector::determineCollisionGeometry(Circle * firstBody, EdgeStructure * secondBody) {
    collisionGeometry_type collisionGeometry = determineEdgeAndCircleGeometry(secondBody, firstBody);
    // Switch normals
    sf::Vector2f normal = collisionGeometry.normals[0];
    collisionGeometry.normals[0] = collisionGeometry.normals[1];
    collisionGeometry.normals[1] = normal;
    return collisionGeometry;
}

collisionGeometry_type CollisionDetector::determineEdgeAndCircleGeometry(EdgeStructure * theEdgeStructure, Circle * theCircle) {
    collisionGeometry_type collisionGeometry;
    float separation = std::numeric_limits<float>::max();
    // Modified SAT algorithm
    circleSeparationData_type circleSeparationData = calculateMinCircleSeparation(*theEdgeStructure, *theCircle);
    float cornerSeparation = circleSeparationData.cornerSeparation;
    float edgeSeparation = circleSeparationData.edgeSeparation;
    int pointIndex = circleSeparationData.pointIndex;
    int normalIndex = circleSeparationData.normalIndex;
    // TODO change this, nobody understands this
    sf::Vector2f * firstNormalPointer = &(collisionGeometry.normals[0]);
    sf::Vector2f * secondNormalPointer = &(collisionGeometry.normals[1]);

    // std::cout << cornerSeparation << ", " << edgeSeparation << "\n";
    if (circleSeparationData.edgeSeparation < cornerSeparation) {
        sf::Vector2f assumedCollisionNormal = theEdgeStructure->getGlobalNormal(normalIndex);
        sf::Vector2f assumedCollisionLocation =
                sfu::addVectors(theCircle->getPosition(), sfu::scaleVector(assumedCollisionNormal, -theCircle->getRadius()));

        // Check if the Collision Location is actually inside the EdgeStructure
        float collisionSeparation = theEdgeStructure->calculateMinPointSeparation(assumedCollisionLocation).separation;

        if (collisionSeparation < 0) {
            *firstNormalPointer = assumedCollisionNormal;
            collisionGeometry.location = assumedCollisionLocation;
            separation = edgeSeparation;
        } else {
            edgeSeparation = std::numeric_limits<float>::max();
        }
    }

    if (edgeSeparation >= cornerSeparation) {
        separation = cornerSeparation;
        collisionGeometry.location = theEdgeStructure->getGlobalPoint(pointIndex);
        sf::Vector2f normal = sfu::subtractVectors(theCircle->getPosition(), collisionGeometry.location);
        *firstNormalPointer = sfu::normalizeVector(normal);
    }
    //std::cout << separation << "\n";
    collisionGeometry.separation = separation;

    *secondNormalPointer = sfu::scaleVector(*firstNormalPointer, -1.0f);

    return collisionGeometry;
} // end of if (theEdgeStructure != NULL && theCircle != NULL)

// Detect if two Circles are colliding and write collision data to a c_collisionEvent
collisionGeometry_type CollisionDetector::determineCollisionGeometry(Circle * firstBody, Circle * secondBody) {
    collisionGeometry_type collisionGeometry;
    sf::Vector2f firstPosition = firstBody->getPosition();
    sf::Vector2f secondPosition = secondBody->getPosition();
    sf::Vector2f relativePosition = sfu::subtractVectors(firstBody->getPosition(), secondBody->getPosition());

    float offset = sfu::getVectorLength(relativePosition);
    collisionGeometry.separation = offset - firstBody->getRadius() - secondBody->getRadius();

    collisionGeometry.normals[1] = sfu::normalizeVector(relativePosition);
    collisionGeometry.normals[0] = sfu::scaleVector(collisionGeometry.normals[1], -1);
    sf::Vector2f relativeCollisionPosition = sfu::scaleVector(collisionGeometry.normals[0], firstBody->getRadius());
    collisionGeometry.location = sfu::addVectors(firstBody->getPosition(), relativeCollisionPosition);

    return collisionGeometry;
}

CollisionDetector & CollisionDetector::getInstance() {
    if (s_instance == nullptr) {
        std::lock_guard<std::mutex> lock(mtx);
        if (!s_instance) {
            s_instance.reset(new CollisionDetector());
        }
    }
    return *s_instance;
}

// Constructor
CollisionDetector::CollisionDetector() {}

// Destructor
CollisionDetector::~CollisionDetector() {}

//  Detects a collision between two edgeStructures and writes results to a collisionEvent
bool CollisionDetector::detectCollision(CollisionEvent & c_collisionEvent) {
    RigidBody * firstBody = c_collisionEvent.getCollisionPartner(0);
    RigidBody * secondBody = c_collisionEvent.getCollisionPartner(1);

    // Find out what the first body is

    collisionGeometry_type collisionGeometry;

    if (EdgeStructure * firstBodyAsEdgeStructure = dynamic_cast<EdgeStructure *>(firstBody)) {
        if (EdgeStructure * secondBodyAsEdgeStructure = dynamic_cast<EdgeStructure *>(secondBody)) {
            // Both bodies are EdgeStructures
            collisionGeometry = determineCollisionGeometry(firstBodyAsEdgeStructure, secondBodyAsEdgeStructure);
        } else if (Circle * secondBodyAsCircle = dynamic_cast<Circle *>(secondBody)) {
            // Body one is EdgeStructure, body two is Circle
            collisionGeometry = determineCollisionGeometry(firstBodyAsEdgeStructure, secondBodyAsCircle);
        }
    } else if (Circle * firstBodyAsCircle = dynamic_cast<Circle *>(firstBody)) {
        if (EdgeStructure * secondBodyAsEdgeStructure = dynamic_cast<EdgeStructure *>(secondBody)) {
            // Body one is Circle, body two is EdgeStructure
            collisionGeometry = determineCollisionGeometry(firstBodyAsCircle, secondBodyAsEdgeStructure);
        } else if (Circle * secondBodyAsCircle = dynamic_cast<Circle *>(secondBody)) {
            // Both bodies are Circles
            collisionGeometry = determineCollisionGeometry(firstBodyAsCircle, secondBodyAsCircle);
        }
    }
    c_collisionEvent.setCollisionGeometry(collisionGeometry);

    return collisionGeometry.separation < 0;
}

edgeStructureSeparationData_type CollisionDetector::calculateMinEdgeStructureSeparation(EdgeStructure & i_body1,
        EdgeStructure & i_body2) const {

    edgeStructureSeparationData_type sepData;
    int normalIndex = 0;

    std::vector<int> preliminaryCollIndexVec;
    std::vector<int> preliminaryCollIndexVec2;

    // Loop through all vertices for Body1 and get normal vector
    for (int i = 0; i < i_body1.getNormalCount(); i++) {
        sf::Vector2f normal = i_body1.getGlobalNormal(i);
        float minSep = std::numeric_limits<float>::max();

        // Modified SAT algorithm
        for (int j = 0; j < i_body2.getPointCount(); j++) {
            // Calculate dot product for each normal and for each connecting line between vertices
            sf::Vector2f pointConnector = sf::Vector2f(i_body2.getGlobalPoint(j).x - i_body1.getGlobalPoint(i).x,
                    i_body2.getGlobalPoint(j).y - i_body1.getGlobalPoint(i).y);
            float dotProd = normal.x * pointConnector.x + normal.y * pointConnector.y;
            // Find minimum value of all possible dot products (for each normal vector)
            if (dotProd < minSep - MIN_SEP_EPSILON) {
                // minSep = std::min(minSep, dotProd);
                minSep = dotProd;
                preliminaryCollIndexVec.clear();      // previous indices are irrelevant
                preliminaryCollIndexVec.push_back(j); // save index
            }
            // If a minSep value is sufficiently close to a previous one, save another index (edge-to-edge-collision possible)
            // Tolerance needs to be larger here to correctly account for angle deviations
            else if (fabs(minSep - dotProd) <= 4 * MIN_SEP_EPSILON) {
                preliminaryCollIndexVec.push_back(j);
            }
        }

        // The maximum minSep value for all normal vectors (for all i values) is the minimal seperation
        if (minSep > sepData.separation) {

            sepData.separation = minSep;
            preliminaryCollIndexVec2 = preliminaryCollIndexVec;
            normalIndex = i;
            sepData.normal = normal;
        }
    }
    sepData.indexVec = preliminaryCollIndexVec2; // output
    // std::cout << collData.indexVec.size() << ", ";
    return sepData;
}

circleSeparationData_type CollisionDetector::calculateMinCircleSeparation(EdgeStructure & i_edgeStructure, Circle & i_circle) const {
    float radius = i_circle.getRadius();
    circleSeparationData_type separationData;

    // Modified SAT algorithm
    for (int i = 0; i < i_edgeStructure.getNormalCount(); i++) {
        sf::Vector2f testPoint = i_edgeStructure.getGlobalPoint(i);
        sf::Vector2f relativeCirclePosition = sfu::subtractVectors(i_circle.getPosition(), testPoint);

        float newCornerSeparation = sfu::getVectorLength(relativeCirclePosition) - radius;
        if (newCornerSeparation < separationData.cornerSeparation) {
            separationData.cornerSeparation = newCornerSeparation;
            separationData.pointIndex = i;
        }

        sf::Vector2f testNormal = i_edgeStructure.getGlobalNormal(i);
        sf::Vector2f pointConnector = sfu::subtractVectors(i_circle.getPosition(), testPoint);
        // std::cout << testPoint.x << ", " << testPoint.y << "\n";
        float newEdgeSeparation = sfu::scalarProduct(testNormal, pointConnector) - radius;
        // Edge separation is the maximum value of all possible edge separations
        // std::cout << newEdgeSeparation << "\n";
        if (newEdgeSeparation > separationData.edgeSeparation) {
            // minSep = std::min(minSep, dotProd);
            separationData.edgeSeparation = newEdgeSeparation; // previous indices are irrelevant
            separationData.normalIndex = i;                    // save index
        }
    }

    return separationData;
}
