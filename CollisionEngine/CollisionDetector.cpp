#include "CollisionDetector.hpp"
#include <iostream>
#include <array>
#include <algorithm>
#include <vector>
#include "sfml_utility.hpp"

std::unique_ptr<CollisionDetector> CollisionDetector::s_instance = nullptr;
std::mutex CollisionDetector::mtx;

/**
 * @brief This is called if an edge-on-edge-Collision is detected.
 * @param i_sepData1 Holds the indices of the two points making up the colliding edge of the first body.
 * @param i_sepData2 Holds the indices of the two points making up the colliding edge of the second body.
 * @param i_body1 Reference to the first body.
 * @param i_body2 Reference to the second body.
 * @return The center point of the colliding segment given in global coordinates.
 */
sf::Vector2f CollisionDetector::findCenterOfContact(VertexBasedBodySeparation_type & i_sepData1,
        VertexBasedBodySeparation_type & i_sepData2, VertexBasedBody & i_body1, VertexBasedBody & i_body2) {
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

    return sf::Vector2f(computeMedian(xValues), computeMedian(yValues));
}

/**
 * @brief Determines the collision location and normal vector.
 * @param firstBody One of the collision partners, order doesn't matter.
 * @param secondBody The other collision partner.
 * @return The collision geometry.
 */
collisionGeometry_type CollisionDetector::determineCollisionGeometry(VertexBasedBody * firstBody, VertexBasedBody * secondBody) {
    collisionGeometry_type collisionGeometry;

    // Do the actual calculations
    VertexBasedBodySeparation_type collData2 = calculateMinVertexBasedBodySeparation(*firstBody, *secondBody);
    VertexBasedBodySeparation_type collData1 = calculateMinVertexBasedBodySeparation(*secondBody, *firstBody);
    //  If one of the separation values is greater than 0, the bodies are not colliding
    // if (collData1.separation <= 0 && collData2.separation <= 0) /*(collIndexVec2.size()>0 && collIndexVec1.size()>0)*/ {
    if (collData1.indexVec.size() > 1 && collData2.indexVec.size() > 1) {
        // Two values in each vector indicate an edge-to-edge collision
        collisionGeometry.minSeparation = collData2.separation;
        collisionGeometry.location = findCenterOfContact(collData1, collData2, *firstBody, *secondBody);
        collisionGeometry.normals[0] = collData2.normal;
        collisionGeometry.normals[1] = collData1.normal;
    } else if (collData2.separation < collData1.separation) {
        // Vertex of body 1 hits edge of body 2
        collisionGeometry.minSeparation = collData1.separation;
        collisionGeometry.location = firstBody->getGlobalPoint(collData1.indexVec[0]);
        collisionGeometry.normals[0] = sfu::scaleVector(collData1.normal, -1.0f);
        collisionGeometry.normals[1] = sfu::scaleVector(collData1.normal, 1.0f);
        // assignNormal(c_collisionEvent, collData1);
    } else /*if (collData2.separation > collData1.separation)*/ {
        // Vertex of body 2 hits edge of body 1
        collisionGeometry.minSeparation = collData2.separation;
        collisionGeometry.location = secondBody->getGlobalPoint(collData2.indexVec[0]);
        collisionGeometry.normals[0] = sfu::scaleVector(collData2.normal, 1.0f);
        collisionGeometry.normals[1] = sfu::scaleVector(collData2.normal, -1.0f);
    }

    return collisionGeometry;
}

collisionGeometry_type CollisionDetector::determineCollisionGeometry(VertexBasedBody * firstBody, Circle * secondBody) {
    collisionGeometry_type collisionGeometry = determineEdgeAndCircleGeometry(firstBody, secondBody);

    return collisionGeometry;
}

collisionGeometry_type CollisionDetector::determineCollisionGeometry(Circle * firstBody, VertexBasedBody * secondBody) {
    collisionGeometry_type collisionGeometry = determineEdgeAndCircleGeometry(secondBody, firstBody);
    // Switch normals
    sf::Vector2f normal = collisionGeometry.normals[0];
    collisionGeometry.normals[0] = collisionGeometry.normals[1];
    collisionGeometry.normals[1] = normal;
    return collisionGeometry;
}

/**
 * @brief Determines collision geometry if one Body is a VertexBasedBody and the other is a Circle.
 * @attention Avoid using this directly. Use the appropriate overload of determineCollisionGeometry() instead, which will call this method.
 * @param theVertexBasedBody The VertexBasedBody.
 * @param theCircle The Circle.
 * @return The Collision geometry.
 */
collisionGeometry_type CollisionDetector::determineEdgeAndCircleGeometry(VertexBasedBody * theVertexBasedBody, Circle * theCircle) {
    collisionGeometry_type collisionGeometry;
    float separation = std::numeric_limits<float>::max();
    // Modified SAT algorithm
    circleSeparation_type circleSeparationData = calculateMinCircleSeparation(*theVertexBasedBody, *theCircle);
    float cornerSeparation = circleSeparationData.cornerSeparation;
    float edgeSeparation = circleSeparationData.edgeSeparation;
    int pointIndex = circleSeparationData.pointIndex;
    int normalIndex = circleSeparationData.normalIndex;
    sf::Vector2f normal = sf::Vector2f();

    // std::cout << cornerSeparation << ", " << edgeSeparation << "\n";
    if (circleSeparationData.edgeSeparation < cornerSeparation) {
        sf::Vector2f assumedCollisionNormal = theVertexBasedBody->getGlobalNormal(normalIndex);
        sf::Vector2f assumedCollisionLocation =
                sfu::addVectors(theCircle->getPosition(), sfu::scaleVector(assumedCollisionNormal, -theCircle->getRadius()));

        // Check if the Collision Location is actually inside the VertexBasedBody
        float collisionSeparation = theVertexBasedBody->calculateMinPointSeparation(assumedCollisionLocation).separation;

        if (collisionSeparation < 0) {
            normal = assumedCollisionNormal;
            collisionGeometry.location = assumedCollisionLocation;
            separation = edgeSeparation;
        } else {
            edgeSeparation = std::numeric_limits<float>::max();
        }
    }

    if (edgeSeparation >= cornerSeparation) {
        separation = cornerSeparation;
        collisionGeometry.location = theVertexBasedBody->getGlobalPoint(pointIndex);
        normal = sfu::subtractVectors(theCircle->getPosition(), collisionGeometry.location);
    }

    collisionGeometry.minSeparation = separation;
    normal = sfu::normalizeVector(normal);
    collisionGeometry.normals[0] = normal;
    collisionGeometry.normals[1] = sfu::scaleVector(normal, -1.0f);

    return collisionGeometry;
}

collisionGeometry_type CollisionDetector::determineCollisionGeometry(Circle * firstBody, Circle * secondBody) {
    collisionGeometry_type collisionGeometry;
    sf::Vector2f firstPosition = firstBody->getPosition();
    sf::Vector2f secondPosition = secondBody->getPosition();
    sf::Vector2f relativePosition = sfu::subtractVectors(firstBody->getPosition(), secondBody->getPosition());

    float offset = sfu::getVectorLength(relativePosition);
    collisionGeometry.minSeparation = offset - firstBody->getRadius() - secondBody->getRadius();

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

// Constructor.
CollisionDetector::CollisionDetector() {}

// Destructor.
CollisionDetector::~CollisionDetector() {}

//  Detects a collision between two VertexBasedBodys and writes results to a collisionEvent
CollisionEvent CollisionDetector::generateCollisionEvent(RigidBody * i_firstBody, RigidBody * i_secondBody) {

    // Find out what the first body is

    collisionGeometry_type collisionGeometry;

    if (VertexBasedBody * firstBodyAsVertexBasedBody = dynamic_cast<VertexBasedBody *>(i_firstBody)) {
        if (VertexBasedBody * secondBodyAsVertexBasedBody = dynamic_cast<VertexBasedBody *>(i_secondBody)) {
            // Both bodies are VertexBasedBodys
            collisionGeometry = determineCollisionGeometry(firstBodyAsVertexBasedBody, secondBodyAsVertexBasedBody);
        } else if (Circle * secondBodyAsCircle = dynamic_cast<Circle *>(i_secondBody)) {
            // Body one is VertexBasedBody, body two is Circle
            collisionGeometry = determineCollisionGeometry(firstBodyAsVertexBasedBody, secondBodyAsCircle);
        }
    } else if (Circle * firstBodyAsCircle = dynamic_cast<Circle *>(i_firstBody)) {
        if (VertexBasedBody * secondBodyAsVertexBasedBody = dynamic_cast<VertexBasedBody *>(i_secondBody)) {
            // Body one is Circle, body two is VertexBasedBody
            collisionGeometry = determineCollisionGeometry(firstBodyAsCircle, secondBodyAsVertexBasedBody);
        } else if (Circle * secondBodyAsCircle = dynamic_cast<Circle *>(i_secondBody)) {
            // Both bodies are Circles
            collisionGeometry = determineCollisionGeometry(firstBodyAsCircle, secondBodyAsCircle);
        }
    }

    return CollisionEvent(i_firstBody, i_secondBody, collisionGeometry);
}

/**
 * @brief Runs the SAT algorithm to determine the separation of two VertexBasedBodies.
 * @param i_body1 One of the bodies, order doesn't matter.
 * @param i_body2 The other body.
 * @return A struct holding the separation and some associated data.
 */
VertexBasedBodySeparation_type CollisionDetector::calculateMinVertexBasedBodySeparation(VertexBasedBody & i_body1,
        VertexBasedBody & i_body2) const {

    VertexBasedBodySeparation_type sepData;
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

/**
 * @brief Run a modified SAT algorithm to determine the separation of a VertexBasedBody and a Circle.
 * @param i_VertexBasedBody The VertexBasedBody.
 * @param i_circle The Circle.
 * @return A struct holding the separation and some associated data.
 */
circleSeparation_type CollisionDetector::calculateMinCircleSeparation(VertexBasedBody & i_VertexBasedBody, Circle & i_circle) const {
    float radius = i_circle.getRadius();
    circleSeparation_type separationData;

    // Modified SAT algorithm
    for (int i = 0; i < i_VertexBasedBody.getNormalCount(); i++) {
        sf::Vector2f testPoint = i_VertexBasedBody.getGlobalPoint(i);
        sf::Vector2f relativeCirclePosition = sfu::subtractVectors(i_circle.getPosition(), testPoint);

        float newCornerSeparation = sfu::getVectorLength(relativeCirclePosition) - radius;
        if (newCornerSeparation < separationData.cornerSeparation) {
            separationData.cornerSeparation = newCornerSeparation;
            separationData.pointIndex = i;
        }

        sf::Vector2f testNormal = i_VertexBasedBody.getGlobalNormal(i);
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

/**
 * @brief Compute the median of an array of 4 floats.
 * @param i_arr The array of floats.
 * @return The median.
 */
float CollisionDetector::computeMedian(const std::array<float, 4> & i_arr) {
    // Make a copy of the array because we need to sort it
    std::array<float, 4> sortedArr = i_arr;
    std::sort(sortedArr.begin(), sortedArr.end());

    // Compute and return the median (average of the two middle elements)
    return (sortedArr[1] + sortedArr[2]) / 2.0f;
}
