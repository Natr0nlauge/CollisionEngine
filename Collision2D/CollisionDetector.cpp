#include "CollisionDetector.hpp"
#include <iostream>
#include <array>
#include <algorithm>
#include <vector>
#include "sfml_utility.hpp"

std::unique_ptr<CollisionDetector> CollisionDetector::s_instance = nullptr;
std::mutex CollisionDetector::mtx;

/**
 * @brief This method is called if an edge-on-edge-Collision is detected.
 * @param i_sepData1 Holds the indices of the two points making up the colliding edge of the first body.
 * @param i_sepData2 Holds the indices of the two points making up the colliding edge of the second body.
 * @param i_body1 Reference to the first body.
 * @param i_body2 Reference to the second body.
 * @return The center point of the colliding segment given in global coordinates.
 */
sf::Vector2f CollisionDetector::findCenterOfContact(VertexBasedBodySeparation & i_sepData1,
        VertexBasedBodySeparation & i_sepData2, VertexBasedBody & i_body1, VertexBasedBody & i_body2) {
    const int numberOfPoints = 4;
    // Get global coordinates of all involved points
    std::array<sf::Vector2f, numberOfPoints> vertices;
    vertices[0] = i_body2.getGlobalPoint(i_sepData2.indices[0]);
    vertices[1] = i_body2.getGlobalPoint(i_sepData2.indices[1]);
    vertices[2] = i_body1.getGlobalPoint(i_sepData1.indices[0]);
    vertices[3] = i_body1.getGlobalPoint(i_sepData1.indices[1]);

    // Get the global x and y coordinates of all involved points (it doesn't matter to which body they belong)
    std::array<float, numberOfPoints> xValues = {0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, numberOfPoints> yValues = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < numberOfPoints; i++) {
        xValues[i] = vertices[i].x;
        yValues[i] = vertices[i].y;
    }
    // The center point of the contact area is determined by using the median
    return sf::Vector2f(computeMedian(xValues), computeMedian(yValues));
}

/**
 * @brief Determines the collision location and normal vector.
 * @param firstBody One of the collision partners, order doesn't matter.
 * @param secondBody The other collision partner.
 * @return The collision geometry.
 */
collisionGeometry CollisionDetector::determineCollisionGeometry(VertexBasedBody * firstBody, VertexBasedBody * secondBody) {
    collisionGeometry collisionGeometry;
    // Calculate the separations, the relevant corner indices and the relevant normal indices
    VertexBasedBodySeparation sepData2 = calculateMinVertexBasedBodySeparation(*firstBody, *secondBody);
    VertexBasedBodySeparation sepData1 = calculateMinVertexBasedBodySeparation(*secondBody, *firstBody);
    // If one of the separation values is greater than 0, the bodies are not colliding
    if (sepData1.separation <= 0 && sepData2.separation <= 0) {
        if (sepData1.indices[1] != -1 && sepData2.indices[1] != -1) {
            // Two values in each vector indicate an edge-to-edge collision
            collisionGeometry.minSeparation = sepData2.separation;
            collisionGeometry.location = findCenterOfContact(sepData1, sepData2, *firstBody, *secondBody);
            collisionGeometry.normals[0] = sepData2.normal;
            collisionGeometry.normals[1] = sepData1.normal;
        } else if (sepData2.separation < sepData1.separation) {
            // Vertex of body 1 hits edge of body 2
            collisionGeometry.minSeparation = sepData1.separation;
            collisionGeometry.location = firstBody->getGlobalPoint(sepData1.indices[0]);
            collisionGeometry.normals[0] = sfu::scaleVector(sepData1.normal, -1.0f);
            collisionGeometry.normals[1] = sepData1.normal;
        } else if (sepData2.separation >= sepData1.separation) {
            // Vertex of body 2 hits edge of body 1
            collisionGeometry.minSeparation = sepData2.separation;
            collisionGeometry.location = secondBody->getGlobalPoint(sepData2.indices[0]);
            collisionGeometry.normals[0] = sepData2.normal;
            collisionGeometry.normals[1] = sfu::scaleVector(sepData2.normal, -1.0f);
        }
    }
    return collisionGeometry;
}

/**
 * @brief Determines the collision location and normal vector.
 * @param firstBody One of the collision partners, order doesn't matter.
 * @param secondBody The other collision partner.
 * @return The collision geometry.
 */
collisionGeometry CollisionDetector::determineCollisionGeometry(VertexBasedBody * firstBody, Circle * secondBody) {
    collisionGeometry collisionGeometry = determineVertexBodyAndCircleGeometry(firstBody, secondBody);
    return collisionGeometry;
}

/**
 * @brief Determines the collision location and normal vector.
 * @param firstBody One of the collision partners, order doesn't matter.
 * @param secondBody The other collision partner.
 * @return The collision geometry.
 */
collisionGeometry CollisionDetector::determineCollisionGeometry(Circle * firstBody, VertexBasedBody * secondBody) {
    collisionGeometry collisionGeometry = determineVertexBodyAndCircleGeometry(secondBody, firstBody);
    // Switch out normal vectors (this is necessary because of the way how determineVertexBodyAndCircleGeometry() works)
    sf::Vector2f temporaryVector = collisionGeometry.normals[0];
    collisionGeometry.normals[0] = collisionGeometry.normals[1];
    collisionGeometry.normals[1] = temporaryVector;
    return collisionGeometry;
}

/**
 * @brief Determines the collision location and normal vector.
 * @param firstBody One of the collision partners, order doesn't matter.
 * @param secondBody The other collision partner.
 * @return The collision geometry.
 */
collisionGeometry CollisionDetector::determineCollisionGeometry(Circle * firstBody, Circle * secondBody) {
    collisionGeometry collisionGeometry;
    // Determine the position of the circle center points relative to each other
    sf::Vector2f firstPosition = firstBody->getPosition();
    sf::Vector2f secondPosition = secondBody->getPosition();
    sf::Vector2f relativeBodyPosition = sfu::subtractVectors(firstBody->getPosition(), secondBody->getPosition());
    // Calculate the separation
    float offset = sfu::getVectorLength(relativeBodyPosition);
    collisionGeometry.minSeparation = offset - firstBody->getRadius() - secondBody->getRadius();
    // Calculate normals and collision location
    collisionGeometry.normals[1] = sfu::normalizeVector(relativeBodyPosition);
    collisionGeometry.normals[0] = sfu::scaleVector(collisionGeometry.normals[1], -1);
    sf::Vector2f relativeCollisionPosition = sfu::scaleVector(collisionGeometry.normals[0], firstBody->getRadius());
    collisionGeometry.location = sfu::addVectors(firstBody->getPosition(), relativeCollisionPosition);
    return collisionGeometry;
}

/**
 * @brief Determines collision geometry if one body is a VertexBasedBody and the other is a Circle.
 * @note This method is called by the appropriate overloads of determineCollisionGeometry(). Avoid using this method otherwise.
 * @param theVertexBasedBody The VertexBasedBody.
 * @param theCircle The Circle.
 * @return The Collision geometry.
 */
collisionGeometry CollisionDetector::determineVertexBodyAndCircleGeometry(VertexBasedBody * theVertexBasedBody, Circle * theCircle) {
    float separation = std::numeric_limits<float>::max();
    // Determine separation distance to circle
    circleSeparation circleSeparationData = calculateMinCircleSeparation(*theVertexBasedBody, *theCircle);
    float cornerSeparation = circleSeparationData.cornerSeparation;
    float edgeSeparation = circleSeparationData.edgeSeparation;
    int pointIndex = circleSeparationData.pointIndex;
    int normalIndex = circleSeparationData.normalIndex;
    sf::Vector2f location = sf::Vector2f();
    // Initialize variables
    sf::Vector2f normal = sf::Vector2f();
    float assumedCollisionSeparation = std::numeric_limits<float>::max();
    sf::Vector2f assumedCollisionNormal = sf::Vector2f();
    sf::Vector2f assumedCollisionLocation = sf::Vector2f();
    // The SAT algorithm thinks the Circle is colliding with an edge rather than a corner
    if (edgeSeparation < cornerSeparation) {
        // The SAT algorithm thinks the Circle is colliding with an edge rather than with a corner.
        // Check if the assumed collision location is inside of the VertexBasedBody by calculating the separation seperately.
        assumedCollisionNormal = theVertexBasedBody->getGlobalNormal(normalIndex);
        assumedCollisionLocation =
                sfu::addVectors(theCircle->getPosition(), sfu::scaleVector(assumedCollisionNormal, -theCircle->getRadius()));
        assumedCollisionSeparation = theVertexBasedBody->calculateMinPointSeparation(assumedCollisionLocation).separation;
    }
    if (edgeSeparation < cornerSeparation && assumedCollisionSeparation < 0) {
        // The assumed collision location is actually inside the body and the collision detection was correct.
        normal = assumedCollisionNormal;
        location = assumedCollisionLocation;
        separation = edgeSeparation;
    } else /* if (edgeSeparation >= cornerSeparation) */ {
        // Either is the assumed collision location not actually inside the body, or it was detected as a corner collision in the first
        // place. A corner collision does not need an additional check, as the corner vertex is obviously a part of the VertexBasedBody.
        separation = cornerSeparation;
        location = theVertexBasedBody->getGlobalPoint(pointIndex);
        normal = sfu::subtractVectors(theCircle->getPosition(), location);
    }

    // Assign the values to the returned struct
    collisionGeometry collisionGeometry;
    collisionGeometry.minSeparation = separation;
    collisionGeometry.location = location;
    normal = sfu::normalizeVector(normal);
    collisionGeometry.normals[0] = normal;
    collisionGeometry.normals[1] = sfu::scaleVector(normal, -1.0f);

    return collisionGeometry;
}

/**
 * @brief Retrieves the singleton instance of the class.
 *
 * @return A reference to the CollisionDetector.
 */
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
    collisionGeometry collisionGeometry;

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

VertexBasedBodySeparation CollisionDetector::evaluateEdge(VertexBasedBody & i_body1, VertexBasedBody & i_body2, int i_index) const {
    VertexBasedBodySeparation sepDataForEdge;
    int j = 0;
    size_t body2PointCount = i_body2.getPointCount();
    // Holds the point indices of the point(s) with the smallest separation to the currently observed edge
    std::array<int, 2> tempIndexArray{-1, -1};
    sf::Vector2f normalVector = i_body1.getGlobalNormal(i_index);
    float minSep = std::numeric_limits<float>::max();
    while (j < body2PointCount) {
        // Calculate dot product for each normal and for each connecting line between vertices
        sf::Vector2f pointConnector = sf::Vector2f(i_body2.getGlobalPoint(j).x - i_body1.getGlobalPoint(i_index).x,
                i_body2.getGlobalPoint(j).y - i_body1.getGlobalPoint(i_index).y);
        float dotProd = normalVector.x * pointConnector.x + normalVector.y * pointConnector.y;

        // Find minimum value of all possible dot products (for each normal vector)
        if (dotProd <= minSep + SEPARATION_TOLERANCE) {
            minSep = dotProd;
            tempIndexArray[0] = j;  // save index
            tempIndexArray[1] = -1; // previous index is irrelevant

            // Check difference in normal angles
            float normalAngle = sfu::getVectorDirection(normalVector);
            float theOtherNormalAngle = sfu::getVectorDirection(i_body2.getGlobalNormal(j));
            float absAngleDiff = fabs(fabs(normalAngle - theOtherNormalAngle) - 180.0f);

            if (absAngleDiff < MAX_ANGLE_FOR_EDGE_TO_EDGE) {
                tempIndexArray[1] = (j + 1) % (body2PointCount); // Save index of the next vertex
                j++;                                             // Skip the next iteration manually
            }
        }
        j++; // standard loop increment
    }
    sepDataForEdge.normal = normalVector;
    sepDataForEdge.separation = minSep;
    sepDataForEdge.indices = tempIndexArray;
    return sepDataForEdge;
}

/**
 * @brief Runs the SAT algorithm to determine the separation of two VertexBasedBodies.
 * @param i_body1 One of the bodies, order doesn't matter.
 * @param i_body2 The other body.
 * @return A struct holding the separation and some associated data.
 */
VertexBasedBodySeparation CollisionDetector::calculateMinVertexBasedBodySeparation(VertexBasedBody & i_body1,
        VertexBasedBody & i_body2) const {

    VertexBasedBodySeparation separationData;

    std::array<int, 2> indexArray2{-1, -1}; // Holds the point indices for the edge corresponding to the largest separation
    // Loop through all vertices for Body1 and get normal vector
    for (int i = 0; i < i_body1.getNormalCount(); i++) {
        VertexBasedBodySeparation tempSeparationData = evaluateEdge(i_body1, i_body2, i);

        
        // The maximum minSep value for all normal vectors (for all i values) is the minimal seperation
        if (tempSeparationData.separation >= separationData.separation + SEPARATION_TOLERANCE) {
            separationData = tempSeparationData;
        }
    }
    return separationData;
}



/**
 * @brief Run a modified SAT algorithm to determine the separation of a VertexBasedBody and a Circle.
 * @param i_VertexBasedBody The VertexBasedBody.
 * @param i_circle The Circle.
 * @return A struct holding the separation and some associated data.
 */
circleSeparation CollisionDetector::calculateMinCircleSeparation(VertexBasedBody & i_VertexBasedBody, Circle & i_circle) const {
    float radius = i_circle.getRadius();
    circleSeparation separationData;

    for (int i = 0; i < i_VertexBasedBody.getNormalCount(); i++) {
        // Iterate through all normals and determine the distance to the center point of the Circle
        sf::Vector2f testPoint = i_VertexBasedBody.getGlobalPoint(i);
        sf::Vector2f relativeCirclePosition = sfu::subtractVectors(i_circle.getPosition(), testPoint);

        // Always substract the radius from the determined separation
        float newCornerSeparation = sfu::getVectorLength(relativeCirclePosition) - radius;
        // Corner separation is the separation to the closest corner, or the minimum value
        if (newCornerSeparation < separationData.cornerSeparation) {
            // Save new separation and index
            separationData.cornerSeparation = newCornerSeparation;
            separationData.pointIndex = i;
        }

        sf::Vector2f testNormal = i_VertexBasedBody.getGlobalNormal(i);
        sf::Vector2f pointConnector = sfu::subtractVectors(i_circle.getPosition(), testPoint);
        float newEdgeSeparation = sfu::scalarProduct(testNormal, pointConnector) - radius;
        // Edge separation is the maximum value of all possible edge separations
        if (newEdgeSeparation > separationData.edgeSeparation) {
            // Save new separation and index
            separationData.edgeSeparation = newEdgeSeparation;
            separationData.normalIndex = i;
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
    // Make a copy of the array because it has to be sorted
    std::array<float, 4> sortedArr = i_arr;
    std::sort(sortedArr.begin(), sortedArr.end());

    // Compute and return the median (average of the two middle elements)
    return (sortedArr[1] + sortedArr[2]) / 2.0f;
}
