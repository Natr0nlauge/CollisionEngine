#include "CollisionDetector.hpp"
#include <iostream>
#include <array>
#include <algorithm>
#include <vector>
#include "sfml_utility.hpp"

std::unique_ptr<CollisionDetector> CollisionDetector::s_instance = nullptr;
std::mutex CollisionDetector::mtx;

// Find global coordinates of the colliding edges' vertices
sf::Vector2f CollisionDetector::findCenterOfContact(polygonSeparationData_type & i_sepData1, polygonSeparationData_type & i_sepData2, Polygon & i_body1,
        Polygon & i_body2) {
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

bool CollisionDetector::detectPolygonCollision(CollisionEvent & c_collisionEvent) {
    Polygon * collPars[2] = {static_cast<Polygon *>(c_collisionEvent.m_collisionPartners[0]),
            static_cast<Polygon *>(c_collisionEvent.m_collisionPartners[1])};

    polygonSeparationData_type collData2 = calculateMinPolygonSeparation(*collPars[0], *collPars[1]);
    polygonSeparationData_type collData1 = calculateMinPolygonSeparation(*collPars[1], *collPars[0]);
    // std::cout << collData1.separation << ", " << collData2.separation << "\n";
    //  If both minimum seperations are smaller than 0, it indicates a collision
    if (collData1.separation <= 0 && collData2.separation <= 0) /*(collIndexVec2.size()>0 && collIndexVec1.size()>0)*/ {
        // Two values in each vector indicate an edge-to-edge collision
        if (collData1.indexVec.size() > 1 && collData2.indexVec.size() > 1) {
            // collData1.normal = sfu::scaleVector(collData1.normal, -1); // make sure that normal has the correct direction
            c_collisionEvent.m_collisionLocation = findCenterOfContact(collData1, collData2, *collPars[0], *collPars[1]);
            c_collisionEvent.m_contactNormals[0] = collData2.normal;
            c_collisionEvent.m_contactNormals[1] = collData1.normal;
        } else if (collData2.separation < collData1.separation) {
            // Vertex of body 1 hits edge of body 2
            // Assign location
            c_collisionEvent.m_collisionLocation = collPars[0]->getGlobalPoint(collData1.indexVec[0]);
            // collData1.normal = sfu::scaleVector(collData1.normal, -1.0f); // make sure that normal has the correct direction
            c_collisionEvent.m_contactNormals[0] = sfu::scaleVector(collData1.normal, -1.0f);
            c_collisionEvent.m_contactNormals[1] = sfu::scaleVector(collData1.normal, 1.0f);

            // assignNormal(c_collisionEvent, collData1);
        } else /*if (collData2.separation > collData1.separation)*/ {
            // Vertex of body 2 hits edge of body 1
            // Assign location
            c_collisionEvent.m_collisionLocation = collPars[1]->getGlobalPoint(collData2.indexVec[0]);
            c_collisionEvent.m_contactNormals[0] = sfu::scaleVector(collData2.normal, 1.0f);
            c_collisionEvent.m_contactNormals[1] = sfu::scaleVector(collData2.normal, -1.0f);
        }
        // std::cout << "Position in CollisionDetector: " << c_collisionEvent.collLoc1.x << ", " << c_collisionEvent.collLoc1.y << "\n";
        return true;
    } else {
        return false;
    }
}

bool CollisionDetector::detectPolygonAndCircleCollision(CollisionEvent & c_collisionEvent) {
    // Detect which body is the circle and which is the polygon
    // Take a guess
    Circle * theCircle = dynamic_cast<Circle *>(c_collisionEvent.m_collisionPartners[0]);
    Polygon * thePolygon = dynamic_cast<Polygon *>(c_collisionEvent.m_collisionPartners[1]);
    sf::Vector2f * firstNormalPointer = &c_collisionEvent.m_contactNormals[1];
    sf::Vector2f * secondNormalPointer = &c_collisionEvent.m_contactNormals[0];

    if (!theCircle) { // The guess was wrong, reassign pointers
        theCircle = dynamic_cast<Circle *>(c_collisionEvent.m_collisionPartners[1]);
        thePolygon = dynamic_cast<Polygon *>(c_collisionEvent.m_collisionPartners[0]);
        firstNormalPointer = &c_collisionEvent.m_contactNormals[0];
        secondNormalPointer = &c_collisionEvent.m_contactNormals[1];
    }

    if (!thePolygon || !theCircle) {
        return false;
    }

    float separation = std::numeric_limits<float>::max();

    // Modified SAT algorithm
    circleSeparationData_type circleSeparationData = calculateMinCircleSeparation(*thePolygon, *theCircle);
    float cornerSeparation = circleSeparationData.cornerSeparation;
    float edgeSeparation = circleSeparationData.edgeSeparation;
    int pointIndex = circleSeparationData.pointIndex;
    int normalIndex = circleSeparationData.normalIndex;

    std::cout << cornerSeparation << ", " << edgeSeparation << "\n";
    if (circleSeparationData.edgeSeparation < cornerSeparation) {
        sf::Vector2f assumedCollisionNormal = thePolygon->getGlobalNormal(normalIndex);
        sf::Vector2f assumedCollisionLocation =
                sfu::addVectors(theCircle->getPosition(), sfu::scaleVector(assumedCollisionNormal, -theCircle->getRadius()));

        // Check if the Collision Location is actually inside the Polygon
        float collisionSeparation = thePolygon->calculateMinPointSeparation(assumedCollisionLocation).separation;
        std::cout << collisionSeparation << "\n";
        if (collisionSeparation < 0) {
            *firstNormalPointer = assumedCollisionNormal;
            c_collisionEvent.m_collisionLocation = assumedCollisionLocation;
            separation = edgeSeparation;
        } else {
            edgeSeparation = std::numeric_limits<float>::max();
        }
    }

    if (edgeSeparation >= cornerSeparation) {
        separation = cornerSeparation;
        c_collisionEvent.m_collisionLocation = thePolygon->getGlobalPoint(pointIndex);
        sf::Vector2f normal = sfu::subtractVectors(theCircle->getPosition(), c_collisionEvent.m_collisionLocation);
        *firstNormalPointer = sfu::normalizeVector(normal);
    }
    // std::cout << separation << "\n";

    *secondNormalPointer = sfu::scaleVector(*firstNormalPointer, -1.0f);

    return separation < 0;
} // end of if (thePolygon != NULL && theCircle != NULL)



bool CollisionDetector::detectCircleCollision(CollisionEvent & c_collisionEvent) {

    Circle * firstBody = static_cast<Circle *>(c_collisionEvent.m_collisionPartners[0]);
    Circle * secondBody = static_cast<Circle *>(c_collisionEvent.m_collisionPartners[1]);
    if (firstBody && secondBody) {
        sf::Vector2f firstPosition = c_collisionEvent.m_collisionPartners[0]->getPosition();
        sf::Vector2f secondPosition = c_collisionEvent.m_collisionPartners[1]->getPosition();
        sf::Vector2f relativePosition = sfu::subtractVectors(firstBody->getPosition(), secondBody->getPosition());

        float offset = sfu::getVectorLength(relativePosition);
        float separation = offset - firstBody->getRadius() - secondBody->getRadius();

        if (separation < 0) {
            c_collisionEvent.m_contactNormals[1] = sfu::normalizeVector(relativePosition);
            c_collisionEvent.m_contactNormals[0] = sfu::scaleVector(c_collisionEvent.m_contactNormals[1], -1);
            sf::Vector2f relativeCollisionPosition = sfu::scaleVector(c_collisionEvent.m_contactNormals[0], firstBody->getRadius());
            c_collisionEvent.m_collisionLocation = sfu::addVectors(firstBody->getPosition(), relativeCollisionPosition);

            return true;
        }
    }
    return false;
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

CollisionDetector::CollisionDetector() {}

CollisionDetector::~CollisionDetector() {}

//  Detects a collision between two polygons and writes results to a collisionEvent
bool CollisionDetector::detectCollision(CollisionEvent & c_collisionEvent) {
    RigidBody * firstBody = c_collisionEvent.m_collisionPartners[0];
    RigidBody * secondBody = c_collisionEvent.m_collisionPartners[1];
    if (dynamic_cast<Polygon *>(firstBody) && dynamic_cast<Polygon *>(secondBody)) {
        // Both bodies are Polygons
        return detectPolygonCollision(c_collisionEvent);
    } else if ((dynamic_cast<Polygon *>(firstBody) && dynamic_cast<Circle *>(secondBody)) ||
               (dynamic_cast<Circle *>(firstBody) && dynamic_cast<Polygon *>(secondBody))) {
        // One body is a circle and one is a polygon
        return detectPolygonAndCircleCollision(c_collisionEvent);
    } else if ((dynamic_cast<Circle *>(firstBody) && dynamic_cast<Circle *>(secondBody))) {
        // Both bodies are circles
        return detectCircleCollision(c_collisionEvent);
    } else {
        return false;
    }
}

polygonSeparationData_type CollisionDetector::calculateMinPolygonSeparation(Polygon & i_body1, Polygon & i_body2) const {

    polygonSeparationData_type sepData;
    int normalIndex = 0;

    std::vector<int> preliminaryCollIndexVec;
    std::vector<int> preliminaryCollIndexVec2;

    // Loop through all vertices for Body1 and get normal vector
    for (int i = 0; i < i_body1.getPointCount(); i++) {
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

circleSeparationData_type CollisionDetector::calculateMinCircleSeparation(Polygon & i_polygon, Circle & i_circle) const {
    float radius = i_circle.getRadius();
    circleSeparationData_type separationData;

    // Modified SAT algorithm
    for (int i = 0; i < i_polygon.getPointCount(); i++) {
        sf::Vector2f testPoint = i_polygon.getGlobalPoint(i);
        sf::Vector2f relativeCirclePosition = sfu::subtractVectors(i_circle.getPosition(), testPoint);

        float newCornerSeparation = sfu::getVectorLength(relativeCirclePosition) - radius;
        if (newCornerSeparation < separationData.cornerSeparation) {
            separationData.cornerSeparation = newCornerSeparation;
            separationData.pointIndex = i;
        }

        sf::Vector2f testNormal = i_polygon.getGlobalNormal(i);
        sf::Vector2f pointConnector = sfu::subtractVectors(i_circle.getPosition(), testPoint);
        // std::cout << testPoint.x << ", " << testPoint.y << "\n";
        float newEdgeSeparation = sfu::scalarProduct(testNormal, pointConnector) - radius;
        // Edge separation is the maximum value of all possible edge separations
        // std::cout << newEdgeSeparation << "\n";
        if (newEdgeSeparation > separationData.edgeSeparation) {
            // minSep = std::min(minSep, dotProd);
            separationData.edgeSeparation = newEdgeSeparation; // previous indices are irrelevant
            separationData.normalIndex = i;                     // save index
        }
    }

    return separationData;
}
