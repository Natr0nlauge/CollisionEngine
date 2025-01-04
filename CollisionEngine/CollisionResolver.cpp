#include "CollisionResolver.hpp"
#include <iostream>

CollisionResolver* CollisionResolver::s_instance = nullptr;


CollisionResolver::~CollisionResolver() {
}

void CollisionResolver::handleCollision(collisionEvent& c_collEvent) {

	//TODO make this more readable, remove repetitions

	float contactTransformationAngle = sfu::getVectorDirection(c_collEvent.normal1);
	//TODO: check which normal to take, do I even need two normals?

	//TODO: what happens if 3 or more objects collide?

	//TODO think about making c_collEvent a class

	// Get relative position in global coordinates for body 1 and 2
	sf::Vector2f relativePosition1 = sfu::subtractVectors(c_collEvent.collLoc1, c_collEvent.rBody1.getPosition());
	sf::Vector2f relativePosition2 = sfu::subtractVectors(c_collEvent.collLoc1, c_collEvent.rBody2.getPosition());
	//sf::Vector2f relativePosition2 = sfu::subtractVectors(c_collEvent.collLoc1, c_collEvent.rBody2.getPosition());

	// Implement the three equations, see p. 338
	float torquePerUnitImpulse1 = sfu::pseudoCrossProduct(relativePosition1, c_collEvent.normal1);
	float torquePerUnitImpulse2 = sfu::pseudoCrossProduct(relativePosition2, c_collEvent.normal2);
	float inverseMomentOfInertia1 = c_collEvent.rBody1.getInverseMomentOfInertia();
	float inverseMomentOfInertia2 = c_collEvent.rBody2.getInverseMomentOfInertia();
	float rotationPerUnitImpulse1 = inverseMomentOfInertia1 * torquePerUnitImpulse1;
	float rotationPerUnitImpulse2 = inverseMomentOfInertia2 * torquePerUnitImpulse2;
	// simplification of cross product //TODO might want to make this a seperate function
	sf::Vector2f velocityPerUnitImpulse1 = sf::Vector2f(-rotationPerUnitImpulse1 * relativePosition1.y, rotationPerUnitImpulse1 * relativePosition1.x);
	sf::Vector2f velocityPerUnitImpulse2 = sf::Vector2f(-rotationPerUnitImpulse2 * relativePosition2.y, rotationPerUnitImpulse2 * relativePosition2.x);

	// deltaVel is the total velocity change per unit impulse
	// Splitting into summands for debugging
	float deltaVelSummand1 = sfu::scalarProduct(velocityPerUnitImpulse1, c_collEvent.normal1);
	float deltaVelSummand2 = sfu::scalarProduct(velocityPerUnitImpulse2, c_collEvent.normal2);

	float deltaVelSummand3 = c_collEvent.rBody1.getInverseMass();
	float deltaVelSummand4 = c_collEvent.rBody2.getInverseMass();

	float deltaVel = deltaVelSummand1 + deltaVelSummand2 + deltaVelSummand3 + deltaVelSummand4;

	// Calculate closing velocity at contact point
	float angVel1 = c_collEvent.rBody1.getAngularVelocity();
	float angVel2 = c_collEvent.rBody2.getAngularVelocity();
	// simplification of cross product
	sf::Vector2f closingVel1 = sf::Vector2f(-angVel1 * relativePosition1.y, angVel1 * relativePosition1.x);
	sf::Vector2f closingVel2 = sf::Vector2f(-angVel2 * relativePosition2.y, angVel2 * relativePosition2.x);
	closingVel1 += c_collEvent.rBody1.getVelocity();
	closingVel2 += c_collEvent.rBody2.getVelocity();

	sf::Vector2f closingVel = sfu::subtractVectors(closingVel1, closingVel2);
	
	// Calculating desired velocity change (slightly modified)
	//sf::Vector2f contactVel1 = sfu::rotateVector(closingVel1, contactTransformationAngle);
	float contactVel1 = sfu::scalarProduct(closingVel1, c_collEvent.normal1);
	//float contactVel = sfu::scalarProduct(closingVel, c_collEvent.normal1);
	

	// Projected closing velocity - Are objects moving towards each other or not?
	float closingVel1Proj = sfu::scalarProduct(closingVel1,c_collEvent.normal1);
	float closingVel2Proj = sfu::scalarProduct(closingVel2, c_collEvent.normal2);
	float contactVel = closingVel1Proj + closingVel2Proj;
	std::cout << contactVel << "\n";

	if (contactVel > 0)
		{
		float restitution = 1.0f; //depends on material
		float desiredDeltaVel1 = -contactVel1 * (1 + restitution);
		float desiredDeltaVel = -contactVel * (1 + restitution);

		sf::Vector2f impulse1Contact = sf::Vector2f(desiredDeltaVel1 / deltaVel, 0.0f);
		sf::Vector2f impulseContact = sf::Vector2f(desiredDeltaVel / deltaVel, 0.0f);

		// In global
		//sf::Vector2f impulse1 = sfu::rotateVector(impulse1Contact, -contactTransformationAngle);
		
		sf::Vector2f impulse1 = sfu::rotateVector(impulseContact, -contactTransformationAngle);
		sf::Vector2f impulse2 = sfu::scaleVector(impulse1,-1.0f); // Newton's third law

		//sf::Vector2f velocityChange1 = sfu::scaleVector(impulse1,c_collEvent.rBody1.getInverseMass());
		sf::Vector2f velocityChange1 = sfu::scaleVector(impulse1, c_collEvent.rBody1.getInverseMass());
		sf::Vector2f velocityChange2 = sfu::scaleVector(impulse2, c_collEvent.rBody2.getInverseMass());
		//velocityChange2 = sfu::scaleVector(velocityChange2, -1.0f);

		float impulsiveTorque1 = sfu::pseudoCrossProduct( relativePosition1, impulse1);
		float impulsiveTorque2 = sfu::pseudoCrossProduct( relativePosition2, impulse2);
		float angularVelocityChange1 = c_collEvent.rBody1.getInverseMomentOfInertia() * impulsiveTorque1;
		float angularVelocityChange2 = c_collEvent.rBody2.getInverseMomentOfInertia() * impulsiveTorque2;

		sf::Vector2f newVel1 = sfu::addVectors(c_collEvent.rBody1.getVelocity(), velocityChange1);
		float newAngVel1 = c_collEvent.rBody1.getAngularVelocity() + angularVelocityChange1;
		sf::Vector2f newVel2 = sfu::addVectors(c_collEvent.rBody2.getVelocity(), velocityChange2);
		float newAngVel2 = c_collEvent.rBody2.getAngularVelocity() + angularVelocityChange2;
		c_collEvent.rBody1.setVelocity(newVel1);
		c_collEvent.rBody2.setVelocity(newVel2);
		c_collEvent.rBody1.setAngularVelocity(newAngVel1);
		c_collEvent.rBody2.setAngularVelocity(newAngVel2);
   		std::cout << newVel1.x << ", " << newVel1.y << ", " << newVel2.x << ", " << newVel2.y << "\n";
	}
}





CollisionResolver* CollisionResolver::getInstance() {
	if (s_instance == nullptr) {
		//TODO: check thread safety
		//std::lock_guard<std::mutex> lock(mtx);
		if (s_instance == nullptr) {
			s_instance = new CollisionResolver();
		}
	}
	return s_instance;
}



CollisionResolver::CollisionResolver()
{
}

//sf::Vector2f CollisionResolver::transformVectorToContact(sf::Vector2f i_globalVec)
//{
//
//	return sf::Vector2f();
//}
//
//sf::Vector2f CollisionResolver::transformPointToContact(sf::Vector2f i_globalPoint)
//{
//	//float theta = sfu::getVectorDirection();
//	return sf::Vector2f();
//}




