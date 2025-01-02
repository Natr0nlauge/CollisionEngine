#include "CollisionResolver.hpp"

CollisionResolver* CollisionResolver::s_instance = nullptr;


CollisionResolver::~CollisionResolver(){
}

void CollisionResolver::handleCollision(collisionEvent& c_collEvent){
	c_collEvent.rBody1.setVelocity({0.0f,0.0f});
	c_collEvent.rBody2.setVelocity({ 0.0f,0.0f });
	c_collEvent.rBody1.setAngularVelocity(0.0f);
	c_collEvent.rBody2.setAngularVelocity(0.0f);
	
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




