#pragma once
#include "sfml/Graphics.hpp"
#include "RigidBody.hpp"
//#include "CollisionDetector.hpp"
#include "CollisionResolver.hpp"
#include "vector"

#define VIEW_HEIGHT 512.0f
#define VIEW_WIDTH 512.0f

class Simulation
{
public:
	static Simulation* getInstance();
	~Simulation();

	void run();

	//static Simulation* instance;
	float m_dT = 0.0005; // TODO use clock for this?

	sf::RenderWindow m_window;

	
	

private:
	sf::Clock clock;
	std::vector<RigidBody*>collisionPartners; //TODO use reference instead of pointer?
	std::vector<sf::RectangleShape*>pointMarkers;
	std::vector<sf::RectangleShape*>axisMarkers;
	Simulation();
	static Simulation * s_instance;
	CollisionDetector * m_cd = CollisionDetector::getInstance();
	CollisionResolver * m_cr = CollisionResolver::getInstance();
	

	sf::View m_view;

	void update();
	void initWindow();
	void initBodies();
	void handleEvents();

	



};

