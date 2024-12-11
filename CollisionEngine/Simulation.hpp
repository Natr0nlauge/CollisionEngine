#pragma once
#include "sfml/Graphics.hpp"
#include "RigidBody.hpp"
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
	float m_dT = 0;

	sf::RenderWindow m_window;

	
	

private:
	sf::Clock clock;
	std::vector<RigidBody*>collisionPartners;
	Simulation();
	static Simulation* s_instance;

	sf::Texture *playerTexture = nullptr;//TODO remove
	

	sf::View m_view;

	void update();
	void initWindow();
	void initBodies();
	void handleEvents();



};

