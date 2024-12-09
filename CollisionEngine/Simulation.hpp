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
	static float dT;

	sf::RenderWindow window;

	
	

private:
	sf::Clock clock;
	std::vector<RigidBody*>collisionPartners;
	Simulation();
	static Simulation* instance;

	sf::Texture *playerTexture = nullptr;//TODO remove
	

	sf::View view;

	void update();
	void initWindow();
	void initBodies();
	void handleEvents();



};

