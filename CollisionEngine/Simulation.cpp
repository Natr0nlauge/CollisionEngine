#include "Simulation.hpp"
#include "CollisionDetector.hpp"
#include "iostream"
#include "stdlib.h"


const float PLAYER_VELOCITY = 500.0f; //pixels per second
const float PLAYER_ANGULAR_VELOCITY = 22.5f; //degrees per second

Simulation* Simulation::s_instance = nullptr; //pointer to Singleton instance
//CollisionDetector* s_cd = CollisionDetector::getInstance();

//float Simulation::s_dT = 0.1f;

Simulation::Simulation()
{
	//Initialize everything belonging to Simulation
	initWindow();
	initBodies();

}

Simulation* Simulation::getInstance()
{
	if (s_instance == nullptr) {
		//std::lock_guard<std::mutex> lock(mtx);
		if (s_instance == nullptr) {
			s_instance = new Simulation();
		}
	}
	return s_instance;
}

Simulation::~Simulation()
{
	//Delete everything belonging to Simulation
}

void Simulation::run()
{

	clock.restart();

	while (m_window.isOpen()) {

		handleEvents();
		update();

		m_dT = clock.restart().asSeconds();

	}
}

void Simulation::update()
{
	m_window.setView(m_view); //update view
	m_window.clear(); //remove old Objects
	for (int i = 0; i < collisionPartners.size(); i++)
		m_window.draw(*collisionPartners[i]);
	m_window.display(); //render the frame
}

//prepare window and view
void Simulation::initWindow()
{
	m_window.create(sf::VideoMode(512, 512), "SFML Tutorial", sf::Style::Close | sf::Style::Titlebar | sf::Style::Resize);
	m_view = sf::View(sf::Vector2f(VIEW_HEIGHT / 2, VIEW_HEIGHT / 2), sf::Vector2f(VIEW_HEIGHT, VIEW_HEIGHT));

}

//prepare Bodies
void Simulation::initBodies() {
	std::vector<sf::Vector2f> exampleVertices = { sf::Vector2f(25.0f, -50.0f), sf::Vector2f(-25.0f, -50.0f),sf::Vector2f(-50.0f, 0.0f), sf::Vector2f(-50.0f, 25.0f), sf::Vector2f(25.0f, 25.0f)     };
	collisionPartners.push_back(new Polygon(exampleVertices));
	std::vector<sf::Vector2f> exampleVertices2 = { sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-50.0f, 0.0f), sf::Vector2f(-75.0f, 225.0f), sf::Vector2f(25.0f, 25.0f)     };
	collisionPartners.push_back(new Polygon(exampleVertices2));
	std::vector<sf::Vector2f> marker1 = { sf::Vector2f(5.0f, -5.0f), sf::Vector2f(-5.0f, -5.0f), sf::Vector2f(-5.0f, 5.0f), sf::Vector2f(5.0f, 5.0f)   };
	collisionPartners.push_back(new Polygon(marker1));
	



	for (RigidBody * colPar : collisionPartners) {
		colPar->setOutlineColor(sf::Color::Red);
		colPar->setFillColor(sf::Color::Black);
		colPar->setOrigin(0.0f, 0.0f);
		colPar->setOutlineThickness(-2.0f);
		colPar->setOutlineColor(sf::Color::Red);
	}
	
	collisionPartners[1]->setPosition(200.0f, 200.0f);
}

//handle user input etc.
void Simulation::handleEvents()
{
	sf::Event event;
	while (m_window.pollEvent(event)) {
		sf::Vector2u newSize;

		switch (event.type) {
		case sf::Event::Closed:
			m_window.close();	//close window
			break;
		case sf::Event::Resized: //change window size and adapt view
			std::cout << "New window width: " << event.size.width << ", New window height: " << event.size.height << std::endl;
			m_view.setSize(m_window.getSize().x, m_window.getSize().y); //adapt view size
			m_view.setCenter(m_window.getSize().x / 2, m_window.getSize().y / 2); //adapt view center
			break;

		}

		//keyboard control
		float movIncr = m_dT * PLAYER_VELOCITY;
		float angIncr = m_dT * PLAYER_ANGULAR_VELOCITY;
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {
			collisionPartners[0]->move(-movIncr, 0.0f);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {
			collisionPartners[0]->move(movIncr, 0.0f);
		}

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {
			collisionPartners[0]->move(0.0f, movIncr);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
			collisionPartners[0]->move(0.0f, -movIncr);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) {
			collisionPartners[0]->rotate(angIncr);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) {
			collisionPartners[0]->rotate(-angIncr);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space)) {
			//std::cout << ((static_cast<Polygon*>(collisionPartners[0]))->getGlobalPoints()[0]).x << ", " << ((static_cast<Polygon*>(collisionPartners[0]))->getGlobalPoints()[0]).y << "\n";
			for (int i = 0; i < collisionPartners[0]->getPointCount(); i++) {
				std::cout << ((static_cast<Polygon*>(collisionPartners[0]))->getGlobalNormal(i)).x << ", " << ((static_cast<Polygon*>(collisionPartners[0]))->getGlobalNormal(i)).y << "\n";
			}
		}

		//mouse control
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			sf::Vector2i mousePos = sf::Mouse::getPosition(m_window);
			collisionPartners[0]->setPosition(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
		}

		//collision detection
		Polygon* lvalue1 = static_cast<Polygon*>(collisionPartners[0]);
		Polygon* lvalue2 = static_cast<Polygon*>(collisionPartners[1]);
		sf::Vector2f colLoc;
		if (s_cd->detectCollision(*lvalue1, *lvalue2, colLoc)) {
			collisionPartners[0]->setOutlineColor(sf::Color::Blue);
			collisionPartners[1]->setOutlineColor(sf::Color::Blue);
			collisionPartners[2]->setPosition(colLoc);
		}
		else {
			collisionPartners[0]->setOutlineColor(sf::Color::Red);
			collisionPartners[1]->setOutlineColor(sf::Color::Red);
			collisionPartners[2]->setPosition(sf::Vector2f(0.0f, 0.0f));
		}

	}
}
