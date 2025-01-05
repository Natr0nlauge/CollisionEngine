#include "Simulation.hpp"
#include "CollisionDetector.hpp"
#include "iostream"
#include "stdlib.h"
#include "sfmlUtility.hpp"

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
	//TODO: Delete everything belonging to Simulation
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
	//TODO: change loop
	for (int i = 0; i < collisionPartners.size(); i++) {
		m_window.draw(*collisionPartners[i]);
		collisionPartners[i]->updatePositionAndAngle(m_dT);
	}
	for (sf::RectangleShape* marker : pointMarkers) {
		m_window.draw(*marker);
	}
	for (sf::RectangleShape* marker : axisMarkers) {
		m_window.draw(*marker);
	}
	for (int i = 0; i < collisionPartners.size(); i++) {
		for (int j = 0; j < i; j++) {
			//collision detection
			collisionEvent collEvent(*collisionPartners[i], *collisionPartners[j]);
			if (m_cd->detectCollision(collEvent)) {
				//std::cout << collEvent.normal1.x << ", " << collEvent.normal1.y << ", "  << collEvent.normal2.x << ", " << collEvent.normal2.y << "\n";
				collisionPartners[i]->setOutlineColor(sf::Color::Blue);
				collisionPartners[j]->setOutlineColor(sf::Color::Blue);
				pointMarkers[0]->setPosition(collEvent.collLoc1);
				axisMarkers[0]->setPosition(collEvent.collLoc1);
				axisMarkers[1]->setPosition(collEvent.collLoc1);
				//std::cout << "Position in Simulation: " << collEvent.collLoc1.x << ", " << collEvent.collLoc1.y << "\n";
				axisMarkers[0]->setRotation(sfu::getVectorDirection(collEvent.normal2));
				axisMarkers[1]->setRotation(sfu::getVectorDirection(collEvent.normal2));
				m_cr->handleCollision(collEvent);
			}
			else {
				//collisionPartners[i]->setOutlineColor(sf::Color::Red);
				//collisionPartners[j]->setOutlineColor(sf::Color::Red);
				//pointMarkers[0]->setPosition(sf::Vector2f(0.0f, 0.0f));
				//axisMarkers[0]->setPosition(sf::Vector2f(0.0f, 0.0f));
				//axisMarkers[1]->setPosition(sf::Vector2f(0.0f, 0.0f));
				
			}
			
		}
	}
	collisionEvent collEvent(*collisionPartners[0], *collisionPartners[1]);
	//if (m_cd->detectCollision(collEvent)) {
	//	//std::cout << collEvent.normal1.x << ", " << collEvent.normal1.y << ", "  << collEvent.normal2.x << ", " << collEvent.normal2.y << "\n";
	//	collisionPartners[0]->setOutlineColor(sf::Color::Blue);
	//	collisionPartners[1]->setOutlineColor(sf::Color::Blue);
	//	pointMarkers[0]->setPosition(collEvent.collLoc1);
	//	axisMarkers[0]->setPosition(collEvent.collLoc1);
	//	axisMarkers[1]->setPosition(collEvent.collLoc1);
	//	axisMarkers[0]->setRotation(sfu::getVectorDirection(collEvent.normal1));
	//	axisMarkers[1]->setRotation(sfu::getVectorDirection(collEvent.normal1));
	//	m_cr->handleCollision(collEvent);
	//}
	//else {
	//	//colPar1->setOutlineColor(sf::Color::Red);
	//	//colPar2->setOutlineColor(sf::Color::Red);
	//	pointMarkers[0]->setPosition(sf::Vector2f(0.0f, 0.0f));
	//	axisMarkers[0]->setPosition(sf::Vector2f(0.0f, 0.0f));
	//	axisMarkers[1]->setPosition(sf::Vector2f(0.0f, 0.0f));

	//}
	

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
	//std::vector<sf::Vector2f> exampleVertices = { sf::Vector2f(25.0f, -50.0f), sf::Vector2f(-25.0f, -50.0f),sf::Vector2f(-50.0f, 0.0f), sf::Vector2f(-50.0f, 25.0f), sf::Vector2f(25.0f, 25.0f)     };
	std::vector<sf::Vector2f> exampleVertices = { sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f),sf::Vector2f(-25.0f, 25.0f), sf::Vector2f(25.0f, 25.0f) };
	collisionPartners.push_back(new Polygon(0.1,exampleVertices));
	//std::vector<sf::Vector2f> exampleVertices2 = { sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(-50.0f, 0.0f), sf::Vector2f(-75.0f, 225.0f), sf::Vector2f(25.0f, 25.0f)     };
	std::vector<sf::Vector2f> exampleVertices2 = { sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f),sf::Vector2f(-25.0f, 25.0f), sf::Vector2f(25.0f, 25.0f) };
	collisionPartners.push_back(new Polygon(0.1,exampleVertices2));
	
	// Simulation border TODO: Replace with a seperate class
	std::vector<sf::Vector2f> borderVertices3 = { sf::Vector2f(0.0f, 50.0f), sf::Vector2f(450.0f, 50.0f), sf::Vector2f(450.0f, 0.0f), sf::Vector2f(0.0f, 0.0f)   };
	collisionPartners.push_back(new Polygon(0.0, borderVertices3));
	std::vector<sf::Vector2f> borderVertices4 = { sf::Vector2f(0.0f, 50.0f), sf::Vector2f(450.0f, 50.0f), sf::Vector2f(450.0f, 0.0f), sf::Vector2f(0.0f, 0.0f) };
	collisionPartners.push_back(new Polygon(0.0, borderVertices4));
	std::vector<sf::Vector2f> borderVertices5 = { sf::Vector2f(0.0f, 450.0f), sf::Vector2f(50.0f, 450.0f), sf::Vector2f(50.0f, 0.0f), sf::Vector2f(0.0f, 0.0f) };
	collisionPartners.push_back(new Polygon(0.0, borderVertices5));
	std::vector<sf::Vector2f> borderVertices6 = { sf::Vector2f(0.0f, 450.0f), sf::Vector2f(50.0f, 450.0f), sf::Vector2f(50.0f, 0.0f), sf::Vector2f(0.0f, 0.0f) };
	collisionPartners.push_back(new Polygon(0.0, borderVertices6));
	
	
	
	
	//std::vector<sf::Vector2f> marker1 = { sf::Vector2f(5.0f, -5.0f), sf::Vector2f(-5.0f, -5.0f), sf::Vector2f(-5.0f, 5.0f), sf::Vector2f(5.0f, 5.0f)   };
	pointMarkers.push_back(new sf::RectangleShape({10.0f,10.0f}));
	axisMarkers.push_back(new sf::RectangleShape({50.0f,0.0f }));
	axisMarkers.push_back(new sf::RectangleShape({0.0f,50.0f }));
	



	for (RigidBody * colPar : collisionPartners) {
		colPar->setOutlineColor(sf::Color::Red);
		colPar->setFillColor(sf::Color::Black);
		colPar->setOutlineThickness(-2.0f);
		colPar->setOutlineColor(sf::Color::Red);
	}

	for (sf::RectangleShape* marker : pointMarkers) {
		marker->setOrigin({5.0f,5.0f});
		marker->setOutlineColor(sf::Color::Red);
		marker->setFillColor(sf::Color::Black);
		marker->setOutlineThickness(-2.0f);
		marker->setOutlineColor(sf::Color::Red);
	}

	for (sf::RectangleShape* marker : axisMarkers) {
		marker->setOutlineColor(sf::Color::Red);
		marker->setFillColor(sf::Color::Black);
		marker->setOutlineThickness(-1.0f);
		marker->setOutlineColor(sf::Color::Red);
	}
	
	collisionPartners[0]->setPosition(200.0f, 200.0f);
	collisionPartners[0]->setVelocity(sf::Vector2f(50.0f, 0.0f));
	collisionPartners[0]->setAngularVelocity(10.0f);
	collisionPartners[0]->setRotation(20.0f);
	collisionPartners[1]->setPosition(400.0f, 400.0f);
	//collisionPartners[1]->setVelocity(sf::Vector2f(-50.0f, 0.0f));
	collisionPartners[1]->setAngularVelocity(0.0f);



	collisionPartners[2]->setPosition(256.0f, 5.0f);
	collisionPartners[3]->setPosition(256.0f, 507.0f);
	collisionPartners[4]->setPosition(5.0f, 256.0f);
	collisionPartners[5]->setPosition(507.0f, 256.0f);
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
			/*for (int i = 0; i < collisionPartners[0]->getPointCount(); i++) {
				std::cout << ((static_cast<Polygon*>(collisionPartners[0]))->getGlobalNormal(i)).x << ", " << ((static_cast<Polygon*>(collisionPartners[0]))->getGlobalNormal(i)).y << "\n";
			}*/
		}

		//mouse control
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			sf::Vector2i mousePos = sf::Mouse::getPosition(m_window);
			collisionPartners[0]->setPosition(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
		}

	}
}
