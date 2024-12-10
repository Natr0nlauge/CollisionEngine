#include "Simulation.hpp"
#include "iostream"
#include "stdlib.h"

////TODO: Welche Funktionen davon sind wirklich nötig? Gescheite constructors, bessere Struktur 
////TODO: Simulation als Singleton
////TODO: Paar Kommentare, Header Files umstrukturieren
////TODO: git commit
//TODO: Collision Partners vernünftig aufbauen, mehrere einführen
//TODO: Gescheite Clock/dT
//TODO: Steuerung für die Drehung
//TODO: Coding Standards anschauen




Simulation* Simulation::instance = nullptr; //pointer to Singleton instance

float Simulation::dT = 0.0f;

Simulation::Simulation()
{
	//Initialize everything belonging to Simulation
	initWindow();
	initBodies();

}

Simulation* Simulation::getInstance()
{
	if (instance == nullptr) {
		//std::lock_guard<std::mutex> lock(mtx);
		if (instance == nullptr) {
			instance = new Simulation();
		}
	}
	return instance;
}

Simulation::~Simulation()
{
	//Delete everything belonging to Simulation
}

void Simulation::run()
{

	clock.restart();

	while(window.isOpen()){
		
		handleEvents();
		update();

	}
}

void Simulation::update()
{
	window.setView(view); //update view
	window.clear(); //remove old Objects
	window.draw(*collisionPartners[0]); //display updated Objects in next frame
	window.display(); //render the frame
}

//prepare window and view
void Simulation::initWindow()
{
	window.create(sf::VideoMode(512, 512), "SFML Tutorial", sf::Style::Close | sf::Style::Titlebar | sf::Style::Resize);
	view = sf::View(sf::Vector2f(VIEW_HEIGHT / 2, VIEW_HEIGHT / 2), sf::Vector2f(VIEW_HEIGHT, VIEW_HEIGHT));
	
}

//prepare Bodies
void Simulation::initBodies() {
	collisionPartners.push_back(new Polygon());
	//sf::RectangleShape player = sf::RectangleShape(sf::Vector2f(100.0f, 100.0f));
	playerTexture = new sf::Texture;
	playerTexture->loadFromFile("texture.png");
	//collisionPartners[0]->setTexture(playerTexture);
	collisionPartners[0]->setOutlineColor(sf::Color::Red);
	collisionPartners[0]->setFillColor(sf::Color::Blue);
	collisionPartners[0]->setOrigin(0.0f, 0.0f);
	collisionPartners[0]->setOutlineThickness(5.0f);
}

//handle user input etc.
void Simulation::handleEvents()
{
	sf::Event event;
	while (window.pollEvent(event)) {
		sf::Vector2u newSize;

		switch (event.type) {
		case sf::Event::Closed: 
			window.close();	//close window
			break;
		case sf::Event::Resized: //change window size and adapt view
			std::cout << "New window width: " << event.size.width << ", New window height: " << event.size.height << std::endl;
			//newSize = window.getSize(); //can't be used in view.setSize because it has to be vector2f
			view.setSize(window.getSize().x, window.getSize().y); //adapt view size
			view.setCenter(window.getSize().x/2, window.getSize().y/2); //adapt view center
			break;

		case sf::Event::TextEntered: //log pressed key (pretty much useless)

			if (event.text.unicode < 128) {
				printf("%c", event.text.unicode);
			}
		}

	}

	//keyboard control
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {
		collisionPartners[0]->move(-0.1f, 0.0f);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {
		collisionPartners[0]->move(0.1f, 0.0f);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {
		collisionPartners[0]->move(0.0f, 0.1f);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
		collisionPartners[0]->move(0.0f, -0.1f);

	}

	//mouse control
	if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
		sf::Vector2i mousePos = sf::Mouse::getPosition(window);
		collisionPartners[0]->setPosition(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));

	}


	
}
