#include "Simulation.hpp"
#include "CollisionDetector.hpp"
#include "iostream"
#include "stdlib.h"
#include "sfml_utility.hpp"

const float PLAYER_VELOCITY = 500.0f;        // pixels per second
const float PLAYER_ANGULAR_VELOCITY = 22.5f; // degrees per second

Simulation * Simulation::s_instance = nullptr; // pointer to Singleton instance
std::mutex Simulation::mtx;

Simulation * Simulation::getInstance() {
    if (s_instance == nullptr) {
        std::lock_guard<std::mutex> lock(mtx);
        if (s_instance == nullptr) {
            s_instance = new Simulation();
        }
    }
    return s_instance;
}

Simulation::Simulation() {}

Simulation::~Simulation() {
    // Clean up collisionPartners if Simulation owns the RigidBody pointers
    for (RigidBody * body : m_collisionPartners) {
        delete body; // Assuming Simulation owns the RigidBody objects
    }
    m_collisionPartners.clear();

    // Clean up pointMarkers
    for (sf::RectangleShape * marker : m_pointMarkers) {
        delete marker; // Assuming Simulation owns these
    }
    m_pointMarkers.clear();

    // Clean up axisMarkers
    for (sf::RectangleShape * marker : m_axisMarkers) {
        delete marker; // Assuming Simulation owns these
    }
    m_axisMarkers.clear();

    // Properly close the render window
    if (m_window.isOpen()) {
        m_window.close();
    }
}

void Simulation::run() {

    m_clock.restart();

    while (m_window.isOpen()) {
        // Use crappy polling algorithm because sf::Clock doesn't support anything else
        while (m_clock.getElapsedTime().asSeconds() < m_dT) {} // wait until m_dT is elapsed
        handleEvents();
        update();
        m_clock.restart();
        // m_dT = clock.restart().asSeconds();
    }
}

void Simulation::addCollisionPartner(RigidBody * r_collisionPartner) {
    m_collisionPartners.push_back(r_collisionPartner);
    r_collisionPartner->setOutlineColor(sf::Color::Red);
    r_collisionPartner->setFillColor(sf::Color::Black);
    r_collisionPartner->setOutlineThickness(-2.0f);
}

void Simulation::deleteCollisionPartner(int i_index) {
    RigidBody * colParToDelete = m_collisionPartners[i_index];
    delete colParToDelete;
    m_collisionPartners.erase(m_collisionPartners.begin() + i_index);
}

void Simulation::initWindow(float i_viewWidth, float i_viewHeight, float i_frameRate) {
    m_window.create(sf::VideoMode(i_viewWidth, 512), "SFML 2D collision Simulation",
            sf::Style::Close | sf::Style::Titlebar | sf::Style::Resize);
    m_view = sf::View(sf::Vector2f(i_viewWidth / 2, i_viewHeight / 2), sf::Vector2f(i_viewWidth, i_viewHeight));
    m_dT = 1 / i_frameRate;
}

// prepare Bodies
void Simulation::initBodies(std::vector<RigidBody *> i_rigidBodies) {

    // std::vector<sf::Vector2f> marker1 = { sf::Vector2f(5.0f, -5.0f), sf::Vector2f(-5.0f, -5.0f), sf::Vector2f(-5.0f, 5.0f),
    // sf::Vector2f(5.0f, 5.0f)   };
    m_pointMarkers.push_back(new sf::RectangleShape({10.0f, 10.0f}));
    m_axisMarkers.push_back(new sf::RectangleShape({50.0f, 0.0f}));
    m_axisMarkers.push_back(new sf::RectangleShape({0.0f, 50.0f}));

    for (RigidBody * rBody : i_rigidBodies) {
        addCollisionPartner(rBody);
    }

    for (sf::RectangleShape * marker : m_pointMarkers) {
        marker->setOrigin({5.0f, 5.0f});
        marker->setOutlineColor(sf::Color::Red);
        marker->setFillColor(sf::Color::Black);
        marker->setOutlineThickness(-2.0f);
    }

    for (sf::RectangleShape * marker : m_axisMarkers) {
        marker->setOutlineColor(sf::Color::Red);
        marker->setFillColor(sf::Color::Black);
        marker->setOutlineThickness(-1.0f);
    }
}

void Simulation::update() {
    m_window.setView(m_view); // update view
    m_window.clear();         // remove old Objects
    for (int i = 0; i < m_collisionPartners.size(); i++) {
        for (int j = 0; j < i; j++) {
            // collision detection
            CollisionEvent collEvent(m_collisionPartners[i], m_collisionPartners[j]);
            if (m_cd->detectCollision(collEvent)) {
                // std::cout << collEvent.normal1.x << ", " << collEvent.normal1.y << ", "  << collEvent.normal2.x << ", " <<
                // collEvent.normal2.y << "\n"; collisionPartners[i]->setOutlineColor(sf::Color::Blue);
                // collisionPartners[j]->setOutlineColor(sf::Color::Blue);
                m_pointMarkers[0]->setPosition(collEvent.m_collisionLocation);
                m_axisMarkers[0]->setPosition(collEvent.m_collisionLocation);
                m_axisMarkers[1]->setPosition(collEvent.m_collisionLocation);
                // std::cout << "Position in Simulation: " << collEvent.collLoc1.x << ", " << collEvent.collLoc1.y << "\n";
                m_axisMarkers[0]->setRotation(sfu::getVectorDirection(collEvent.m_contactNormal));
                m_axisMarkers[1]->setRotation(sfu::getVectorDirection(collEvent.m_contactNormal));
                collEvent.resolve();
            }
        }
        m_window.draw(*m_collisionPartners[i]);
        m_collisionPartners[i]->updatePositionAndAngle(m_dT);
    }
    for (sf::RectangleShape * marker : m_pointMarkers) {
        m_window.draw(*marker);
    }
    for (sf::RectangleShape * marker : m_axisMarkers) {
        m_window.draw(*marker);
    }
    for (int i = 0; i < m_collisionPartners.size(); i++) {}

    m_window.display(); // render the frame
}

// handle user input etc.
void Simulation::handleEvents() {
    sf::Event event;
    while (m_window.pollEvent(event)) {
        sf::Vector2u newSize;

        switch (event.type) {
        case sf::Event::Closed:
            Simulation::~Simulation(); // destroy Simulation; close window
            break;
        case sf::Event::Resized: // change window size and adapt view
            std::cout << "New window width: " << event.size.width << ", New window height: " << event.size.height << std::endl;
            m_view.setSize(m_window.getSize().x, m_window.getSize().y);                 // adapt view size
            m_view.setCenter(m_window.getSize().x / 2.0f, m_window.getSize().y / 2.0f); // adapt view center
            break;
        }

        // keyboard control
        float movIncr = m_dT * PLAYER_VELOCITY;
        float angIncr = m_dT * PLAYER_ANGULAR_VELOCITY;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {
            m_collisionPartners[0]->move(-movIncr, 0.0f);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {
            m_collisionPartners[0]->move(movIncr, 0.0f);
        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {
            m_collisionPartners[0]->move(0.0f, movIncr);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
            m_collisionPartners[0]->move(0.0f, -movIncr);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) {
            m_collisionPartners[0]->rotate(angIncr);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) {
            m_collisionPartners[0]->rotate(-angIncr);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space)) {}

        // mouse control
        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            sf::Vector2i mousePos = sf::Mouse::getPosition(m_window);
            m_collisionPartners[0]->setPosition(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
        }
    }
}
