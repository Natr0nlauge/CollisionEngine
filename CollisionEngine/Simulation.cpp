#include "Simulation.hpp"
#include "CollisionDetector.hpp"
#include "iostream"
#include "stdlib.h"
#include "sfml_utility.hpp"

const float PLAYER_VELOCITY = 500.0f;
const float PLAYER_ANGULAR_VELOCITY = 22.5f; // degrees per second

std::unique_ptr<Simulation> Simulation::s_instance = nullptr; // pointer to Singleton instance
std::mutex Simulation::mtx;

/**
 * @brief Retrieves the singleton instance of the class.
 *
 * This method provides access to the single instance of the Singleton class.
 * If the instance does not already exist, it is created.
 *
 * @return A reference to the Simulation.
 */
Simulation & Simulation::getInstance() {
    if (s_instance == nullptr) {
        std::lock_guard<std::mutex> lock(mtx);
        if (!s_instance) {
            s_instance.reset(new Simulation());
        }
    }
    return *s_instance;
}

/**
 * @brief Constructor.
 */
Simulation::Simulation() {}

/**
 * @brief Destructor.
 */
Simulation::~Simulation() {
    // Clean up all the members to avoid memory leaks
    cleanupMember(m_collisionPartners);
    delete m_collisionLocationMarker;
    delete m_collisionNormalMarkers[0];
    delete m_collisionNormalMarkers[1];

    // Properly close the render window
    if (m_window.isOpen()) {
        m_window.close();
    }
}

/**
 * @brief Cleans up and clears a vector of dynamically allocated objects.
 *
 * This method deletes all dynamically allocated objects in the given vector
 * and clears the vector to release any remaining memory it holds.
 *
 * @tparam T The type of the objects stored in the vector.
 * @param member A reference to the vector containing pointers to objects of type T.
 */
template <typename T> inline void Simulation::cleanupMember(std::vector<T *> & member) {
    for (T * element : member) {
        delete element;
    }
    member.clear();
}

/**
 * @brief Opens the window and starts running the simulation.
 *
 * This initiates the Simulation and opens the window to display it.
 *
 * It stays active until the window is closed and handles timing of the frames. The handleEvents() and update() methods are called every
 * frame.
 *
 */
void Simulation::run() {

    initCollisionMarkers();
    initWindow();
    m_clock.restart();

    while (m_window.isOpen()) {
        // Wait until it's time for the next frame
        while (m_clock.getElapsedTime().asSeconds() < m_dT) {} // wait until m_dT is elapsed
        // Process frame
        handleEvents();
        update();
        // Restart clock
        m_clock.restart();
    }
}

/**
 * @brief Add a new body to the simulation.
 *
 * @param i_collisionPartner A pointer to the object that needs to be added.
 */
void Simulation::addCollisionPartner(RigidBody * i_collisionPartner) {
    m_collisionPartners.push_back(i_collisionPartner);
    m_players.push_back(i_collisionPartner);
    i_collisionPartner->setOutlineColor(sf::Color::Red);
    i_collisionPartner->setFillColor(sf::Color::Black);
    i_collisionPartner->setOutlineThickness(-2.0f);
}

/**
 * @brief Add a new player to the simulation.
 *
 * @param i_player A pointer to the player that needs to be added.
 */
void Simulation::addPlayer(RigidBody * i_player) {
    m_players.push_back(i_player);
    addCollisionPartner(i_player);
}

/**
 * @brief Removes a body from the simulation.
 *
 * @param i_index The index of the body that needs to be deleted.
 */
void Simulation::deleteCollisionPartner(int i_index) {
    // Retrieve body pointer
    RigidBody * colParToDelete = m_collisionPartners[i_index];
    // Delete body
    delete colParToDelete;
    // Remove pointer from vector
    m_collisionPartners.erase(m_collisionPartners.begin() + i_index);
}

/**
 * @brief Removes a body from the simulation.
 *
 * @param i_bodyToDelete Pointer to the body that needs to be deleted.
 */
void Simulation::deleteCollisionPartner(RigidBody * i_bodyToDelete) {
    for (int i = 0; i < m_collisionPartners.size(); i++) {
        // First check if the body is even part of the simulation
        if (m_collisionPartners[i] == i_bodyToDelete) {
            // Delete body
            delete m_collisionPartners[i];
            // Remove pointer from vector
            m_collisionPartners.erase(m_collisionPartners.begin() + i);
        }
    }
}

/**
 * @brief Opens the simulation window and defines basic settings.
 *
 * @param i_viewWidth The desired width of the window in pixels.
 *
 * @param i_viewHeight The desired height of the window in pixels.
 *
 * @param i_frameRate The desired frame rate in Hz.
 */
void Simulation::initWindow(float i_viewWidth, float i_viewHeight, float i_frameRate) {
    // Simulation needs to have at least one pixel
    if (i_viewWidth < 1) {
        i_viewWidth = DEFAULT_VIEW_WIDTH;
    }
    if (i_viewHeight < 1) {
        i_viewHeight = DEFAULT_VIEW_HEIGHT;
    }
    // Frame rate needs to be positive
    if (i_frameRate <= 0) {
        i_frameRate = DEFAULT_FRAME_RATE;
    }
    // Open window
    m_window.create(sf::VideoMode(i_viewWidth, 512), "SFML 2D collision Simulation",
            sf::Style::Close | sf::Style::Titlebar | sf::Style::Resize);
    // Initialize view and set it to the center of the window
    m_view = sf::View(sf::Vector2f(i_viewWidth / 2, i_viewHeight / 2), sf::Vector2f(i_viewWidth, i_viewHeight));
    // Set time increment
    m_dT = 1 / i_frameRate;
}

/**
 * @brief Sets the collision geometry indicators properties.
 */
void Simulation::initCollisionMarkers() {

    m_collisionLocationMarker->setOrigin({5.0f, 5.0f});
    m_collisionLocationMarker->setOutlineColor(sf::Color::Red);
    m_collisionLocationMarker->setFillColor(sf::Color::Black);
    m_collisionLocationMarker->setOutlineThickness(-2.0f);

    for (sf::RectangleShape * marker : m_collisionNormalMarkers) {
        marker->setOutlineColor(sf::Color::Red);
        marker->setFillColor(sf::Color::Black);
        marker->setOutlineThickness(-1.0f);
    }
}

/**
 * @brief Add a new boundary element to the simulation.
 *
 * @param i_player A pointer to the boundary element that needs to be added.
 */
void Simulation::addBoundaryElement(BoundaryElement * i_boundaryElement) {
    // addCollisionPartner(static_cast<RigidBody *>(i_boundaryElement));
    m_boundaryElements.push_back(i_boundaryElement);
}

/**
 * @brief Is called every frame. Calls methods to check for collisions and resolve them. Updates body positions.
 *
 * This method generates CollisionEvents for all possible combinations of bodies. If a collision is actually detected, it is resolved. If
 * the window has been resized, the view is updated accordingly. The change in position and rotation is applied and displayed for all
 * bodies.
 *
 * @param i_player A pointer to the boundary element that needs to be added.
 */
void Simulation::update() {
    m_window.setView(m_view); // update view
    m_window.clear();         // remove old Objects
    // Iterate through all bodies
    for (int i = 0; i < m_collisionPartners.size(); i++) {
        // Iterate through all potential partners (only lower indices are used to avoid the same collision being processed twice)
        for (int j = 0; j < i; j++) {
            CollisionEvent collEvent = m_cd.generateCollisionEvent(m_collisionPartners[j], m_collisionPartners[i]);
            evaluateCollisionEvent(collEvent);
        }
        // Iterate through all boundaryElements
        for (int j = 0; j < m_boundaryElements.size(); j++) {
            CollisionEvent collEvent = m_cd.generateCollisionEvent(m_boundaryElements[j], m_collisionPartners[i]);
            evaluateCollisionEvent(collEvent);
        }
        // Update and display bodies
        m_collisionPartners[i]->updatePositionAndAngle(m_dT);
        m_window.draw(*m_collisionPartners[i]);
        
    }

    // Update and display collision geometry markers
    m_window.draw(*m_collisionLocationMarker);

    for (sf::RectangleShape * marker : m_collisionNormalMarkers) {
        m_window.draw(*marker);
    }

    // Update and display boundaryElements
    for (BoundaryElement * element : m_boundaryElements) {
        sf::Vertex * vertexArray = element->getVertexArray();
        // Copy the content into a local array
        sf::Vertex localVertexArray[2];
        localVertexArray[0] = vertexArray[0];
        localVertexArray[1] = vertexArray[1];
        m_window.draw(localVertexArray, 2, sf::Lines);
    }

    // Render the frame
    m_window.display();
}

/**
 * @brief Checks if the CollisionEvent actually indicates a collision. Calls the resolve() method, Updates position and angle of the
 * collision geometry markers.
 *
 * @param i_collisionEvent The CollisionEvent to evaluate.
 */
void Simulation::evaluateCollisionEvent(CollisionEvent & i_collisionEvent) {
    if (i_collisionEvent.getMinSeparation() < 0) {
        collisionGeometry_type collisionGeometry = i_collisionEvent.getCollisionGeometry();
        m_collisionLocationMarker->setPosition(collisionGeometry.location);
        m_collisionNormalMarkers[0]->setPosition(collisionGeometry.location);
        m_collisionNormalMarkers[1]->setPosition(collisionGeometry.location);
        m_collisionNormalMarkers[0]->setRotation(sfu::getVectorDirection(collisionGeometry.normals[0]));
        m_collisionNormalMarkers[1]->setRotation(sfu::getVectorDirection(collisionGeometry.normals[0]));
        i_collisionEvent.resolve();
    }
}

// TODO add proper control mechanism for players
void Simulation::handleEvents() {
    sf::Event event;
    while (m_window.pollEvent(event)) {
        sf::Vector2u newSize;

        switch (event.type) {
        case sf::Event::Closed:
            Simulation::~Simulation(); // destroy Simulation; close window
            break;
        case sf::Event::Resized: // change window size and adapt view
            // std::cout << "New window width: " << event.size.width << ", New window height: " << event.size.height << std::endl;
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
