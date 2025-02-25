#include "Simulation.hpp"
#include "CollisionDetector.hpp"
#include "iostream"
#include "stdlib.h"
#include "sfml_utility.hpp"

std::unique_ptr<Simulation> Simulation::s_instance = nullptr; // pointer to Singleton instance
std::mutex Simulation::mtx;

/**
 * @brief Retrieves the singleton instance of the class.
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

Simulation::Simulation() {}

Simulation::~Simulation() {
    // Clean up all the members to avoid memory leaks
    cleanupMember(m_collisionPartners);
    cleanupMember(m_players);

    // Properly close the render window
    if (m_window.isOpen()) {
        m_window.close();
    }
}

/**
 * @brief Cleans up and clears a vector of dynamically allocated objects.
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
 * This method is called to launch the simulation. It starts the clock and opens the window for displaying everything. While the Window is open,
 * the clock is used to handle the timing of the Simulation frames. The handleEvents() and update() methods are called every frame.
 */
void Simulation::run() {

    initCollisionMarkers();
    m_clock.restart();

    while (m_window.isOpen()) {
        // Wait until it's time for the next frame
        while (m_clock.getElapsedTime().asSeconds() < m_dT) {} // wait until m_dT is elapsed
        update();
        m_clock.restart();
        handleEvents(); // HandleEvents is last because it calls the destructor if the window is closed
    }
}

/**
 * @brief Add a new body to the simulation.
 *
 * The body is pushed back to m_collisionPartners and the appearance is set.
 *
 * @param i_collisionPartner A pointer to the object that needs to be added.
 */
void Simulation::addCollisionPartner(RigidBody * i_collisionPartner) {
    m_collisionPartners.push_back(i_collisionPartner);
    i_collisionPartner->setOutlineColor(sf::Color::Red);
    i_collisionPartner->setFillColor(sf::Color::Black);
    i_collisionPartner->setOutlineThickness(-2.0f);
}

/**
 * @brief Add a new player to the simulation.
 *
 * @param i_player A pointer to the player that needs to be added.
 */
void Simulation::addPlayer(PlayerController * i_playerController) {
    m_players.push_back(i_playerController);
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
 * Can be used to define initial size and frame rate of the window. The size can be changed later on by resizing the window.
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
 * @brief Sets the collision geometry indicators' properties.
 */
void Simulation::initCollisionMarkers() {

    m_collisionLocationMarker.setOrigin({5.0f, 5.0f});
    m_collisionLocationMarker.setOutlineColor(sf::Color::Red);
    m_collisionLocationMarker.setFillColor(sf::Color::Black);
    m_collisionLocationMarker.setOutlineThickness(-2.0f);

    for (sf::RectangleShape & marker : m_collisionNormalMarkers) {
        marker.setOutlineColor(sf::Color::Red);
        marker.setFillColor(sf::Color::Black);
        marker.setOutlineThickness(1.0f);
    }
}

/**
 * @brief Add a new boundary element to the simulation.
 *
 * @param i_player A pointer to the boundary element that needs to be added.
 */
void Simulation::addBoundaryElement(BoundaryElement * i_boundaryElement) {
    m_boundaryElements.push_back(i_boundaryElement);
}

/**
 * @brief Updates the window, bodies and collisions.
 *
 * This method is called every frame. Checks for collisions and resolves them by calling other methods. Updates body positions. Generates
 * CollisionEvents for all possible combinations of bodies. If a collision is detected, it is resolved. If the window has been resized, the
 * view is updated accordingly. The change in position and rotation is applied and displayed for all bodies.
 *
 * @param i_player A pointer to the boundary element that needs to be added.
 */
void Simulation::update() {
    m_window.setView(m_view); // update view
    m_window.clear();         // remove old Objects
    
    std::vector<RigidBody *> playerBodies;

    for (PlayerController * player : m_players) {
        playerBodies.push_back(player->getPlayerBody());
    }

    // Create a new vector to hold combined contents
    std::vector<RigidBody *> allBodies;
    allBodies.reserve(m_collisionPartners.size() + m_players.size()); // Reserve memory to improve performance

    // Copy the relevant content into allBodies
    allBodies.insert(allBodies.end(), m_collisionPartners.begin(), m_collisionPartners.end());
    allBodies.insert(allBodies.end(), playerBodies.begin(), playerBodies.end());
    // Iterate through all bodies
    for (int i = 0; i < allBodies.size(); i++) {
        // Iterate through all potential partners (only lower indices are used to avoid the same collision being processed twice)
        for (int j = 0; j < i; j++) {
            CollisionEvent collEvent = m_cd.generateCollisionEvent(allBodies[j], allBodies[i]);
            evaluateCollisionEvent(collEvent);
        }
        // Iterate through all boundaryElements
        for (int j = 0; j < m_boundaryElements.size(); j++) {
            CollisionEvent collEvent = m_cd.generateCollisionEvent(m_boundaryElements[j], allBodies[i]);
            evaluateCollisionEvent(collEvent);
        }
    }

    // Render the frame
    updateAndDrawBodies();
    m_window.display();
}

/**
 * @brief Calls the update method for all simulated RigidBodies and draws them into the new frame.
 */
void Simulation::updateAndDrawBodies() {
    for (RigidBody * body : m_collisionPartners) {
        body->updateBody(m_dT);
        m_window.draw(*body);
    }

    for (PlayerController * player : m_players) {
        player->update(m_dT);
        m_window.draw(*(player->getPlayerBody()));
    }

    // Update and display collision geometry markers
    m_window.draw(m_collisionLocationMarker);

    for (sf::RectangleShape & marker : m_collisionNormalMarkers) {
        m_window.draw(marker);
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
}

/**
 * @brief Checks if the CollisionEvent actually indicates a collision. Calls the resolve() method, updates position and angle of the
 * collision geometry markers.
 *
 * @param i_collisionEvent The CollisionEvent to evaluate.
 */
void Simulation::evaluateCollisionEvent(CollisionEvent & i_collisionEvent) {
    if (i_collisionEvent.getMinSeparation() <= 0) {
        collisionGeometry_type collisionGeometry = i_collisionEvent.getCollisionGeometry();
        m_collisionLocationMarker.setPosition(collisionGeometry.location);
        m_collisionNormalMarkers[0].setPosition(collisionGeometry.location);
        m_collisionNormalMarkers[1].setPosition(collisionGeometry.location);
        m_collisionNormalMarkers[0].setRotation(sfu::getVectorDirection(collisionGeometry.normals[0]));
        m_collisionNormalMarkers[1].setRotation(sfu::getVectorDirection(collisionGeometry.normals[0]));
        i_collisionEvent.resolve(); 
    }
}

/**
 * @brief Handle resizing and closing of the window by the user.
 */
void Simulation::handleEvents() {
    sf::Event event;
    while (m_window.pollEvent(event)) {
        sf::Vector2u newSize;

        switch (event.type) {
        case sf::Event::Closed:
            Simulation::~Simulation(); // destroy Simulation; close window
            break;
        case sf::Event::Resized: // change window size and adapt view
            m_view.setSize(m_window.getSize().x, m_window.getSize().y);                 // adapt view size
            m_view.setCenter(m_window.getSize().x / 2.0f, m_window.getSize().y / 2.0f); // adapt view center
            break;
        }

        // mouse control
        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            sf::Vector2i mousePos = sf::Mouse::getPosition(m_window);
            m_players[0]->getPlayerBody()->setPosition(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
        }
        const float ROTATION_PER_TIMESTEP = 0.05f;
        // Rotation control
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
            float oldRotation = m_players[0]->getPlayerBody()->getRotation();
            float newRotation = oldRotation - ROTATION_PER_TIMESTEP;
            m_players[0]->getPlayerBody()->setRotation(newRotation);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::E)) {
            float oldRotation = m_players[0]->getPlayerBody()->getRotation();
            float newRotation = oldRotation + ROTATION_PER_TIMESTEP;
            m_players[0]->getPlayerBody()->setRotation(newRotation);
        }
    }
}
