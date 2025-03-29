#pragma once
#include "sfml/Graphics.hpp"
#include "VertexBasedBody.hpp"
#include "CollisionDetector.hpp"
#include "BoundaryElement.hpp"
#include "PlayerController.hpp"
#include <vector>
#include <array>
#include <mutex>
#include <memory>

/**
 * @class Simulation
 * @brief A singleton class that manages the physics simulation, rendering, and event handling.
 */
class Simulation {
  public:
    // Singleton accessor
    static Simulation & getInstance();

    // Destructor
    ~Simulation();

    // Public methods
    void addBoundaryElement(BoundaryElement * i_boundaryElement);
    void initWindow(unsigned int i_viewWidth = DEFAULT_VIEW_WIDTH, unsigned int i_viewHeight = DEFAULT_VIEW_HEIGHT,
            float i_frameRate = DEFAULT_FRAME_RATE);
    void run();
    void addCollisionPartner(RigidBody * i_collisionPartner);
    void addPlayer(PlayerController * i_playerController);
    void deleteCollisionPartner(int i_index);
    void deleteCollisionPartner(RigidBody * i_bodyToDelete);

    /// Choose if you want to show collision geometry indicators
    bool m_showCollisionMarkers = true;

  private:
    // Singleton implementation
    static std::unique_ptr<Simulation> s_instance;
    static std::mutex mtx;
    Simulation();

    // Delete copy constructor and assignment operator
    Simulation(const Simulation &) = delete;
    Simulation & operator=(const Simulation &) = delete;

    // Private methods
    void update();
    void updateAndDrawBodies();
    void evaluateCollisionEvent(CollisionEvent & i_collisionEvent);
    void handleEvents();
    void initCollisionMarkers();
    template <typename T> void cleanupMember(std::vector<T *> & member);

    // Member variables
    sf::Clock m_clock;
    /// Vector of pointers to the bodies managed by the Simulation (does not include player controlled bodies)
    std::vector<RigidBody *> m_bodiesToSimulate;
    std::vector<PlayerController *> m_players; ///< Vector of pointers to the players
    // These shapes serve to visualize collision geometry
    sf::RectangleShape m_collisionLocationMarker{sf::RectangleShape({10.0f, 10.0f})};
    std::array<sf::RectangleShape, 2> m_collisionNormalMarkers{sf::RectangleShape(sf::Vector2f(50.0f, 1.0f)),
            sf::RectangleShape(sf::Vector2f(1.0f, 50.0f))};
    /// BoundaryElements get their own vector to avoid (nonsensical) collisions between them
    std::vector<BoundaryElement *> m_boundaryElements;
    /// Collision Detector instance
    CollisionDetector & m_cd = CollisionDetector::getInstance();
    
    sf::View m_view;
    sf::RenderWindow m_window;
    float m_dT = 1 / DEFAULT_FRAME_RATE;                 ///< Time per frame in seconds.
    static constexpr unsigned int DEFAULT_VIEW_WIDTH = 512;  ///< In pixels.
    static constexpr unsigned int DEFAULT_VIEW_HEIGHT = 512; ///< In pixels.
    static constexpr float DEFAULT_FRAME_RATE = 120.0f;  ///< In Hz.
};
