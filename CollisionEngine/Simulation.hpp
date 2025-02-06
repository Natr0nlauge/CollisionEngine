#pragma once
#include "sfml/Graphics.hpp"
#include "EdgeStructure.hpp"
#include "CollisionDetector.hpp"
#include "BoundaryElement.hpp"
#include "PlayerController.hpp"
#include <vector>
#include <mutex>
#include <memory>

class Simulation {
  public:
    // Singleton accessor
    static Simulation & getInstance();

    // Destructor
    ~Simulation();

    // Public methods
    void addBoundaryElement(BoundaryElement * i_boundaryElement);
    void initWindow(float i_viewWidth = DEFAULT_VIEW_WIDTH, float i_viewHeight = DEFAULT_VIEW_HEIGHT,
            float i_frameRate = DEFAULT_FRAME_RATE);
    void run();
    void addCollisionPartner(RigidBody * i_collisionPartner);
    void addPlayer(PlayerController * i_playerController);
    void deleteCollisionPartner(int i_index);
    void deleteCollisionPartner(RigidBody * i_bodyToDelete);

  private:
    // Singleton implementation
    static std::unique_ptr<Simulation> s_instance;
    static std::mutex mtx;
    Simulation();

    // Deleted copy constructor and assignment operator
    Simulation(const Simulation &) = delete;
    Simulation & operator=(const Simulation &) = delete;

    // Private methods
    void update();
    void evaluateCollisionEvent(CollisionEvent & i_collisionEvent);
    void handleEvents();
    void initCollisionMarkers();
    template <typename T> void cleanupMember(std::vector<T*>& member);

    // Member variables
    sf::Clock m_clock;
    std::vector<RigidBody *> m_collisionPartners;
    std::vector<PlayerController *> m_players;
    sf::RectangleShape * m_collisionLocationMarker = new sf::RectangleShape({10.0f, 10.0f});
    sf::RectangleShape * m_collisionNormalMarkers[2] = {new sf::RectangleShape({50.0f, 0.0f}), new sf::RectangleShape({0.0f, 50.0f})};
    std::vector<BoundaryElement *> m_boundaryElements;
    CollisionDetector & m_cd = CollisionDetector::getInstance();
    sf::View m_view;
    float m_dT = 1/DEFAULT_FRAME_RATE; // time per frame in seconds
    sf::RenderWindow m_window;
    static constexpr float DEFAULT_VIEW_WIDTH = 512.0f;
    static constexpr float DEFAULT_VIEW_HEIGHT = 512.0f; // Window initial size
    static constexpr float DEFAULT_FRAME_RATE = 120.0f;
};


