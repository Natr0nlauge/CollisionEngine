#pragma once
#include "sfml/Graphics.hpp"
#include "RigidBody.hpp"
#include "CollisionDetector.hpp"
#include "vector"
#include <mutex>
#include <memory>

class Simulation {
  public:
    // Singleton accessor
    static Simulation & getInstance();

    // Destructor
    ~Simulation();

    // Public methods
    void initBodies(std::vector<RigidBody *> i_rigidBodies);
    void initWindow(float i_viewWidth = DEFAULT_VIEW_WIDTH, float i_viewHeight = DEFAULT_VIEW_HEIGHT,
            float i_frameRate = DEFAULT_FRAME_RATE);
    void run();
    void addCollisionPartner(RigidBody * i_collisionPartner);
    void deleteCollisionPartner(int i_index);

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
    void handleEvents();
    template <typename T> void cleanupMember(std::vector<T*>& member);

    // Member variables
    sf::Clock m_clock;
    std::vector<RigidBody *> m_collisionPartners;
    std::vector<sf::RectangleShape *> m_pointMarkers;
    std::vector<sf::RectangleShape *> m_axisMarkers;
    CollisionDetector & m_cd = CollisionDetector::getInstance();
    sf::View m_view;
    float m_dT = 1/DEFAULT_FRAME_RATE; // in seconds
    sf::RenderWindow m_window;
    static constexpr float DEFAULT_VIEW_WIDTH = 512.0f;
    static constexpr float DEFAULT_VIEW_HEIGHT = 512.0f; // Window initial size
    static constexpr float DEFAULT_FRAME_RATE = 120.0f;
};


