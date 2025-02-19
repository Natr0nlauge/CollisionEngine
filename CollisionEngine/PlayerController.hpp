#pragma once
#include "RigidBody.hpp"

static const float FLOAT_MAX = std::numeric_limits<float>::max();
static const float FLOAT_LOWEST = std::numeric_limits<float>::lowest();
static const float DEFAULT_ACCEL = 100.0f;
static const int AMOUNT_OF_CONTROLS = 4;

/**
 * @class PlayerController
 *
 * @brief Enables the bodies in the simulation to be controlled by the user.
 *
 * This class holds a pointer to the RigidBody that is supposed to be controlled by the player. The same pointer is not included in
 * Simulation::m_collisionPartners. Also holds settings for controlling the body (acceleration, buttons for controls).
 *
 */
class PlayerController {

  public:
    // Constructor
    PlayerController(RigidBody * i_playerBody);

    // Destructor
    ~PlayerController();

    // Setters
    void setVerticalMovementWindow(float i_lowerYLimit, float i_upperYLimit, float i_lowerXLimit, float i_upperXLimit);
    void setMaximumSpeed(float i_maxSpeed);
    void setAcceleration(float i_accel);
    void setVerticalControls(sf::Keyboard::Key i_up, sf::Keyboard::Key i_down);
    void setHorizontalControls(sf::Keyboard::Key i_left, sf::Keyboard::Key i_right);

    // Getters
    RigidBody * getPlayerBody();

    // Public methods
    void update(float i_dT);

  private:
    // Private methods
    void adjustPlayerPosition();
    void handlePlayerEvents(float i_dT);

    // Private member variables
    /// Pointer to the body to be controlled
    RigidBody * m_playerBody;
    /// Holds lower and upper limit for vertical movement
    float m_verticalMovementWindow[2] = {FLOAT_LOWEST, FLOAT_MAX};
    /// Holds lower and upper limit for horizontal movement
    float m_horizontalMovementWindow[2] = {FLOAT_LOWEST, FLOAT_MAX};
    /// Maximum movement speed in pixels per second
    float m_maximumSpeed = FLOAT_MAX;
    /// The acceleration to be applied in pixels/second^2 when a control button is pressed
    float m_acceleration = DEFAULT_ACCEL;
    /// The inputs used for controlling the body (format: up, down, left, right)
    sf::Keyboard::Key controls[AMOUNT_OF_CONTROLS] = {sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::A,
            sf::Keyboard::Key::D}; 
};
