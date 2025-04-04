#include "PlayerController.hpp"
#include "sfml_utility.hpp"

/**
 * @brief Constructor
 * @param i_playerBody A pointer to the body to be controlled.
 */
PlayerController::PlayerController(RigidBody * i_playerBody) : m_playerBody(i_playerBody) {
    i_playerBody->setOutlineColor(sf::Color::Red);
    i_playerBody->setFillColor(sf::Color::Black);
    i_playerBody->setOutlineThickness(-2.0f);
}

PlayerController::~PlayerController() {
    delete m_playerBody;
}

/**
 * @brief Sets the maximum and minimum x- and y- coordinates allowed for the player. (Default is unlimited movement.)
 * @param i_lowerYLimit Upper movement limit in pixels from origin. (Note y-axis is facing downwards)
 * @param i_upperYLimit Lower movement limit in pixels from origin. (Note y-axis is facing downwards)
 * @param i_lowerXLimit Left movement limit in pixels from origin.
 * @param i_upperXLimit Right movement limit in pixels from origin.
 */
void PlayerController::setMovementWindow(float i_lowerYLimit, float i_upperYLimit, float i_lowerXLimit, float i_upperXLimit) {
    m_verticalMovementWindow[0] = i_lowerYLimit;
    m_verticalMovementWindow[1] = i_upperYLimit;
    m_horizontalMovementWindow[0] = i_lowerXLimit;
    m_horizontalMovementWindow[1] = i_lowerXLimit;
}

/**
 * @brief Set maximum allowed speed for the player body. (Default is unlimited)
 * @param i_maxSpeed Max speed in pixels per second.
 */
void PlayerController::setMaximumSpeed(float i_maxSpeed) {
    m_maximumSpeed = i_maxSpeed;
}

/**
 * @brief Set acceleration for the player body.
 * @param i_accel Acceleration in pixels/s^2
 */
void PlayerController::setAcceleration(float i_accel) {
    m_acceleration = i_accel;
}

/**
 * @brief Specify the keys for vertical control (Default is W for up, S for down).
 * @param i_up Key for up movement.
 * @param i_down Key for down movement.
 */
void PlayerController::setVerticalControls(sf::Keyboard::Key i_up, sf::Keyboard::Key i_down) {
    controls[0] = i_up;
    controls[1] = i_down;
}

/**
 * @brief Specify the keys for horizontal control (Default is A for left, D for right).
 * @param i_left Key for left movement.
 * @param i_right Key for right movement.
 */
void PlayerController::setHorizontalControls(sf::Keyboard::Key i_left, sf::Keyboard::Key i_right) {
    controls[2] = i_left;
    controls[3] = i_right;
}

RigidBody * PlayerController::getPlayerBody() {
    return m_playerBody;
}

/**
 * @brief Makes sure the new player position is inside the limits.
 */
void PlayerController::adjustPlayerPosition() {
    sf::Vector2f playerPosition = m_playerBody->getPosition();

    if (playerPosition.x < m_horizontalMovementWindow[0]) {
        playerPosition.x = m_horizontalMovementWindow[0];
    } else if (playerPosition.x > m_horizontalMovementWindow[1]) {
        playerPosition.x = m_horizontalMovementWindow[1];
    }

    if (playerPosition.y < m_horizontalMovementWindow[0]) {
        playerPosition.y = m_horizontalMovementWindow[0];
    } else if (playerPosition.y > m_horizontalMovementWindow[1]) {
        playerPosition.y = m_horizontalMovementWindow[1];
    }
    m_playerBody->setPosition(playerPosition);
}

/**
 * @brief Process the user inputs.
 * @param i_dT Time increment in seconds.
 */
void PlayerController::handlePlayerEvents(float i_dT) {
    // keyboard control
    float velIncr = i_dT * m_acceleration;
    sf::Vector2f velocity = m_playerBody->getVelocity();
    if (sf::Keyboard::isKeyPressed(controls[0])) {
        velocity = sfu::addVectors(velocity, sf::Vector2f(0.0f, -velIncr));
    }
    if (sf::Keyboard::isKeyPressed(controls[1])) {
        velocity = sfu::addVectors(velocity, sf::Vector2f(0.0f, velIncr));
    }
    if (sf::Keyboard::isKeyPressed(controls[2])) {
        velocity = sfu::addVectors(velocity, sf::Vector2f(-velIncr, 0.0f));
    }
    if (sf::Keyboard::isKeyPressed(controls[3])) {
        velocity = sfu::addVectors(velocity, sf::Vector2f(velIncr, 0.0f));
    }
    float speed = sfu::getVectorLength(velocity);
    if (speed > m_maximumSpeed) {
        velocity = sfu::scaleVector(velocity, m_maximumSpeed / speed);
    }
    m_playerBody->setVelocity(velocity);
}

/**
 * @brief Update the body controlled by the player. Process the user inputs, call the RigidBody::update() method and make sure the body
 * position is inside the limits.
 * @param i_dT Time increment in seconds.
 */
void PlayerController::update(float i_dT) {
    handlePlayerEvents(i_dT);
    m_playerBody->updateBody(i_dT);
    adjustPlayerPosition();
}
