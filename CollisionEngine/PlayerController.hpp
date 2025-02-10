#pragma once
#include "RigidBody.hpp"

static const float FLOAT_MAX = std::numeric_limits<float>::max();
static const float FLOAT_LOWEST = std::numeric_limits<float>::lowest();
static const float DEFAULT_ACCEL = 100.0f;
static const int AMOUNT_OF_CONTROLS = 4;

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

    // Getters
    /*float * getVerticalMovementWindow();
    float * getHorizontalMovementWindow();
    float getMaximumVerticalVelocity();
    float getMaximumHorizontalVelocity();
    float getVerticalAcceleration();
    float getHorizontalAcceleration();*/

  private:
    // Private methods
    void adjustPlayerPosition();
    void handlePlayerEvents(float i_dT);

    // Private member variables
    RigidBody * m_playerBody;
    float m_verticalMovementWindow[2] = {FLOAT_LOWEST, FLOAT_MAX};
    float m_horizontalMovementWindow[2] = {FLOAT_LOWEST, FLOAT_MAX};
    float m_maximumSpeed = FLOAT_MAX;
    float m_acceleration = DEFAULT_ACCEL;
    sf::Keyboard::Key controls[AMOUNT_OF_CONTROLS] = {sf::Keyboard::Key::W, sf::Keyboard::Key::S, sf::Keyboard::Key::A,
            sf::Keyboard::Key::D}; // up, down, left, right

    // Hold pointer to body
    // Hold movement limits (position and velocity)
    // Hold accelerations (use default values)
    // Hold controls (use default values: No controls at all)
    // Call update function of body and check, Destroy itself when body doesn't exist anymore
    //
    // Decide for a variant:
    //
    // variant 1: (Let's do it)
    // Make players separate from the current m_collisionPartners vector in Simulation
    // players and m_collisionPartners get temporarily put into one array to facilitate collision geometry
    // updating happens independently, m_collisionPartners are updated directly, players indirectly
    // Vorteil: Position ist zu jedem Zeitpunkt korrekt
    //
    // Nachteil:
    // Viel komplexere Umsetzung
    // Pointer m�ssen alle kopiert werden
    //
    // variant 2:
    // players are part of m_collisionPartners. Both get updated together and PlayerController checks afterwards if the movement windows are
    // violated
    // Vorteile: Beim l�schen vom playerController existiert kann der K�rper einfach als RigidBody weiterexistieren (Bei variante 1 kann man
    // das aber �ber eine methode von Simulation l�sen)
    // Nachteil: position ist kurzzeitig falsch (kann Probleme verursachen) - "Behebung" �ber eigene Methode updatePlayer? (In gewisser
    // Weise w�re das Problem auch bei variante 1 vorhanden - das macht dann nicht so nen gro�en unterschied)
    //
    // method to assign controls to each body
    //
    // method to set velocity of the body according to controls
    //
    // getter for body pointer (setter may be useful, but won't be needed it for now)
};
