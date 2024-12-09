#include <iostream>
#include <SFML/Graphics.hpp>
#include "Simulation.hpp"

int main() { 
	Simulation * simPointer = Simulation::getInstance();
	simPointer->run();
	
}