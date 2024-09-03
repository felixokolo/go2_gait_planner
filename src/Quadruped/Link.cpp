/**
 * @file Link.cpp
 * @brief This file contains the implementation of the Link class.
 *
 * The Link class represents a link in a quadruped robot's leg.
 * It stores the length of the link and provides methods to access this length.
 *
 * @date 2 Jun 2024
 * @authors Felix, Priyanka
 */

#include "Quadruped/Link.h"

/**
 * @brief Constructs a Link with the specified length.
 * 
 * @param length The length of the link.
 */
Link::Link(double length) : length(length) {
    // Constructor implementation
}

/**
 * @brief Destructor for the Link class.
 */
Link::~Link() {
    // Destructor implementation
}

/**
 * @brief Retrieves the length of the link.
 * 
 * @return The length of the link.
 */
double Link::getLength() const {
    return length;
}
