/**
 * @file Link.h
 * @brief This file contains the definition of the Link class.
 *
 * The Link class represents a link in a quadruped robot's leg.
 * It stores the length of the link and provides methods to access this length.
 *
 * @date: 2 Jun 2024
 * @author: Priyanka
 * @checked: Felix
 */

#ifndef LINK_H
#define LINK_H

/**
 * @class Link
 * @brief Represents a link in a quadruped robot's leg.
 *
 * The Link class stores the length of a link and provides a method to retrieve it.
 */
class Link
{
public:
    /**
     * @brief Constructs a Link with the specified length.
     *
     * @param length The length of the link.
     */
    Link(double length);

    /**
     * @brief Destructor for the Link class.
     */
    ~Link();

    /**
     * @brief Retrieves the length of the link.
     *
     * @return The length of the link.
     */
    double getLength() const;

private:
    double length; /**< The length of the link. */
};

#endif // LINK_H
