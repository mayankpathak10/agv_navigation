/**
 * MIT License
 *
 * Copyright (c) 2018 Mayank Pathak, Bhargav Dandamudi
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the software is furnished to do so,
 * subject to the following conditions:

 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
 * THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef INCLUDE_ASTAR_NAVIGATOR_HPP_
#define INCLUDE_ASTAR_NAVIGATOR_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>
#include "../include/map.hpp"
#include "../include/nodes.hpp"

class astar_navigator {
 public:
    int x_start_;
    int y_start_;
    int x_goal_;
    int y_goal_;

    /**
     * @brief astar_navigator class Constructor
     * @param none
     * @return none
     */
    astar_navigator();

    /**
     * @brief Parameterized astar_navigator Constructor
     *
     * @param x coordinate of the start node
     * @param y coordinate of the start node
     * @param x coordinate of the goal node
     * @param y coordinate of the goal node
     *
     * @return none
     */
    astar_navigator(int xStart, int yStart, int xGoal, int yGoal);

    /**
     *
     * @brief Get an optimal path from start to goal node.
     *
     * @param vec is a map which holds info if a node is walkable or blocked
     *
     * @return path with parent directions from start to goal
     */
    std::vector<std::pair<int, int> > astar_path(
        std::vector<std::vector<int> > map);

    /**
     *
     * @brief overloads the '>' operator
     *
     * Gives priority to node with greater total cost
     * to rearrange the priority Queue.
     *
     * @param p is a reference of a node
     * @param q is a reference of a node
     *
     * @return true if the total cost (f cost) of p is greater, else false
     *
     */
    friend bool operator>(const nodes& node1, const nodes& node2);
};
#endif   // INCLUDE_ASTAR_HPP_
