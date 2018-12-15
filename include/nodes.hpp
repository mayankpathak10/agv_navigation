#ifndef INCLUDE_NODES_HPP_
#define INCLUDE_NODES_HPP_

#include <cmath>
#include <iostream>

class nodes {
 public:
    /**
     * @brief nodes class Constructor
     * @param none
     * @return none
     */
    nodes();

    /**
     * @brief Parameterized nodes Constructor
     *
     * @param x coordinate of the node
     * @param y coordinate of the node
     * @param cost to come of that node
     * @param total cost of that node
     *
     * @return none
     */
    nodes(int x, int y, double g_cost, double total_cost);

    /**
     * @brief Calculates the Euclidian Distance between the current node and
     * goal node
     *
     * @param x coordinate of the goal
     * @param y coordinate of the goal
     *
     * @return the distance in double
     */
    auto compute_h(int xgoal, int ygoal) -> double;

    /**
     * @brief Calculates the path cost or cost to come of a node
     *
     * For diagonal movement cost is 1.41
     * For straight line movement cost is 1
     *
     * @param k is the direction of the movement
     *
     * @retun cost to come of that node from start node
     */
    auto compute_g(int k) -> double;

    /**
     * @brief Calculates total cost of the node (f cost)
     *
     * @param x1 is the x coordinate of the goal
     * @param y1 is the y coordinate of the goal
     *
     * @return total cost of the node
     */
    auto compute_f(int x1, int y1) -> double;

    /**
     * @brief Destructor of the nodes object
     */
    ~nodes();

    int x_;
    int y_;
    double g_cost_;
    double f_cost_;
};
#endif   // INCLUDE_NODES_HPP_
