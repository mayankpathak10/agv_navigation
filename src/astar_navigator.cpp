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

/**
 * @file astar_navigator.cpp
 * @brief
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-04
 */
#include "../include/astar_navigator.hpp"
#include "../include/explorer.hpp"

// Default constructor
astar_navigator::astar_navigator : (void)
    : x_start_(0), y_start_(0), x_goal_(1), y_goal_(1) {
    std::cout << "Default Constructor Called" << std::endl;
}

// Parameterized constructor
astar_navigator:: : astar_navigator
                    : (int xStart, int yStart, int xGoal, int yGoal)
    : x_start_(xStart), y_start_(yStart), x_goal_(xGoal), y_goal_(yGoal) {}

// Gives vector of x and y coordinates of optimal path given the full map.
std::vector<std::pair<int, int> >
    astar_navigator:: : astar_path(std::vector<std::vector<int> > map) {
    int neighbours = 8;   // max no of possible neighbors for any node

    std::vector<std::pair<int, int> > path;
    // Create a priority list of open nodes
    std::priority_queue<nodes, std::vector<nodes>, std::greater<nodes> >
        open_list;
    // Create a priority list to store temporary nodes
    std::priority_queue<nodes, std::vector<nodes>, std::greater<nodes> >
        temp_list;
    // Create 2D vectors to store open and visited nodes.
    std::vector<std::vector<int> > open(800, std::vector<int>(800, 0));
    std::vector<std::vector<int> > visited(800, std::vector<int>(800, 0));
    std::vector<std::vector<int> > parentDir(800, std::vector<int>(800, 0));

    // 8 possible moves in x and y direction
    std::vector<int> moveX{1, 1, 0, -1, -1, -1, 0, 1};
    std::vector<int> moveY{0, 1, 1, 1, 0, -1, -1, -1};

    nodes start(x_start_, y_start_, 0, 0);
    start.f_cost_ = start.compute_f(x_goal_, y_goal_);
    open_list.push(start);
    open[start.x_][start.y_] = start.f_cost_;

    /**This loop takes the first entry of the open list as the current node
     * since it has the lowest total cost function.*/

    while (!open_list.empty()) {
        nodes current = open_list.top();
        open_list.pop();
        open[current.x_][current.y_] = 0;
        visited[current.x_][current.y_] = 1;

        /** Trace the path and store it of the goal is found.*/

        if (current.x_ == x_goal_ && current.y_ == y_goal_) {
            int p = current.x_, q = current.y_;
            while (!(p == x_start_ && q == y_start_)) {
                int j = parentDir[p][q];
                path.push_back(std::make_pair(p, q));
                p += moveX[j];
                q += moveY[j];
            }
            return path;
        }

        /** Expand all the nodes for current node, calculate their
         * costs and store them in their variables.*/
        int dir = 0;
        while (dir < 8) {
            int dx = current.x_ + moveX[dir], dy = current.y_ + moveY[dir];
            /** For all the nodes lying outside the map, blocked
             * or is in the visisted list then simply ignor them.*/
            if (!(dx < 0 || dx > 799 || dy < 0 || dy > 799 ||
                  map[dx][dy] == 0 || visited[dx][dy] == 1)) {
                nodes child(dx, dy, current.g_cost_, current.f_cost_);
                child.g_cost_ = child.compute_g(dir);
                child.f_cost_ = child.compute_f(x_goal_, y_goal_);
                /* For new nodes, calculate their costs and store them.*/
                if (open[dx][dy] == 0) {
                    open[dx][dy] = child.f_cost_;
                    open_list.push(child);
                    parentDir[dx][dy] = (dir + neighbours / 2) % neighbours;
                    /* If the node exists in open list but has more
                    * promising total cost then it is replaced by
                    * the previous node and it's parent direction is updated.*/
                } else if (open[dx][dy] > child.f_cost_) {
                    open[dx][dy] = child.f_cost_;
                    parentDir[dx][dy] = (dir + 8 / 2) % 8;
                    while (!(open_list.top().x_ == dx &&
                             open_list.top().y_ == dy)) {
                        temp_list.push(open_list.top());
                        open_list.pop();
                    }
                    open_list.pop();
                    while (!temp_list.empty()) {
                        open_list.push(temp_list.top());
                        temp_list.pop();
                    }
                    open_list.push(child);
                }
            }
            ++dir;
        }
    }
    std::cout << "Path not found." << std::endl;
    path.push_back(std::make_pair(-1, -1));
    return path;
}

bool operator>(const nodes& node1, const nodes& node2) {
    return node1.f_cost_ > node2.f_cost_;
}
