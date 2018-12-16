/*
 * @copyright (c) MIT License 2018 Bhargav Dandamudi, Mayank Pathak
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the 'Software'), to deal in the Software without
 * restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so,subject to
 * the following conditions:
 * The above copyright notice and this permission notice shall
 * be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED ''AS IS'', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @file vertex.hpp
 * @brief agv_navigation package
 *
 * @section DESCRIPTION
 *
 *  This file is a header file to declare all the class variables and
 *  functions that will be used for rrt algorithm expansion.
 *  Note that it uses the the same namespace as the 'RRTPlanner' class
 *
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-15
 */

#ifndef INCLUDE_AGV_RRT_VERTEX_HPP_
#define INCLUDE_AGV_RRT_VERTEX_HPP_

#include <cmath>
#include <utility>

namespace agv_rrt {
class Vertex {
  private:
    /**
     * @brief the x coordinate of the vertex
     */
    float x_;

    /**
     * @brief the y coordinate of the vertex
     */
    float y_;

    /**
     * @brief the index of the vertex
     */
    int index_;

    /**
     * @brief the vertex's parent index
     */
    int parent_index_;

  public:
    /**
     * @brief Simple Vertex constructor
     */
    Vertex() {}

    /**
     * @brief the constructor for a Vertex
     * @param x the x coordinate of the vertex
     * @param y the y coordinate of the vertex
     * @param index the index of the vertex
     * @param parent_index the index of the parent vertex
     */
    Vertex(float x, float y, int index, int parent_index);

    /**
     * @brief destructor
     */
    ~Vertex() {}

    /**
     * @brief sets the coordinate
     * @param x x coordinate of vertex
     * @param y y coordinate of vertex
     */
    void set_coordinate(float x, float y);

    /**
     * @brief sets the index of the vertex
     * @param index index of the vertex
     */
    void set_index(int index);

    /**
     * @brief sets the parent vertex
     * @param parent_index index of the parent vertex
     */
    void set_parent(int index);

    /**
     * @brief returns the x,y coordinate of the vertex
     * @return returns std::pair<x,y>
     */
    std::pair<float, float> get_coordinate();

    /**
     * @brief returns the index of the vertex
     * @return index of the vertex
     */
    int get_index();

    /**
     * @brief returns the index of the parent vertex
     * @return returns the index of the parent
     */
    int get_parent();

    /**
     * @brief overload of == operator
     */
    bool operator==(const Vertex& v) {
        return (x_ == v.x_ && y_ == v.y_ && parent_index_ == v.parent_index_);
    }

    /**
     * @brief overload of != operator
     */
    bool operator!=(const Vertex& v) {
        return (x_ != v.x_ || y_ != v.y_ || parent_index_ != v.parent_index_);
    }
};
}   // namespace agv_rrt

#endif   // INCLUDE_AGV_RRT_VERTEX_HPP_
