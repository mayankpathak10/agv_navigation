/* Copyright (C)
 * 2018 - Bhargav Dandamudi and Mayank Pathak
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
 */
/**
 * @file vertex.cpp
 * @brief
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-16
 */
#include "../include/agv_rrt/vertex.hpp"

namespace agv_rrt {
Vertex::Vertex(float x, float y, int index, int parent_index) {
    x_ = x;
    y_ = y;
    index_ = index;
    parent_index_ = parent_index;
}

void Vertex::set_coordinate(float x, float y) {
    x_ = x;
    y_ = y;
}

void Vertex::set_index(int index) { index_ = index; }

void Vertex::set_parent(int parent_index) { parent_index_ = parent_index; }

std::pair<float, float> Vertex::get_coordinate() {
    return std::pair<float, float>(x_, y_);
}

int Vertex::get_index() { return index_; }

int Vertex::get_parent() { return parent_index_; }
}   // namespace agv_rrt
