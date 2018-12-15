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
 *
 * The above copyright notice and this permission notice shall
 * be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED ''AS IS'', WITHOUT WARRANTY OF ANY KIND,
 *
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "../include/nodes.hpp"

nodes::nodes(void) : x_(0), y_(0), g_cost_(0), f_cost_(0) {
    std::cout << "Default Constructor Called" << std::endl;
}

nodes::nodes(int x, int y, double g_cost, double total_cost)
    : x_(x), y_(y), g_cost_(g_cost), f_cost_(total_cost) {}

auto nodes::compute_h(int xgoal, int ygoal) -> double {
    return sqrt((x_ - xgoal) * (x_ - xgoal) + (y_ - ygoal) * (y_ - ygoal));
}

auto nodes::compute_g(int k) -> double {
    return g_cost_ += (k % 2 == 0 ? 1 : 1.41);
}

auto nodes::compute_f(int x1, int y1) -> double {
    return f_cost_ = g_cost_ + compute_h(x1, y1);
}

nodes::~nodes() {}
