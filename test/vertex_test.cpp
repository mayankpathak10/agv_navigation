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
 * @file vertexTest.cpp
 * @brief
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-16
 */

#include <gtest/gtest.h>
#include "../include/turtlebot_rrt/vertex.hpp"

TEST(vertex_test, SingleVertex) {
    turtlebot_rrt::Vertex vertex(5.0, 7.5, 3, 2);

    // test get location
    EXPECT_FLOAT_EQ(vertex.get_location().first, 5.0);
    EXPECT_FLOAT_EQ(vertex.get_location().second, 7.5);

    // test get index
    EXPECT_EQ(vertex.get_index(), 3);

    // test get_parent
    EXPECT_EQ(vertex.get_parent(), 2);

    // use setter methods and retest
    vertex.set_location(3.0, 6.23);
    vertex.set_index(12);
    vertex.set_parent(1);

    // test get location
    EXPECT_FLOAT_EQ(vertex.get_location().first, 3.0);
    EXPECT_FLOAT_EQ(vertex.get_location().second, 6.23);

    // test get index
    EXPECT_EQ(vertex.get_index(), 12);

    // test get_parent
    EXPECT_EQ(vertex.get_parent(), 1);
}
