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
#include "turtlebot_rrt/turtlebot_rrt.hpp"
#include <gtest/gtest.h>



TEST(RRTPlanner, addVertex) {
    turtlebot_rrt::RRTPlanner* rrt = new turtlebot_rrt::RRTPlanner();
    turtlebot_rrt::Vertex v(1.0, 2.0, 0, -1);
    rrt->addVertex(v);
    std::vector<turtlebot_rrt::Vertex> tree = rrt->getVertexTree();
    EXPECT_EQ(tree.size(), 1);
}

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "rrt_test");
//     testing::InitGoogleTest(&argc, argv);

//     return RUN_ALL_TESTS();
// }

