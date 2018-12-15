#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <SDL/SDL.h>
#include <iostream>
#include <utility>
#include <vector>

class map {
 public:
    /**
     * @brief creates a map woth obtacles and free space as a known
     * environment for path finding
     *
     * Points that can be traversed are 1 and which cannot be traversed
     * or that are obstacles are 0
     *
     * @param none
     *
     * @return 2D vector with map points
     */
    std::vector<std::vector<int> > create_map();

    auto print_path(std::vector<std::vector<int> > map,
                    std::vector<std::pair<int, int> > path) -> bool;
};

#endif   // INCLUDE_MAP_HPP_
