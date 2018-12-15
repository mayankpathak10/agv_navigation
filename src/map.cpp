/* Copyright (C)
 * 2018 - Bhargav Dandamudi and Mayank Pathak
 *
 * MIT License
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
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include "../include/map.hpp"

std::vector<std::vector<int> > map::create_map() {
    // Initilaizing the map with 1s as a free space
    std::vector<std::vector<int> > map_vec(100, std::vector<int>(100, 1));
    int rectangle_center_x = 20;   // Rectangle center's x coordinate.
    int rectangle_center_y = 20;   // Rectangle center's y coordinate.
    int rectangle_span = 10;       // Span of Rectangle in all direction.

    for (int i = rectangle_center_x - rectangle_span;
         i < rectangle_center_x + rectangle_span; i++) {
        for (int j = rectangle_center_y - rectangle_span;
             j < rectangle_center_y + rectangle_span; j++) {
            map_vec[i][j] =
                0;   // updating the matrix elements to 0 if in obstacle.
        }
    }
    return map_vec;
}

bool map::print_path(std::vector<std::vector<int> > map,
                     std::vector<std::pair<int, int> > path) {
    if (path[0].first == -1) {
        return 0;
    }

    // Initialize the SDL library to create a window.
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window =
        SDL_CreateWindow("A* Algorithm", SDL_WINDOWPOS_CENTERED,
                         SDL_WINDOWPOS_CENTERED, 800, 800, SDL_WINDOW_SHOWN);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    // Set obstacle color to red
    for (auto i = 0; i <= 799; i++) {
        for (auto j = 0; j <= 799; j++) {
            if (map[i][j] == 0) {
                SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
                SDL_RenderDrawPoint(renderer, i, j);
            }
        }
    }

    // Set path colour to blue
    for (std::string::size_type i = 0; i < path.size(); i++) {
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        SDL_RenderDrawPoint(renderer, path[i].first, path[i].second);
    }

    // Update the screen
    SDL_RenderPresent(renderer);
    SDL_Delay(5000);

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
}
