#ifndef HYPERGRID_RAYTRACING_HPP
#define HYPERGRID_RAYTRACING_HPP

#include <arrayfire.h>

namespace hypergrid
{
    void add_lines(af::array& grid, int value,
                   size_t start_x, size_t start_y,
                   af::array endpoints);

    void add_line(af::array& grid, int value,
                   size_t start_x, size_t start_y,
                   size_t end_x, size_t end_y);

    /*
        DDA algorithm to draw line from one point to another in the occupancy grid.
    */
    void DDA(af::array& grid, int value,
             float start_x, float start_y,
             float end_x, float end_y);

} // hypergrid namespace

#endif
