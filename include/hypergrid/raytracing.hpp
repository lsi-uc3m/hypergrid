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
             float end_x, float end_y)
    {
        float dx = abs(end_x - start_x);
        float dy = abs(end_y - start_y);

        // float step = dx >= dy ? dx : dy;
        float step = ((int)(dx >= dy) * dx) + ((int)(dx < dy) * dy);

        dx = (end_x - start_x) / step;
        dy = (end_y - start_y) / step;

        af::array x_arr = af::constant(start_x, step);
        af::array y_arr = af::constant(start_y, step);
        af::array count = af::seq(0, step - 1);

        x_arr += dx * count;
        y_arr += dy * count;

        af::array indices = y_arr.as(s32) * grid.dims(0) + x_arr.as(s32);

        grid(indices) = value;
    }

} // hypergrid namespace

#endif