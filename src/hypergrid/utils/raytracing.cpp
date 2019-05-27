#include <hypergrid/utils/raytracing.hpp>
#include <iostream>

namespace hypergrid
{

/*
    DDA algorithm to draw line from one point to another in the occupancy grid.
*/
void DDA(af::array& grid, int value,
         float start_x, float start_y,
         float end_x, float end_y)
{
    float dx = std::abs<float>(end_x - start_x);
    float dy = std::abs<float>(end_y - start_y);

    // float step = dx >= dy ? dx : dy;
    float step = ((int)(dx >= dy) * dx) + ((int)(dx < dy) * dy);

    dx = (end_x - start_x) / step;
    dy = (end_y - start_y) / step;

    if(step!=0){
        af::array x_arr = af::constant(start_x, step);
        af::array y_arr = af::constant(start_y, step);
        af::array count = af::seq(0, step - 1);

        x_arr += dx * count;
        y_arr += dy * count;

        af::array indices = y_arr.as(s32) * grid.dims(0) + x_arr.as(s32);

        grid(indices) = value;
    }
}

void add_lines(af::array& grid, int value,
               size_t start_x, size_t start_y,
               af::array endpoints)
{
    // Obtain device pointer from array object
    int *device_end_x = endpoints(af::span, 0).device<int>();
    int *device_end_y = endpoints(af::span, 1).device<int>();
    

    // Remove duplicated endpoints
    size_t lut_size = grid.dims(0) * grid.dims(1);
    std::vector<bool> index_lut(lut_size, false);
    std::vector<int> unique_end_x;
    unique_end_x.reserve(endpoints.dims(0));
    std::vector<int> unique_end_y;
    unique_end_y.reserve(endpoints.dims(0));

    for (int i = 0; i < endpoints.dims(0); ++i)
    {
        int global_idx = device_end_x[i] * grid.dims(0) + device_end_y[i];
        if (global_idx >= lut_size) continue;
        if (!index_lut[global_idx])
        {
            unique_end_x.push_back(device_end_x[i]);
            unique_end_y.push_back(device_end_y[i]);
            index_lut[global_idx] = true;
        }
    }

    

    // Draw the free lines
    for (int i = 0; i < unique_end_x.size(); ++i)
    {
       
        DDA(grid, value, start_x, start_y,
            unique_end_x[i], unique_end_y[i]);
    }
    
}


} // hypergrid namespace
