#include <hypergrid/raytracing.hpp>


namespace hypergrid
{

void add_lines(af::array& grid, int value,
               size_t start_x, size_t start_y,
               af::array endpoints)
{
    for (int i = 0; i < endpoints.dims(0); ++i)
    {
        DDA(grid, value, start_x, start_y,
            endpoints(i, 0).scalar<int>(),
            endpoints(i, 1).scalar<int>());
    }
}


} // hypergrid namespace
