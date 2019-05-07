#ifndef HYPERGRID_RAYTRACING_HPP
#define HYPERGRID_RAYTRACING_HPP

#include <arrayfire.h>
#include <hypergrid/line_iterator.hpp>

namespace hypergrid
{
    void add_lines(af::array& grid, int value,
                   size_t start_x, size_t start_y,
                   af::array endpoints);

} // hypergrid namespace

#endif