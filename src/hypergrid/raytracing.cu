#include <iostream>
#include <hypergrid/raytracing.hpp>
#include <af/cuda.h>


namespace hypergrid
{

// CUDA Kernel: 
__global__
void raytracing_kernel(int* grid, int value,
                       size_t start_x, size_t start_y,
                       int* end_x, int* end_y, 
                       size_t dim_x, size_t dim_y,
                       size_t endpoints_size,
                       int connectivity = 8)
{
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= endpoints_size) return;

    printf("blockIdx x: %i\t threadIdx x: %i\t idx: %i\n", blockIdx.x, threadIdx.x, idx);

    size_t bt_pix0 = 1;
    size_t bt_pix = bt_pix0;
    size_t istep = dim_x;

    int dx = end_x[idx] - (int)start_x;
    int dy = end_y[idx] - (int)start_y;
    int s = dx < 0 ? -1 : 0;

    dx = (dx ^ s) - s;
    bt_pix = (bt_pix ^ s) - s;

    size_t ptr_ = start_y * istep + start_x;

    s = dy < 0 ? -1 : 0;
    dy = (dy ^ s) - s;
    istep = (istep ^ s) - s;

    s = dy > dx ? -1 : 0;

    // conditional swaps
    dx ^= dy & s;
    dy ^= dx & s;
    dx ^= dy & s;

    bt_pix ^= istep & s;
    istep ^= bt_pix & s;
    bt_pix ^= istep & s;

    int minus_delta_ = 0;
    int plus_delta_ = 0;
    int minus_step_ = 0;
    int plus_step_ = 0;
    int error_ = 0;
    size_t size_ = 0;

    if (connectivity == 4)
    {
        error_ = 0;
        plus_delta_ = (dx + dx) + (dy + dy);
        minus_delta_ = -(dy + dy);
        plus_step_ = (int)(istep - bt_pix);
        minus_step_ = (int)bt_pix;
        size_ = dx + dy + 1;
    }
    else // connectivity == 8
    {
        error_ = dx - (dy + dy);
        plus_delta_ = dx + dx;
        minus_delta_ = -(dy + dy);
        plus_step_ = (int)istep;
        minus_step_ = (int)bt_pix;
        size_ = dx + 1;
    }
    size_t current_pos_y = (int)(ptr_ / dim_x);
    size_t current_pos_x = (int)((ptr_ - (current_pos_y * dim_x)));

    int mask = 0;

    printf("size_: %d\n", size_);
    for (int i = 0; i < size_; i++)
    {
        grid[ptr_] = value;

        mask = error_ < 0 ? -1 : 0;
        error_ += minus_delta_ + (plus_delta_ & mask);
        ptr_ += minus_step_ + (plus_step_ & mask);
        // printf("ptr_: %d", ptr_);
    }

}


void add_lines(af::array& grid, int value,
               size_t start_x, size_t start_y,
               af::array endpoints)
{
    // Ensure any JIT kernels have executed
    grid.eval();
    endpoints.eval();

    // Obtain device pointer from array object
    int *device_grid = grid.device<int>();
    int *device_end_x = endpoints(af::span, 0).device<int>();
    int *device_end_y = endpoints(af::span, 1).device<int>();

    // Determine ArrayFire's CUDA stream
    int af_id = af::getDevice();
    int cuda_id = afcu::getNativeId(af_id);
    cudaStream_t af_cuda_stream = afcu::getStream(cuda_id);

    // Set arguments and run the kernel in ArrayFire's stream
    int block_size = 512;
    int grid_size = (endpoints.dims(0) + block_size - 1) / block_size;
    std::cout << "grid_size: " << grid_size << std::endl;
    std::cout << "block_size: " << block_size << std::endl;
    raytracing_kernel<<<grid_size, block_size, 0, af_cuda_stream>>>(device_grid, value,
                                                                    start_x, start_y,
                                                                    device_end_x, device_end_y,
                                                                    grid.dims(0), grid.dims(1),
                                                                    endpoints.dims(0),
                                                                    4);

    // Finish any pending CUDA operations
    cudaDeviceSynchronize();

    // Return control of af::array memory to ArrayFire
    grid.unlock();
    grid.eval();
    std::cout << "LINESSSS" << std::endl;
    af_print(grid);
}


} // hypergrid namespace
