#ifndef HYPERGRID_LINE_ITERATOR_HPP
#define HYPERGRID_LINE_ITERATOR_HPP

#include <memory>
#include <arrayfire.h>
#include <hypergrid/point.hpp>

namespace hypergrid
{

class LineIterator
{
public:

    /* Smart Pointers typedefs */
    typedef std::shared_ptr<LineIterator> Ptr;
    typedef std::shared_ptr<const LineIterator> ConstPtr;

    /* Constructors */
    LineIterator(size_t grid_row_size, size_t start_x, size_t start_y,
                 size_t end_x, size_t end_y, int connectivity = 4)
    {
        grid_row_size_ = grid_row_size;

        size_t bt_pix0 = 1;
        size_t bt_pix = bt_pix0;
        size_t istep = grid_row_size_;

        int dx = (int)end_x - (int)start_x;
        int dy = (int)end_y - (int)start_y;
        int s = dx < 0 ? -1 : 0;

        dx = (dx ^ s) - s;
        bt_pix = (bt_pix ^ s) - s;

        ptr_ = start_y * istep + start_x;

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
        current_pos_.y = (int)(ptr_ / grid_row_size_);
        current_pos_.x = (int)((ptr_ - (current_pos_.y * grid_row_size_)));

        // std::cout << "LineIterator created" << std::endl;
        // std::cout << "size_: " << size_ << std::endl;
        // std::cout << "start: " << start_x << ", " << start_y << std::endl;
        // std::cout << "end: " << end_x << ", " << end_y << std::endl;
        // std::cout << "ptr_: " << ptr_ << std::endl;
        // std::cout << "current_pos_: " << current_pos_.str() << std::endl;
        // std::cout << "minus_delta_: " << minus_delta_ << std::endl;
        // std::cout << "plus_delta_: " << plus_delta_ << std::endl;
        // std::cout << "minus_step_: " << minus_step_ << std::endl;
        // std::cout << "plus_step_: " << plus_step_ << std::endl;
        // std::cout << "error_: " << error_ << std::endl;
    }

    LineIterator(size_t grid_row_size, Cell start, Cell end, int connectivity = 4) :
        LineIterator(grid_row_size, start.x, start.y, end.x, end.y, connectivity)
    {}

    inline Cell operator ()() const {return current_pos_;};
    inline Cell operator *() const {return current_pos_;};
    inline size_t size() const {return size_;};

    inline LineIterator& operator ++()
    {
        int mask = error_ < 0 ? -1 : 0;

        error_ += minus_delta_ + (plus_delta_ & mask);
        ptr_ += minus_step_ + (plus_step_ & mask);

        current_pos_.y = (int)(ptr_ / grid_row_size_);
        current_pos_.x = (int)((ptr_ - (current_pos_.y * grid_row_size_)));

        return *this;
    }
    inline LineIterator operator ++(int)
    {
        LineIterator it = *this;
        ++(*this);
        return it;
    }

protected:
    Cell current_pos_;
    size_t grid_row_size_;
    int minus_delta_;
    int plus_delta_;
    int minus_step_;
    int plus_step_;
    int error_;
    size_t ptr_;
    size_t size_;
};


} // hypergrid namespace

#endif
