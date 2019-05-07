#ifndef HYPERGRID_LINE_ITERATOR_HPP
#define HYPERGRID_LINE_ITERATOR_HPP

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
    LineIterator(af::array grid, Cell start, Cell end, int connectivity = 4)
    {
        grid_ = grid;
        start_ = start;
        end_ = end;
        size_ = -1;

        size_t bt_pix0 = 1;
        size_t bt_pix = bt_pix0;
        size_t istep = grid_.dims(0);

        int dx = end_.x - start_.x;
        int dy = end_.y - start_.y;
        int s = dx < 0 ? -1 : 0;

        dx = (dx ^ s) - s;
        bt_pix = (bt_pix ^ s) - s;

        ptr_ = start_.y * istep + start_.x;

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
            assert( dx >= 0 && dy >= 0 );

            error_ = 0;
            plus_delta_ = (dx + dx) + (dy + dy);
            minus_delta_ = -(dy + dy);
            plus_step_ = (int)(istep - bt_pix);
            minus_step_ = (int)bt_pix;
            size_ = dx + dy + 1;
        }
        else // connectivity == 8
        {
            assert( dx >= 0 && dy >= 0 );

            error_ = dx - (dy + dy);
            plus_delta_ = dx + dx;
            minus_delta_ = -(dy + dy);
            plus_step_ = (int)istep;
            minus_step_ = (int)bt_pix;
            size_ = dx + 1;
        }
        current_pos_.y = (int)(ptr_ / grid_.dims(0));
        current_pos_.x = (int)((ptr_ - (current_pos_.y * grid_.dims(0))));

        // std::cout << "LineIterator created" << std::endl;
        // std::cout << "size_: " << size_ << std::endl;
        // std::cout << "start_: " << start_.str() << std::endl;
        // std::cout << "end_: " << end_.str() << std::endl;
        // std::cout << "ptr_: " << ptr_ << std::endl;
        // std::cout << "current_pos_: " << current_pos_.str() << std::endl;
        // std::cout << "minus_delta_: " << minus_delta_ << std::endl;
        // std::cout << "plus_delta_: " << plus_delta_ << std::endl;
        // std::cout << "minus_step_: " << minus_step_ << std::endl;
        // std::cout << "plus_step_: " << plus_step_ << std::endl;
        // std::cout << "error_: " << error_ << std::endl;
    }

    inline Cell operator ()() const {return current_pos_;};
    inline Cell operator *() const {return current_pos_;};
    inline size_t size() const {return size_;};

    inline LineIterator& operator ++()
    {
        int mask = error_ < 0 ? -1 : 0;

        error_ += minus_delta_ + (plus_delta_ & mask);
        ptr_ += minus_step_ + (plus_step_ & mask);

        current_pos_.y = (int)(ptr_ / grid_.dims(0));
        current_pos_.x = (int)((ptr_ - (current_pos_.y * grid_.dims(0))));

        return *this;
    }
    inline LineIterator operator ++(int)
    {
        LineIterator it = *this;
        ++(*this);
        return it;
    }

protected:
    af::array grid_;
    Cell current_pos_;
    Cell start_;
    Cell end_;
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
