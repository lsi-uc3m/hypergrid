#ifndef HYPERGRID_POINT_HPP
#define HYPERGRID_POINT_HPP

#include <utility>

namespace hypergrid
{

template<typename T>
class Point
{
public:
    T x;
    T y;

    /* Empty constructor */
    Point()
    {
        this->x = 0.0;
        this->y = 0.0;
    }
    /* Copy constructor */
    Point(const Point& other_point)
    {
        this->x = other_point.x;
        this->y = other_point.y;
    }
    /* Move constructor */
    Point(Point&& other_point) noexcept
    {
        this->x = other_point.x;
        this->y = other_point.y;
    }
    /* Assignement operator */
    Point& operator = (const Point& other_point)
    {
        // check for self-assignment
        if(&other_point == this) return *this;

        this->x = other_point.x;
        this->y = other_point.y;
        return *this;
    }
    
    /* Conversions */
    std::pair<T, T> toStdPair()
    {
        return {this->x, this->y};
    }

    std::string str()
    {
        return "[" + std::to_string(this->x) + ", " + std::to_string(this->y) + "]";
    }

};

/* Point arithmetic operators */
template<typename T>
inline Point<T> operator+(Point<T> p1, Point<T> p2) {return Point<T>(p1.x + p2.x, p1.y + p2.y);}

template<typename T>
inline Point<T> operator-(Point<T> p1, Point<T> p2) {return Point<T>(p1.x - p2.x, p1.y - p2.y);}

template<typename T, typename S>
inline Point<T> operator+(Point<T> p, S scalar) {return Point<T>(p.x + scalar, p.y + scalar);}

template<typename T, typename S>
inline Point<T> operator+(S scalar, Point<T> p) {return (p + scalar);}

template<typename T, typename S>
inline Point<T> operator-(Point<T> p, S scalar) {return Point<T>(p.x - scalar, p.y - scalar);}

template<typename T, typename S>
inline Point<T> operator*(Point<T> p, S scalar) {return Point<T>(p.x * scalar, p.y * scalar);}

template<typename T, typename S>
inline Point<T> operator*(S scalar, Point<T> p) {return (p * scalar);}

template<typename T, typename S>
inline Point<T> operator/(Point<T> p, S scalar) {return (scalar == 0.0 ?
                                                            Point<T>(0,0) :
                                                            Point<T>(p.x / scalar, p.y / scalar));}

template<typename T, typename S>
inline Point<T> operator*=(Point<T> p, S scalar) {return p * scalar;}

template<typename T, typename S>
inline Point<T> operator/=(Point<T> p, S scalar) {return p / scalar;}


using Cell = Point<size_t>;
using Pointf = Point<float>;
using Pointd = Point<double>;

} // hypergrid namespace

#endif
