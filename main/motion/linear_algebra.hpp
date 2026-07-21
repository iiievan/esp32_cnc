#pragma once

#include <cmath>
#include <optional>
#include <type_traits>

namespace MATH 
{

// Template constants (available in C++14 and later; for older standards, you can use regular floats)
template <typename T>
constexpr T DEG2RAD_v = static_cast<T>(0.017453292519943295);
template <typename T>
constexpr T RAD2DEG_v = static_cast<T>(57.29577951308232);

template <typename T>
struct Matrix2D;

/**
 * @brief A generic template class for a 2D vector.
 * @tparam T The data type of the coordinates (float, double, int, etc.)
 */
template <typename T = float>
struct Vector2D 
{
    T x = 0;
    T y = 0;


    constexpr Vector2D() = default;
    constexpr Vector2D(T x_val, T y_val) : x(x_val), y(y_val) {}
    constexpr explicit Vector2D(T scalar) : x(scalar), y(scalar) {}

    // Type conversion constructor (for example, from Vector2D<int> to Vector2D<float>)
    template <typename U>
    constexpr explicit Vector2D(const Vector2D<U>& other) 
        : x(static_cast<T>(other.x)), y(static_cast<T>(other.y)) {}


    static constexpr Vector2D xAxis() { return Vector2D(1, 0); }
    static constexpr Vector2D yAxis() { return Vector2D(0, 1); }
    static constexpr Vector2D zero()  { return Vector2D(0, 0); }

    /**
     * @brief Creates a unit vector based on the sensor's PGV angle (in degrees)
     * Available only for floating-point types (float, double)
     */
    static Vector2D fromPgvAngle(T degrees) 
    {
        static_assert(std::is_floating_point<T>::value, "fromPgvAngle требуется тип float или double");
        degrees = std::fmod(degrees, static_cast<T>(360));
        if (degrees < 0) degrees += static_cast<T>(360);

        T rad = (static_cast<T>(90) - degrees) * DEG2RAD_v<T>;
        return Vector2D(std::cos(rad), std::sin(rad));
    }

    /**
     * @brief Returns the target direction vector along one of the axes
     */
    static constexpr Vector2D targetAlongAxis(T pgv_angle) 
    {
         // Map to the range [0, 360]
        T angle = pgv_angle;
        if constexpr (std::is_floating_point<T>::value) 
        {
            angle = std::fmod(angle, static_cast<T>(360));
        } 
        else 
        {
            angle = angle % 360;
        }

        if (angle < 0) angle += static_cast<T>(360);

        if ((angle < 45 && angle >= 0) || (angle >= 315 && angle < 360)) 
        {
            return Vector2D(0, 1);
        } 
        else 
        if (angle >= 45 && angle < 135) 
        {
            return Vector2D(1, 0);
        } 
        else 
        if (angle >= 135 && angle < 225) 
        {
            return Vector2D(0, -1);
        } 
        else 
        {
            return Vector2D(-1, 0);
        }
    }


    Vector2D& operator+=(const Vector2D& other) 
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2D& operator-=(const Vector2D& other) 
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2D& operator*=(T scalar) 
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2D& operator/=(T scalar) 
    {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    constexpr T lengthSqr() const { return x * x + y * y; }

    auto length() const { return std::sqrt(lengthSqr()); }

    constexpr T dot(const Vector2D& other) const { return x * other.x + y * other.y; }

    auto cosAngleWith(const Vector2D& other) const 
    {
        auto len_product = length() * other.length();
        if (len_product < 1e-6) return static_cast<decltype(len_product)>(1);
        return dot(other) / len_product;
    }

    constexpr T cross(const Vector2D& other) const { return x * other.y - y * other.x; }

    auto angleRadTo(const Vector2D& other) const 
    {
        auto angle = std::atan2(static_cast<double>(other.x), static_cast<double>(other.y)) - 
                     std::atan2(static_cast<double>(x), static_cast<double>(y));
        if (angle < 0.0) angle += 2.0 * M_PI;
        return static_cast<T>(angle);
    }

    auto angleDegTo(const Vector2D& other) const 
    {
        auto angle_rad = std::atan2(static_cast<double>(cross(other)), static_cast<double>(dot(other)));
        if (angle_rad < 0.0) angle_rad += 2.0 * M_PI;
        return static_cast<T>(angle_rad * RAD2DEG_v<double>);
    }

    auto projectionOn(const Vector2D& b) const 
    {
        auto b_len = b.length();
        if (b_len < 1e-6) return static_cast<decltype(b_len)>(0);
        return dot(b) / b_len;
    }

    Vector2D& normalize() 
    {
        static_assert(std::is_floating_point<T>::value, "Нормализация доступна только для float/double");
        T len = length();
        if (len > static_cast<T>(1e-6)) {
            x /= len;
            y /= len;
        }
        return *this;
    }

    Vector2D normalized() const 
    {
        Vector2D copy = *this;
        return copy.normalize();
    }

    Vector2D& rotate(T theta_deg) 
    {
        static_assert(std::is_floating_point<T>::value, "Rotation is available only for float/double");

        if (std::abs(theta_deg) < static_cast<T>(1e-4)) return *this;
        
        T rad = theta_deg * DEG2RAD_v<T>;
        T c = std::cos(rad);
        T s = std::sin(rad);

        T old_x = x;
        x = old_x * c - y * s;
        y = old_x * s + y * c;

        return *this;
    }

    Vector2D rotated(T theta_deg) const 
    {
        Vector2D copy = *this;
        return copy.rotate(theta_deg);
    }

    constexpr bool isZero(T epsilon = static_cast<T>(0.001)) const 
    {
        if constexpr (std::is_floating_point<T>::value) 
        {
            return std::abs(x) < epsilon && std::abs(y) < epsilon;
        } 
        else 
        {
            return x == 0 && y == 0;
        }
    }
};

template <typename T> constexpr Vector2D<T> operator+(Vector2D<T> lhs, const Vector2D<T>& rhs) { return lhs += rhs; }
template <typename T> constexpr Vector2D<T> operator-(Vector2D<T> lhs, const Vector2D<T>& rhs) { return lhs -= rhs; }
template <typename T> constexpr Vector2D<T> operator*(Vector2D<T> lhs, T scalar) { return lhs *= scalar; }
template <typename T> constexpr Vector2D<T> operator*(T scalar, Vector2D<T> rhs) { return rhs *= scalar; }
template <typename T> constexpr Vector2D<T> operator/(Vector2D<T> lhs, T scalar) { return lhs /= scalar; }

template <typename T>
constexpr bool operator==(const Vector2D<T>& lhs, const Vector2D<T>& rhs) 
{
    if constexpr (std::is_floating_point<T>::value) 
    {
        return std::abs(lhs.x - rhs.x) < static_cast<T>(1e-5) && 
               std::abs(lhs.y - rhs.y) < static_cast<T>(1e-5);
    } 
    else 
    {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
}


/**
 * @brief Universal 2D Matrix Template Class
 */
template <typename T = float>
struct Matrix2D 
{
    T m00 = 1, m01 = 0;
    T m10 = 0, m11 = 1;

    constexpr Matrix2D() = default;
    constexpr Matrix2D(T m00, T m01, T m10, T m11)
        : m00(m00), m01(m01), m10(m10), m11(m11) {}

    static constexpr Matrix2D identity() 
    {
        return Matrix2D(1, 0, 0, 1);
    }

    static Matrix2D rotation(T theta_rad) 
    {
        static_assert(std::is_floating_point<T>::value, "The rotation matrix is available only for float/double");
        T c = std::cos(theta_rad);
        T s = std::sin(theta_rad);
        return Matrix2D(c, -s, s, c);
    }

    Matrix2D& operator+=(const Matrix2D& other) 
    {
        m00 += other.m00; m01 += other.m01;
        m10 += other.m10; m11 += other.m11;
        return *this;
    }

    Matrix2D& operator-=(const Matrix2D& other) 
    {
        m00 -= other.m00; m01 -= other.m01;
        m10 -= other.m10; m11 -= other.m11;
        return *this;
    }

    Matrix2D& operator*=(T scalar) 
    {
        m00 *= scalar; m01 *= scalar;
        m10 *= scalar; m11 *= scalar;
        return *this;
    }

    constexpr T det() const 
    {
        return m00 * m11 - m01 * m10;
    }

    std::optional<Matrix2D> inverse() const 
    {
        static_assert(std::is_floating_point<T>::value, "Обратная матрица доступна только для float/double");
        T d = det();
        if (std::abs(d) < static_cast<T>(1e-7)) return std::nullopt;

        T inv_det = static_cast<T>(1) / d;
        return Matrix2D(m11 * inv_det, -m01 * inv_det, -m10 * inv_det, m00 * inv_det);
    }

    bool isRotation(T epsilon = static_cast<T>(1e-5)) const 
    {
        static_assert(std::is_floating_point<T>::value, "isRotation доступна только для float/double");
        return std::abs(det() - static_cast<T>(1)) <= epsilon;
    }
};

template <typename T> constexpr Matrix2D<T> operator+(Matrix2D<T> lhs, const Matrix2D<T>& rhs) { return lhs += rhs; }
template <typename T> constexpr Matrix2D<T> operator-(Matrix2D<T> lhs, const Matrix2D<T>& rhs) { return lhs -= rhs; }
template <typename T> constexpr Matrix2D<T> operator*(Matrix2D<T> lhs, T scalar) { return lhs *= scalar; }
template <typename T> constexpr Matrix2D<T> operator*(T scalar, Matrix2D<T> rhs) { return rhs *= scalar; }


template <typename T>
constexpr Matrix2D<T> operator*(const Matrix2D<T>& a, const Matrix2D<T>& b) 
{
    return Matrix2D<T>(
        a.m00 * b.m00 + a.m01 * b.m10,  a.m00 * b.m01 + a.m01 * b.m11,
        a.m10 * b.m00 + a.m11 * b.m10,  a.m10 * b.m01 + a.m11 * b.m11
    );
}


template <typename T>
constexpr Vector2D<T> operator*(const Matrix2D<T>& m, const Vector2D<T>& v) 
{
    return Vector2D<T>(
        m.m00 * v.x + m.m01 * v.y,
        m.m10 * v.x + m.m11 * v.y
    );
}

using Vector2df = Vector2D<float>;
using Vector2dd = Vector2D<double>;
using Vector2di = Vector2D<int>;

using Matrix2df = Matrix2D<float>;
using Matrix2dd = Matrix2D<double>;

} // namespace MATH
