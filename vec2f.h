#pragma once

/**
 * @file vec2f.h
 * @brief Lightweight two-dimensional vector of single-precision values.
 */

/**
 * @brief Stores and exposes two floating-point components.
 */
class Vec2f {
private:
    float m_x; /**< X component. */
    float m_y; /**< Y component. */

public:
    /**
     * @brief Constructs a vector with explicit component values.
     * @param x Initial x component.
     * @param y Initial y component.
     */
    Vec2f(float x, float y) : m_x(x), m_y(y) {}

    /**
     * @brief Returns the x component.
     * @return Current x value.
     */
    float getX() const { return m_x; }

    /**
     * @brief Returns the y component.
     * @return Current y value.
     */
    float getY() const { return m_y; }

    /**
     * @brief Returns a mutable reference to the x component.
     * @return Reference to x.
     */
    float &getX() { return m_x; }

    /**
     * @brief Returns a mutable reference to the y component.
     * @return Reference to y.
     */
    float &getY() { return m_y; }

    /**
     * @brief Assigns vector component values from another instance.
     * @param rhs Source vector.
     * @return Reference to this vector.
     */
    Vec2f &operator=(const Vec2f &rhs) {
        m_x = rhs.m_x;
        m_y = rhs.m_y;
        return *this;
    }
};
