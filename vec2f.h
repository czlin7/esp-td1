#pragma once

/**
 * @file vec2f.h
 * @brief Definition of the Vec2f class representing a 2D float vector.
 * @author czlin7
 * @date 2026
 */

/**
 * @class Vec2f
 * @brief Represents a 2-dimensional vector using floating-point values.
 *
 * Provides accessors and assignment operator overload.
 */
class Vec2f {
private:
  float m_x; ///< X component of the vector
  float m_y; ///< Y component of the vector

public:
  /**
   * @brief Constructs a 2D vector with given x and y values.
   * @param x Initial x component.
   * @param y Initial y component.
   */
  Vec2f(float x, float y) : m_x(x), m_y(y) {}

  /**
   * @brief Returns the x component (read-only).
   * @return The x value.
   */
  float getX() const { return m_x; }

  /**
   * @brief Returns the y component (read-only).
   * @return The y value.
   */
  float getY() const { return m_y; }

  /**
   * @brief Returns a reference to the x component.
   * @return Reference to x value (modifiable).
   */
  float &getX() { return m_x; }

  /**
   * @brief Returns a reference to the y component.
   * @return Reference to y value (modifiable).
   */
  float &getY() { return m_y; }

  /**
   * @brief Assignment operator.
   * @param rhs The vector to assign from.
   * @return Reference to this vector after assignment.
   */
  Vec2f &operator=(const Vec2f &rhs) {
    m_x = rhs.m_x;
    m_y = rhs.m_y;
    return *this;
  }

}; // Vec2f class
