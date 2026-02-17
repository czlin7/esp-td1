#pragma once

class Vec2f {
private:
  float m_x, m_y;

public:
  Vec2f(float x, float y) : m_x(x), m_y(y) {}

  float getX() const { return m_x; }
  float getY() const { return m_y; }
  float &getX() { return m_x; }
  float &getY() { return m_y; }

  Vec2f &operator=(const Vec2f &rhs) { // operator overload on =
    m_x = rhs.m_x;
    m_y = rhs.m_y;
    return *this;
  }

}; // Vec2f class
