#ifndef __VERTEX_H__
#define __VERTEX_H__

#include "glm/glm.hpp"
#include <vector>

class Vertex {
public:
  Vertex()
    : _pos(glm::vec3(0.0f, 0.0f, 0.0f)), _color(glm::vec3(0.0f, 0.0f, 0.0f)),
      _force(glm::vec3(0.0f, 0.0f, 0.0f)), _vel(glm::vec3(0.0f, 0.0f, 0.0f)),
      _texture(glm::vec2(0.0f, 0.0f)), _radius(20.0f), _mass(1.0f) {};

  Vertex(glm::vec3 pos)
    : _pos(pos), _color(glm::vec3(0.0f, 0.0f, 0.0f)),
      _force(glm::vec3(0.0f, 0.0f, 0.0f)), _vel(glm::vec3(0.0f, 0.0f, 0.0f)),
      _texture(glm::vec2(0.0f, 0.0f)), _radius(20.0f), _mass(1.0f){};

  Vertex(glm::vec3 pos, glm::vec3 color, float radius, float mass)
    : _pos(pos), _color(color), _radius(radius), _mass(mass)
  {
    _texture = glm::vec2(0.0f, 0.0f);
    _force = glm::vec3(0.0f, 0.0f, 0.0f);
    _vel = glm::vec3(0.0f, 0.0f, 0.0f);
  };

  Vertex(const Vertex &src) {
    _pos = glm::vec3(src.pos());
    _color = glm::vec3(src.color());
    _force = glm::vec3(src.force());
    _vel = glm::vec3(src.vel());
    _texture = glm::vec2(src.texture());
    _radius = src.radius();
    _mass = src.mass();
  }

  glm::vec3 pos() const { return _pos; }
  glm::vec3 color() const { return _color; }
  glm::vec3 force() const { return _force; }
  glm::vec3 vel() const { return _vel; }
  glm::vec2 texture() const { return _texture; }

  void pos(float x, float y, float z) { _pos = glm::vec3(x, y, z); }
  void color(float r, float g, float b) { _color = glm::vec3(r, g, b); }
  void force(float x, float y, float z) { _force = glm::vec3(x, y, z); }
  void vel(float x, float y, float z) { _vel = glm::vec3(x, y, z); }
  void texture(float x, float y) { _texture = glm::vec2(x, y); }

  const float radius() const { return _radius; }
  const float mass() const { return _mass; }
  void radius(const float newR) { _radius = newR; }
  void mass(const float newM) { _mass = newM; }

  void update(
    float dt = 0.1,
    glm::vec3 force = glm::vec3(0.0f, 0.0f, 0.0f),
    glm::vec3 vel = glm::vec3(0.0f, 0.0f, 0.0f)
  ) {
    glm::vec3 acc;

    _force += force;
    acc = _force / _mass;

    _vel += (vel + acc * dt);
    _pos += _vel * dt;
  }

  unsigned int flat(float* result)
  {
    float tmp[] = {
      _pos.x, _pos.y, _pos.z,
      _color.x, _color.y, _color.z,
      _radius,
      _texture.x, _texture.y
    };

    memcpy(result, tmp, sizeof(tmp));
    
    return sizeof(tmp) / sizeof(float);
  }

private:
  glm::vec3 _pos;
  glm::vec3 _color;
  glm::vec3 _force;
  glm::vec3 _vel;
  glm::vec2 _texture;
  float _radius;
  float _mass;
};

#endif
