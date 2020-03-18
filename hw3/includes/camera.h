#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/string_cast.hpp"

// Reference: camera code from learnopengl.com


namespace NSCamera {
  class Camera
  {
  public:
  Camera(glm::vec3 pos)
    : _pos(pos), _globalUp(glm::vec3(0.0f, 1.0f, 0.0f)), _speed(0.05f),
      _sensitivity(0.1f), _yaw(-90.0f), _pitch(0.0f), _zoom(45.0f) { updateVectors(); };

  Camera(glm::vec3 pos, glm::vec3 globalUp)
    : _pos(pos), _globalUp(globalUp), _speed(0.05f), _sensitivity(0.1f),
      _yaw(-90.0f), _pitch(0.0f), _zoom(45.0f) { updateVectors(); };

  Camera(glm::vec3 pos, glm::vec3 globalUp, float yaw, float pitch)
    : _pos(pos), _globalUp(globalUp), _speed(0.05f), _sensitivity(0.1f),
      _yaw(yaw), _pitch(pitch), _zoom(45.0f) { updateVectors(); };

    void moveForward(float dt) { _pos += _speed * dt * _front; };
    void moveBackward(float dt) { _pos -= _speed * dt * _front; };
    void moveLeft(float dt) { _pos -= _speed * dt * _right; };
    void moveRight(float dt) { _pos += _speed * dt * _right; };
    void zoomInOut(double zoom);
    void pitchAndYaw(double xOffset, double yOffset);
    glm::mat4 view() { return glm::lookAt(_pos, _pos + _front, _up); }

    // Getters & Setters
    const float speed() const { return _speed; };
    const float sensitivity() const { return _sensitivity; };
    const float yaw() const { return _yaw; };
    const float pitch() const { return _pitch; };
    const float zoom() const { return _zoom; };
    const glm::vec3& pos() const { return _pos; };
    const glm::vec3& globalUp() const { return _globalUp; };
    const glm::vec3& right() const { return _right; };
    const glm::vec3& up() const { return _up; };
    const glm::vec3& front() const { return _front; };

    void speed(float s) { _speed = s; };
    void sensitivity(float s) { _sensitivity = s; };
    void zoom(float z) { _zoom = z; };
    void pos(glm::vec3 v) { _pos = v; };
    void globalUp(glm::vec3 v) { _globalUp = v; };

  private:
    void updateVectors();

    float _speed;
    float _sensitivity;
    float _yaw;
    float _pitch;
    float _zoom;

    glm::vec3 _pos;
    glm::vec3 _globalUp;
    glm::vec3 _front;
    glm::vec3 _up;
    glm::vec3 _right;
  };
}

#endif
