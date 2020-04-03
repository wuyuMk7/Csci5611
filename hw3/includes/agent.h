#ifndef __SHAPE_H__
#define __SHAPE_H__

#include "glm/gtx/string_cast.hpp"
#include "vertex.h"
#include <iostream>
#include <queue>
#include <random>
#include <ctime>
#include <unordered_map>
#include <unordered_set>

namespace NSShape
{
  class Shape
  {
  public:
    Shape(): _vertices(std::vector<Vertex>(0)) {};

    unsigned int vertexSize() { return _vertices.size(); };
    unsigned int updateVertices(std::vector<std::pair<int, Vertex>>& vertices) {
      unsigned int size = 0;
      for (unsigned int i = 0;i < vertices.size(); ++i) {
        if (vertices[i].first < _vertices.size()) {
          _vertices[vertices[i].first] = vertices[i].second;
          ++size;
        }
      }
      return size;
    };
    const std::vector<Vertex> &vertices() const { return this->_vertices; };
    std::vector<Vertex> &vertices() { return this->_vertices; };

    virtual unsigned int renderSize() = 0;
    virtual unsigned int renderPoints(std::vector<Vertex>& points) = 0;
    virtual unsigned int boundingBox(std::vector<Vertex>& points) = 0;

  protected:
    std::vector<Vertex> _vertices;
  };

  class Circle: public Shape
  {
  public:
    Circle(): _count(360), _radius(1.0f), _centerBackUp(glm::vec3(0.0f, 0.0f, 0.0f))
      { _vertices.push_back(glm::vec3(0.0f, 0.0f, 0.0f)); };
    Circle(Vertex vertex, float radius)
      :_count(360), _radius(radius), _centerBackUp(vertex) { _vertices.push_back(vertex); };
    Circle(Vertex vertex, float radius, unsigned int count)
      : _count(count), _radius(radius), _centerBackUp(vertex) { _vertices.push_back(vertex); };

    unsigned int renderSize() { return 2 * _count; };
    unsigned int renderPoints(std::vector<Vertex>& points);
    unsigned int boundingBox(std::vector<Vertex> &points);

    const unsigned int count() const { return _count; };
    const float radius() const { return _radius; };
    const Vertex center() const { return _vertices.front(); };

    void count(unsigned int newC) { _count = newC; };
    void radius(float newR) { _radius = newR; };
    void center(Vertex newC) {
      _vertices[0].pos(newC.pos().x, newC.pos().y, newC.pos().z);
    };
    void update(float dt, glm::vec3 force, glm::vec3 vel)
    {
      _vertices[0].force(0.0f, 0.0f, 0.0f);
      _vertices[0].update(dt, force, vel);
    };
    void restore() {
      _vertices.clear();
      _vertices.push_back(_centerBackUp);
    }
  private:
    unsigned int _count;
    float _radius;
    Vertex _centerBackUp;
  };


  class Polygon : public Shape
  {
  public:
    Polygon() {};
    Polygon(std::vector<Vertex>& vertices) {
      for (auto vertex: vertices)
        _vertices.push_back(vertex);
    };

    template <typename T, typename... Ts>
    Polygon(T vertex, Ts... vertices){ addVertex(vertex, vertices...); };

    void addVertex(Vertex v) { _vertices.push_back(v); };

    template<typename T, typename... Ts>
    void addVertex(T v1, Ts... vertices)
    {
      _vertices.push_back(v1);
      addVertex(vertices...);
    }

    unsigned int renderSize() { return 2 * _vertices.size(); };
    virtual unsigned int renderPoints(std::vector<Vertex> &points);
    virtual unsigned int boundingBox(std::vector<Vertex> &points);
  };

  class Rectangle : public Polygon {
  public:
    Rectangle(): Polygon(Vertex(), Vertex(), Vertex(), Vertex()) {}
    Rectangle(Vertex v1, Vertex v3): Polygon(v1, v1, v3, v3) { updateVertices(v1, v3); };
    Rectangle(Vertex v1, Vertex v2, Vertex v3, Vertex v4): Polygon(v1, v2, v3, v4) {};

    void updateVertices(Vertex v1, Vertex v3) {
      _vertices[0] = v1;
      _vertices[1] = v1;
      _vertices[1].pos(v3.pos().x, v1.pos().y, v1.pos().z);
      _vertices[2] = v3;
      _vertices[3] = v3;
      _vertices[3].pos(v1.pos().x, v3.pos().y, v1.pos().z);
    }
    void updateVertices(Vertex v1, Vertex v2, Vertex v3, Vertex v4) {
      _vertices[0] = v1;
      _vertices[1] = v2;
      _vertices[2] = v3;
      _vertices[3] = v4;
    };

    unsigned int boundingBox(std::vector<Vertex> &points) {
      for (auto vertex: _vertices)
        points.push_back(vertex);
      return 4;
    }
  };

  class Obstacle : public Circle {
  public:
    Obstacle() : Circle(){};
    Obstacle(Vertex vertex, float radius) : Circle(vertex, radius){};
    Obstacle(Vertex vertex, float radius, unsigned int count)
        : Circle(vertex, radius, count){};

    unsigned int draw(std::vector<Vertex> &points) {
      return this->renderPoints(points);
    };
  };

  class Agent : public Circle {
  public:
    Agent() : _path(std::vector<Vertex>{}), Circle(){};
    Agent(Vertex vertex, float radius) : _path(std::vector<Vertex>{}), Circle(vertex, radius){};
    Agent(Vertex vertex, float radius, unsigned int count)
      : _path(std::vector<Vertex>{}), Circle(vertex, radius, count){};

    unsigned int draw(std::vector<Vertex> &points) {
      return this->renderPoints(points);
    };
    const std::vector<Vertex> &path() const { return this->_path; };

    void updateByBoids(std::vector<Agent> &agents,
                       std::vector<Obstacle> &obs,
                       std::vector<std::vector<glm::vec3>>& nnEdges);
    void findPathByPRM(const std::vector<glm::vec3> &waypoints,
                       const std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                       const std::vector<Obstacle> &obs,
                       unsigned int target,
                       const std::string mode = "AStar");
    void findPathByRRT(std::vector<glm::vec3> &waypoints,
                       std::vector<std::vector<std::pair<unsigned int, float>>> &graph,
                       const std::vector<Obstacle> &obs,
                       const Rectangle &frame, glm::vec3 target,
                       float sampleProb, float stride,
                       unsigned int maxTimes, float maxDist,
                       bool rrtstar = false);

    bool pathObIntersection(glm::vec3 start, glm::vec3 end,
                            const std::vector<Obstacle> &obs);
    void smoothPath(const std::vector<Obstacle> &obs);
    void clearPath() { _path.clear(); };

  private:
    std::vector<Vertex> _path;
  };
}

#endif
