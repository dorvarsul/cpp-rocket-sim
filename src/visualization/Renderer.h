#pragma once
#include "../core/StateVector.h"
#include <Eigen/Dense>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>

class Renderer {
public:
  Renderer();
  ~Renderer();

  void init();
  void render(const Eigen::Matrix4f &view, const Eigen::Matrix4f &proj,
              const std::vector<std::vector<StateVector>> &trajectories);

private:
  GLuint m_shaderProgram;
  GLuint m_vao, m_vbo;

  // Helper to compile shaders
  GLuint createShader(const char *vertexSource, const char *fragmentSource);

  // Grid rendering
  void renderGrid();
  GLuint m_gridVao, m_gridVbo;
  std::vector<float> m_gridVertices;
};
