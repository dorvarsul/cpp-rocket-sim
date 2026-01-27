#include "Renderer.h"
#include <iostream>

const char *vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;
    
    uniform mat4 view;
    uniform mat4 projection;
    
    out vec3 Color;
    
    void main() {
        gl_Position = projection * view * vec4(aPos, 1.0);
        Color = aColor;
    }
)";

const char *fragmentShaderSource = R"(
    #version 330 core
    in vec3 Color;
    out vec4 FragColor;
    
    void main() {
        FragColor = vec4(Color, 1.0);
    }
)";

Renderer::Renderer()
    : m_shaderProgram(0), m_vao(0), m_vbo(0), m_gridVao(0), m_gridVbo(0) {}

Renderer::~Renderer() {
  glDeleteVertexArrays(1, &m_vao);
  glDeleteBuffers(1, &m_vbo);
  glDeleteVertexArrays(1, &m_gridVao);
  glDeleteBuffers(1, &m_gridVbo);
  glDeleteProgram(m_shaderProgram);
}

void Renderer::init() {
  m_shaderProgram = createShader(vertexShaderSource, fragmentShaderSource);

  // Setup Trajectory buffers
  glGenVertexArrays(1, &m_vao);
  glGenBuffers(1, &m_vbo);

  // Setup Grid buffers
  glGenVertexArrays(1, &m_gridVao);
  glGenBuffers(1, &m_gridVbo);

  // Create grid data (lines)
  // 1km x 1km grid
  int size = 2000;
  int step = 100;
  for (int i = -size; i <= size; i += step) {
    // Lines parallel to X (East)
    m_gridVertices.push_back((float)-size);
    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back(0.0f); // Pos
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f); // Color

    m_gridVertices.push_back((float)size);
    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back(0.0f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);

    // Lines parallel to Y (North)
    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back((float)-size);
    m_gridVertices.push_back(0.0f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);

    m_gridVertices.push_back((float)i);
    m_gridVertices.push_back((float)size);
    m_gridVertices.push_back(0.0f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
    m_gridVertices.push_back(0.3f);
  }

  // Add Axis lines
  // X (Red)
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(100);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);

  // Y (Green)
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(100);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);

  // Z (Blue)
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(100);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(0);
  m_gridVertices.push_back(1);

  glBindVertexArray(m_gridVao);
  glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
  glBufferData(GL_ARRAY_BUFFER, m_gridVertices.size() * sizeof(float),
               m_gridVertices.data(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
}

void Renderer::render(
    const Eigen::Matrix4f &view, const Eigen::Matrix4f &proj,
    const std::vector<std::vector<StateVector>> &trajectories) {
  // Check for pre-existing errors
  GLenum err;
  while ((err = glGetError()) != GL_NO_ERROR) {
    std::cerr << "OpenGL Error BEFORE render: " << err << std::endl;
  }

  if (m_shaderProgram == 0) {
    std::cerr << "Error: Shader Program is 0!" << std::endl;
    return;
  }

  glUseProgram(m_shaderProgram);

  // Reset State that ImGui or others might have changed
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);

  GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
  GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");

  if (viewLoc == -1 || projLoc == -1) {
    std::cerr
        << "Warning: Uniform 'view' or 'projection' not found (optimized out?)"
        << std::endl;
  }

  glUniformMatrix4fv(viewLoc, 1, GL_FALSE, view.data());
  glUniformMatrix4fv(projLoc, 1, GL_FALSE, proj.data());

  // Draw Grid
  glLineWidth(1.0f);
  glBindVertexArray(m_gridVao);
  glDrawArrays(GL_LINES, 0, m_gridVertices.size() / 6);

  // Draw Trajectories
  std::vector<float> trajVertices;
  for (const auto &history : trajectories) {
    if (history.empty())
      continue;

    for (const auto &state : history) {
      trajVertices.push_back((float)state.position.x());
      trajVertices.push_back((float)state.position.y());
      trajVertices.push_back((float)state.position.z());

      // White color
      trajVertices.push_back(1.0f);
      trajVertices.push_back(1.0f);
      trajVertices.push_back(1.0f);
    }
  }

  if (!trajVertices.empty()) {
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, trajVertices.size() * sizeof(float),
                 trajVertices.data(), GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // We need to draw separate line strips for each trajectory
    int offset = 0;
    glLineWidth(3.0f); // Make lines thicker
    glPointSize(5.0f); // Make points visible

    for (const auto &history : trajectories) {
      if (history.empty())
        continue;
      // Draw line
      glDrawArrays(GL_LINE_STRIP, offset, history.size());
      // Draw points
      glDrawArrays(GL_POINTS, offset, history.size());

      offset += history.size();
    }
  }
}

GLuint Renderer::createShader(const char *vertexSource,
                              const char *fragmentSource) {
  GLint success;
  char infoLog[512];

  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexSource, NULL);
  glCompileShader(vertexShader);

  // Check for compile errors
  glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
  glCompileShader(fragmentShader);

  // Check for compile errors
  glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  GLuint shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);

  // Check for link errors
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
              << infoLog << std::endl;
  }

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  return shaderProgram;
}
