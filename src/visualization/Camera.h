#pragma once
#include <Eigen/Dense>

class Camera {
public:
  Camera();

  void update();
  void processInput(struct GLFWwindow *window);

  Eigen::Matrix4f getViewMatrix() const;
  Eigen::Matrix4f getProjectionMatrix(float aspectRatio) const;

  void setDistance(float dist) { m_distance = dist; }
  void setTarget(const Eigen::Vector3d &target) {
    m_target = target.cast<float>();
  }

private:
  float m_yaw = -45.0f;
  float m_pitch = 30.0f;
  float m_distance = 50.0f;
  Eigen::Vector3f m_target = Eigen::Vector3f::Zero();

  // Mouse state
  double m_lastX = 0.0, m_lastY = 0.0;
  bool m_firstMouse = true;
  bool m_isDragging = false;
};
