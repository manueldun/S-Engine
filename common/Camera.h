#pragma once

#include <glm/glm.hpp>
class ProjectionCamera {
public:
  ProjectionCamera(const ProjectionCamera &projectionCamera) = default;
  ~ProjectionCamera() = default;
  ProjectionCamera &
  operator=(const ProjectionCamera &projectionCamera) = default;
  ProjectionCamera();
  ProjectionCamera(const float &aspectRatio, const float &fovy,
                   const float &near, const float &far);
  glm::mat4 getProjectionMatrix();

  float aspectRatio;
  float fovy;
  float near;
  float far;

private:
};
