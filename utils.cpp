#include "utils.h"
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <span>
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"

std::vector<char> readFile(const std::string &filename) {
  std::ifstream file(filename, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    throw std::runtime_error("failed to open file!");
  }

  size_t fileSize{(size_t)file.tellg()};
  std::vector<char> buffer(fileSize);

  file.seekg(0);
  file.read(buffer.data(), fileSize);
  file.close();

  return buffer;
}
/* Determines if the normal of a triangle points towards a point following the
 * right hand ordering to determinate the triangle normal direction
 */
constexpr bool isTowardsPlaneNormal(std::array<glm::vec3, 3> trianglePlane,
                                    glm::vec3 point) {

  glm::vec3 triangleEdge1{trianglePlane[1] - trianglePlane[0]};
  glm::vec3 triangleEdge2{trianglePlane[2] - trianglePlane[0]};
  glm::vec3 normal{glm::cross(triangleEdge1, triangleEdge2)};
  float dotProduct{glm::dot(normal, point)};
  return dotProduct > 0.0f;
}

tinygltf::Model loadGltfFile(const std::string &path) {
  tinygltf::Model model;
  tinygltf::TinyGLTF loader;
  std::string err;
  std::string warn;

  bool ret = loader.LoadASCIIFromFile(&model, &err, &warn, path);

  if (!warn.empty()) {
    std::cout << "Warn: " << warn << std::endl;
  }

  if (!ret) {
    throw std::runtime_error("Error loading gltf file");
  }

  if (!err.empty()) {
    std::cout << "Error: " << err << std::endl;
  }
  return model;
}

std::vector<std::span<glm::vec3>>
getVerticeData(const tinygltf::Model &model, const std::string &attributeName) {
  std::vector<std::span<glm::vec3>> bufferSpans;

  for (const tinygltf::Node &node : model.nodes) {
    tinygltf::Mesh mesh = model.meshes[node.mesh];
    assert(mesh.primitives.size() > 1);
    assert(mesh.primitives[0].attributes.count(attributeName));
    if (mesh.primitives.size() > 0) {

      const int primitiveIndex = mesh.primitives[0].attributes[attributeName];
      tinygltf::Accessor accessor = model.accessors[primitiveIndex];
      const int bufferViewIndex = accessor.bufferView;
      tinygltf::BufferView bufferView = model.bufferViews[bufferViewIndex];
      const int bufferIndex = bufferView.buffer;
      tinygltf::Buffer buffer = model.buffers[bufferIndex];
      assert(accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT);
      const size_t componentBytes{4};
      size_t numberOfComponents{0};
      switch (accessor.type) {
      case TINYGLTF_TYPE_SCALAR:
        numberOfComponents = 1;
        break;
      case TINYGLTF_TYPE_VEC2:
        numberOfComponents = 2;
        break;
      case TINYGLTF_TYPE_VEC3:
        numberOfComponents = 3;
        break;
      case TINYGLTF_TYPE_VEC4:
        numberOfComponents = 4;
        break;
      default:
        std::cerr << "accessor type not supported" << std::endl;
        assert(false);
      }
      const size_t numberOfVertices =
          bufferView.byteLength / componentBytes / numberOfComponents;
      glm::vec3 *vec3ptr = reinterpret_cast<glm::vec3 *>(buffer.data.data());
      std::span<glm::vec3> vecBufferSpan(vec3ptr, numberOfVertices);
      bufferSpans.push_back(vecBufferSpan);
    }
  }

  return bufferSpans;
}
constexpr IndexDataSpan::IndexDataSpan(const std::span<uint8_t> &indexSpan,
                                       const uint8_t &bytesperInt)
    : m_indexSpan{indexSpan}, m_bytesPerInt{bytesperInt} {}

constexpr std::span<uint8_t> IndexDataSpan::getIndexSpan() const {

  return m_indexSpan;
}
constexpr int getByteWidthFromComponentType(const int &componentType) {

  switch (componentType) {
  case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
    return 1;
    break;
  }
  case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
    return 2;
    break;
  }
  case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
    return 4;
    break;
  }
  default: {
    return 0;
  }
  }
}
constexpr uint8_t IndexDataSpan::getBytesPerInt() const {
  return m_bytesPerInt;
}
std::vector<IndexDataSpan> getIndexSpans(const tinygltf::Model &model) {

  std::vector<IndexDataSpan> bufferSpans;

  for (const tinygltf::Node &node : model.nodes) {
    tinygltf::Mesh mesh = model.meshes[node.mesh];
    assert(mesh.primitives.size() > 1);
    if (mesh.primitives.size() > 0) {

      const int accesorIndex{mesh.primitives[0].indices};
      tinygltf::Accessor accessor = model.accessors[accesorIndex];
      const int bufferViewIndex{accessor.bufferView};
      tinygltf::BufferView bufferView = model.bufferViews[bufferViewIndex];
      const int bufferIndex = bufferView.buffer;
      tinygltf::Buffer buffer{model.buffers[bufferIndex]};

      int componentBytes{getByteWidthFromComponentType(accessor.componentType)};

      uint8_t *intPtr = reinterpret_cast<uint8_t *>(buffer.data.data());
      std::span<uint8_t> bufferSpan(intPtr, intPtr + bufferView.byteLength);
      bufferSpans.push_back(IndexDataSpan(bufferSpan, componentBytes));
      assert(accessor.type == TINYGLTF_TYPE_SCALAR);
    }
  }

  return bufferSpans;
}

glm::vec3 getCenterOfMass(const std::span<glm::vec3> &vertices,
                                    const IndexDataSpan &dataSpan,
                                    const bool &verbose) {

  switch (dataSpan.getBytesPerInt()) {
  case 1: {
    return getCenterOfMass<uint8_t>(vertices, dataSpan.getIndexSpan(), verbose);
  } break;
  case 2: {
    uint16_t *intPtr =
        reinterpret_cast<uint16_t *>(dataSpan.getIndexSpan().data());
    size_t numberOfIndices{dataSpan.getIndexSpan().size() /
                           dataSpan.getBytesPerInt()};
    std::span<uint16_t> span16(intPtr, intPtr + numberOfIndices);
    return getCenterOfMass<uint16_t>(vertices, span16, verbose);
    break;
  }
  case 3: {
    uint32_t *intPtr =
        reinterpret_cast<uint32_t *>(dataSpan.getIndexSpan().data());
    auto numberOfIndices{dataSpan.getIndexSpan().size() /
                         dataSpan.getBytesPerInt()};
    std::span<uint32_t> span16(intPtr, intPtr + numberOfIndices);
    return getCenterOfMass<uint32_t>(vertices, span16, verbose);
  } break;
  default:
    assert(false);
  }
}

template <typename T>
constexpr glm::vec3 getCenterOfMass(const std::span<glm::vec3> &vertices,
                                    const std::span<T> &indices,
                                    const bool &verbose) {

  unsigned int numOfVertices{0};
  glm::vec3 centerOfMass{0.0f};
  float totalArea{0.0f};

  if (verbose) {
    std::cout << "numOfTriangles: " << numOfVertices << std::endl;
  }
  for (size_t vertexIndex = 0; vertexIndex < numOfVertices; vertexIndex += 3) {
    if (verbose) {
      std::cout << "vertexIndex: " << vertexIndex << std::endl;
      std::cout << "indices" << std::endl;
      std::cout << indices[vertexIndex] << std::endl;
      std::cout << indices[vertexIndex + 1] << std::endl;
      std::cout << indices[vertexIndex + 2] << std::endl;
    }

    glm::vec3 vertex1{vertices[indices[vertexIndex]]};
    if (verbose) {
      std::cout << "vec1" << std::endl;

      std::cout << vertex1.x << " " << vertex1.y << " " << vertex1.z
                << std::endl;
    }
    glm::vec3 vertex2{vertices[indices[vertexIndex + 1]]};
    if (verbose) {
      std::cout << "vec2" << std::endl;
      std::cout << vertex2.x << " " << vertex2.y << " " << vertex2.z
                << std::endl;
    }
    glm::vec3 vertex3{vertices[indices[vertexIndex + 2]]};
    if (verbose) {
      std::cout << "vec3" << std::endl;
      std::cout << vertex3.x << " " << vertex3.y << " " << vertex3.z
                << std::endl;
    }

    glm::vec3 triangleCentroid = (vertex1 + vertex2 + vertex3) / 3.0f;

    if (verbose) {
      std::cout << "centroid" << std::endl;
      std::cout << triangleCentroid.x << " " << triangleCentroid.y << " "
                << triangleCentroid.z << std::endl;
    }
    glm::vec3 a{vertex2 - vertex1};
    glm::vec3 b{vertex3 - vertex1};
    glm::vec3 crossVector{glm::cross(a, b)};
    float area{crossVector.length() / 2.0f};
    totalArea += area;
    centerOfMass += triangleCentroid * area;
    if (verbose) {
      std::cout << "Acumulated center of mass:" << std::endl;
      std::cout << centerOfMass.x << " " << centerOfMass.y << " "
                << centerOfMass.z << std::endl;
    }
  }
  return centerOfMass / totalArea;
}
template <typename T>
constexpr glm::mat4 getInertiaTensor(const std::span<glm::vec3> &vertices,
                                     const std::span<T> &indices,
                                     const bool &verbose) {

  glm::mat3 acumulatedInertialTensor = glm::mat3(0.0f);

  uint32_t numsOfTetrahedrons = 0;
  for (size_t vertexIndex = 0; vertexIndex < indices.size(); vertexIndex += 3) {

    glm::vec3 vertex1 = glm::vec3(0.0f);
    glm::vec3 vertex2 = vertices[indices[vertexIndex] * 3];
    glm::vec3 vertex3 = vertices[indices[vertexIndex + 1] * 3];
    glm::vec3 vertex4 = vertices[indices[vertexIndex + 2] * 3];

    glm::mat3 jacobian = glm::mat3(vertex2, vertex3, vertex4);
    float det = glm::determinant(jacobian);

    if (verbose) {
      std::cout << "First Vertex: " << std::endl;
      std::cout << vertex2.x << "\t" << vertex2.y << "\t" << vertex2.z
                << std::endl;
      std::cout << "Second Vertex: " << std::endl;
      std::cout << vertex3.x << "\t" << vertex3.y << "\t" << vertex3.z
                << std::endl;
      std::cout << "Third Vertex: " << std::endl;
      std::cout << vertex4.x << "\t" << vertex4.y << "\t" << vertex4.z
                << std::endl;

      std::cout << "Jacobian:" << std::endl;
      std::cout << jacobian[0][0] << "\t" << jacobian[0][1] << "\t"
                << jacobian[0][2] << "\t" << std::endl;

      std::cout << jacobian[1][0] << "\t" << jacobian[1][1] << "\t"
                << jacobian[1][2] << "\t" << std::endl;

      std::cout << jacobian[2][0] << "\t" << jacobian[2][1] << "\t"
                << jacobian[2][2] << "\t" << std::endl;
    }
    std::cout << "Determinant of the jacobian : " << det << std::endl;

    float abcx = vertex1.x * vertex1.x + vertex1.x * vertex2.x +
                 vertex2.x * vertex2.x + vertex1.x * vertex3.x +
                 vertex2.x * vertex3.x + vertex3.x * vertex3.x +
                 vertex1.x * vertex4.x + vertex2.x * vertex4.x +
                 vertex3.x * vertex4.x + vertex4.x * vertex4.x;

    float abcy = vertex1.y * vertex1.y + vertex1.y * vertex2.y +
                 vertex2.y * vertex2.y + vertex1.y * vertex3.y +
                 vertex2.y * vertex3.y + vertex3.y * vertex3.y +
                 vertex1.y * vertex4.y + vertex2.y * vertex4.y +
                 vertex3.y * vertex4.y + vertex4.y * vertex4.y;

    float abcz = vertex1.z * vertex1.z + vertex1.z * vertex2.z +
                 vertex2.z * vertex2.z + vertex1.z * vertex3.z +
                 vertex2.z * vertex3.z + vertex3.z * vertex3.z +
                 vertex1.z * vertex4.z + vertex2.z * vertex4.z +
                 vertex3.z * vertex4.z + vertex4.z * vertex4.z;
    float a = (det * (abcy + abcz) / 60.0f) / (det / 6.0);
    float b = (det * (abcx + abcz) / 60.0f) / (det / 6.0);
    float c = (det * (abcx + abcy) / 60.0f) / (det / 6.0);

    float abcxp =
        vertex2.y * vertex1.z + vertex3.y * vertex1.z + vertex4.y * vertex1.z +
        vertex1.y * vertex2.z + vertex3.y * vertex2.z + vertex4.y * vertex2.z +
        vertex1.y * vertex3.z + vertex2.y * vertex3.z + vertex4.y * vertex3.z +
        vertex1.y * vertex4.z + vertex2.y * vertex4.z + vertex3.y * vertex4.z +
        2.0f * vertex1.y * vertex1.z + 2.0f * vertex2.y * vertex2.z +
        2.0f * vertex3.y * vertex3.z + 2.0f * vertex4.y * vertex4.z;

    float abcyp =
        vertex2.x * vertex1.z + vertex3.x * vertex1.z + vertex4.x * vertex1.z +
        vertex1.x * vertex2.z + vertex3.x * vertex2.z + vertex4.x * vertex2.z +
        vertex1.x * vertex3.z + vertex2.x * vertex3.z + vertex4.x * vertex3.z +
        vertex1.x * vertex4.z + vertex2.x * vertex4.z + vertex3.x * vertex4.z +
        2.0f * vertex1.x * vertex1.z + 2.0f * vertex2.x * vertex2.z +
        2.0f * vertex3.x * vertex3.z + 2.0f * vertex4.x * vertex4.z;

    float abczp =
        vertex2.x * vertex1.y + vertex3.x * vertex1.y + vertex4.x * vertex1.y +
        vertex1.x * vertex2.y + vertex3.x * vertex2.y + vertex4.x * vertex2.y +
        vertex1.x * vertex3.y + vertex2.x * vertex3.y + vertex4.x * vertex3.y +
        vertex1.x * vertex4.y + vertex2.x * vertex4.y + vertex3.x * vertex4.y +
        2.0f * vertex1.x * vertex1.y + 2.0f * vertex2.x * vertex2.y +
        2.0f * vertex3.x * vertex3.y + 2.0f * vertex4.x * vertex4.y;

    float ap = (det * abcxp / 120.0f) / (det / 6.0);
    float bp = (det * abcyp / 120.0f) / (det / 6.0);
    float cp = (det * abczp / 120.0f) / (det / 6.0);

    float tetrahedronInertiaTensorArray[] = {a,   -bp, -cp, -bp, b,
                                             -ap, -cp, -ap, c};
    glm::mat3 tetrahedronInertiaTensor =
        glm::make_mat3(tetrahedronInertiaTensorArray);
    if (verbose) {

      std::cout << "Acumulated tetrahedron inertial tensor:" << std::endl;
      std::cout << tetrahedronInertiaTensor[0][0] << "\t"
                << tetrahedronInertiaTensor[0][1] << "\t"
                << tetrahedronInertiaTensor[0][2] << "\t" << std::endl;

      std::cout << tetrahedronInertiaTensor[1][0] << "\t"
                << tetrahedronInertiaTensor[1][1] << "\t"
                << tetrahedronInertiaTensor[1][2] << "\t" << std::endl;

      std::cout << tetrahedronInertiaTensor[2][0] << "\t"
                << tetrahedronInertiaTensor[2][1] << "\t"
                << tetrahedronInertiaTensor[2][2] << "\t" << std::endl;
    }
    acumulatedInertialTensor += tetrahedronInertiaTensor;
    numsOfTetrahedrons++;
  }
  std::cout << "Number of tetrahedrons: " << numsOfTetrahedrons << std::endl;
  return acumulatedInertialTensor / static_cast<float>(numsOfTetrahedrons);
}
