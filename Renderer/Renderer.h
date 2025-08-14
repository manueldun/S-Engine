#pragma once
#include "./common/Interfaces.h"
#include "glm/fwd.hpp"
#include "imgui_impl_vulkan.h"
#include "stb_image.h"
#include "tiny_gltf.h"
#include <sys/wait.h>
#include <vulkan/vulkan_core.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <vk_mem_alloc.h>

#include "Mesh.h"
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
namespace Renderer {
struct UniformBufferObject {
  glm::mat4 model;
  glm::mat4 view;
  glm::mat4 projection;
};

struct ColorUniform {
  glm::vec3 color;
};

struct Vertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 texCoord;
  static std::vector<VkVertexInputBindingDescription> getBindingDescriptions();
  static std::vector<VkVertexInputAttributeDescription>
  getAttributeDescriptions();
};

class VulkanBuffer {
public:
  VulkanBuffer(const VulkanBuffer &buffer) = default;
  ~VulkanBuffer() = default;
  VulkanBuffer(const VkBuffer &buffer, const VmaAllocation &allocation,
               void *const bufferPointer);

  const VkBuffer buffer;
  const VmaAllocation allocation;
  void *const bufferPointer;

private:
};
struct BufferData {
  VkBuffer indexBuffer;
  VmaAllocation indexAllocation;

  VkBuffer positionBuffer;
  VmaAllocation positionAllocation;

  VkBuffer texCoordBuffer;
  VmaAllocation texCoordAllocation;

  VkBuffer normalBuffer;
  VmaAllocation normalAllocation;

  VkBuffer tangentbuffer;
  VmaAllocation tangentAllocation;
};

class MemoryData {
public:
  friend class Renderer;
  MemoryData() = delete;
  MemoryData(const VkBuffer &buffer, const VmaAllocation &allocation);

private:
  const VkBuffer m_buffer;
  const VmaAllocation m_allocation;
};

class Texture {
public:
  Texture(const VkImage &image, const VkImageView &imageView,
          const VmaAllocation &allocation, const VkSampler &sampler,
          const VkDescriptorSet &descriptorSet);
  void clean(const VkDevice &device, const VmaAllocator &allocator) const;

  const VkImage m_image;
  const VkImageView m_imageView;
  const VmaAllocation m_allocation;
  const VkSampler m_sampler;
  const VkDescriptorSet m_descriptorSet;

private:
};

class Pipeline {
public:
  Pipeline() = default;
  Pipeline(const Pipeline &pipeline) = default;
  Pipeline(const VkPipeline &pipeline, const VkPipelineLayout &pipelineLayout);
  ~Pipeline() = default;

  VkPipeline getPipeline();
  VkPipelineLayout getPipelineLayout();

private:
  VkPipeline m_pipeline;
  VkPipelineLayout m_pipelineLayout;
};

class Drawble {
public:
  friend class Renderer;
  Drawble() = delete;
  Drawble(const BufferData &bufferData, const Pipeline &pipelineTextured,
          const Pipeline &pipelineSolid, const uint32_t indexOffset,
          const uint32_t count, const VkIndexType &indexType,
          const uint32_t &indexToTexture, const uint32_t &indexToDescriptor);

  Pipeline getPipelineSolid();
  Pipeline getPipelineTextured();

private:
  const BufferData m_bufferData;
  const Pipeline m_pipelineTextured;
  const Pipeline m_pipelineSolid;
  const uint32_t m_indexOffset;
  const uint32_t m_count;
  const VkIndexType m_indexType;
  const uint32_t m_indexToTexture;
  const uint32_t m_indexToDescriptor;
};

class Node {
public:
  friend class Renderer;
  Node(const std::string &name, const std::vector<Drawble> &drawbles,
       const glm::mat4 &initialMatrix);
  virtual ~Node() = default;
  glm::mat4 getMatrix() const;
  std::string getName() const;
  void setMatrix(const glm::mat4 &matrix);
  void setName(const std::string &name);
  std::vector<Drawble> getDrawbles() const;

private:
  glm::mat4 m_matrix;
  std::string m_name;
  std::vector<Drawble> m_drawbles;
};

class ColoredNode : public Node {
public:
  ColoredNode(const std::shared_ptr<Node> &node, const glm::vec3 &color);
  virtual ~ColoredNode() = default;

  glm::vec3 getColor() const;

private:
  glm::vec3 m_color;
};

const std::vector<Vertex> vertices = {
    {{-0.5f, -0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
    {{0.5f, -0.5f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
    {{0.5f, 0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
    {{-0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f}},

    {{-0.5f, -0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
    {{0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
    {{0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
    {{-0.5f, 0.5f, -0.5f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f}}};

const std::vector<uint16_t> indices = {0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4};

VkResult CreateDebugUtilsMessengerEXT(
    const VkInstance &instance,
    const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
    const VkAllocationCallbacks *pAllocator,
    VkDebugUtilsMessengerEXT *pDebugMessenger);

void DestroyDebugUtilsMessengerEXT(
    const VkInstance &instance, const VkDebugUtilsMessengerEXT &debugMessenger,
    const VkAllocationCallbacks *pAllocator);

struct QueueFamilyIndices {
  std::optional<uint32_t> graphicsFamily;
  std::optional<uint32_t> presentFamily;
  bool isComplete() const;
};

struct SwapChainSupportDetails {
  VkSurfaceCapabilitiesKHR capabilities;
  std::vector<VkSurfaceFormatKHR> formats;
  std::vector<VkPresentModeKHR> presentModes;
};

class Drawing {
public:
  virtual ~Drawing() = default;
  Drawing(const std::shared_ptr<Pipeline> &pipeline, const int numberOfVertices,
          const std::vector<std::shared_ptr<VulkanBuffer>> &vertexVulkanBuffers,
          const std::shared_ptr<VulkanBuffer> &indexVulkanBuffer,
          const std::shared_ptr<VkDescriptorPool> &mvpDescriptorPool,
          const std::vector<std::shared_ptr<VkDescriptorSet>> &mvpDescriptorSet,
          const std::vector<std::shared_ptr<VulkanBuffer>> mvpBuffers);
  virtual void setTranform(const glm::mat4 &transform);
  virtual glm::mat4 getTranform() const;
  virtual void bind(const VkCommandBuffer &cmd, const int currentImage,
                    const VkExtent2D &swapChainExtent) const = 0;
  const std::shared_ptr<Pipeline> pipeline;
  const int numberOfVertices;
  const std::vector<std::shared_ptr<VulkanBuffer>> vertexVulkanBuffers;
  const std::shared_ptr<VulkanBuffer> indexVulkanBuffer;
  const std::shared_ptr<VkDescriptorPool> mvpDescriptorPool;
  const std::vector<std::shared_ptr<VkDescriptorSet>> mvpDescriptorSet;
  const std::vector<std::shared_ptr<VulkanBuffer>> mvpBuffers;

private:
  glm::mat4 m_tranform;
};
class SolidColorDrawing : public Drawing {
public:
  virtual ~SolidColorDrawing() = default;
  virtual void bind(const VkCommandBuffer &cmd, const int currentImage,
                    const VkExtent2D &swapChainExtent) const override;
  const std::weak_ptr<VkDescriptorSet> colorDescriptorSet;

private:
};
class TexturedDrawing : public Drawing {
public:
  virtual ~TexturedDrawing() = default;
  TexturedDrawing(const TexturedDrawing &texturedDrawing) = default;
  TexturedDrawing(
      const std::shared_ptr<Pipeline> &pipeline, const int numberOfVertices,
      const std::vector<std::shared_ptr<VulkanBuffer>> &vertexVulkanBuffers,
      const std::shared_ptr<VulkanBuffer> &indexVulkanBuffer,
      const std::shared_ptr<VkDescriptorPool> &mvpDescriptorPool,
      const std::vector<std::shared_ptr<VkDescriptorSet>> &mvpDescriptorSet,
      const std::vector<std::shared_ptr<VulkanBuffer>> mvpBuffers,
      const std::shared_ptr<VkDescriptorSet> &textureDescriptorSet);

  virtual void bind(const VkCommandBuffer &cmd, const int currentImage,
                    const VkExtent2D &swapChainExtent) const override;
  const std::shared_ptr<VkDescriptorSet> textureDescriptorSet;

private:
};
class Gui : public Subject {
public:
  Gui() = default;
  Gui(const Gui &gui) = default;
  Gui(const VkDevice &device, GLFWwindow *m_window,
      ImGui_ImplVulkan_InitInfo &initInfo);
  void drawGui(const VkCommandBuffer &commandBuffer);

private:
  VkDescriptorPool m_imguiDescriptorPool;
};
class RenderObject;
class Renderer {
public:
  friend class RenderObject;
  Renderer();
  std::shared_ptr<Drawing> loadModel(const Engine::MeshNode &meshNode);
  bool shouldExit();
  void draw(const std::shared_ptr<Drawing> &drawing);
  void drawLabel(const std::string *label);
  void endFrame();
  void destroy();
  void
  addLoadSceneEvent(std::function<void(const std::string &path)> &loadFunc);
  void addSimulationControlEvent(std::function<void()> &resumeSimulation,
                                 std::function<void()> &stopSimulation);
  void addObserver(Observer* observer);

private:
  void init();

  void initWindow();

  void initVulkan();

  void createSurface();
  void createInstance();
  bool checkValidationLayerSupport() const;
  void pickPhysicalDevice();
  bool isDeviceSuitable(VkPhysicalDevice device);
  bool checkDeviceExtensionSupport(
      const VkPhysicalDevice &device); // delete this coment

  void createLogicalDevice();
  void createVMA();
  void createSwapChain();
  VkSurfaceFormatKHR chooseSwapSurfaceFormat(
      const std::vector<VkSurfaceFormatKHR> &availableFormats);
  VkPresentModeKHR chooseSwapPresentMode(
      const std::vector<VkPresentModeKHR> &availablePresentModes);
  VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);
  QueueFamilyIndices findQueueFamilies(const VkPhysicalDevice &device);
  SwapChainSupportDetails querySwapChainSupport(const VkPhysicalDevice &device);

  void createImageViews();
  VkImageView createImageView(const VkImage &image, const VkFormat &format,
                              const VkImageAspectFlags &aspectFlags);
  void createRenderPass();
  VkFormat findDepthFormat();
  VkFormat findSupportedFormat(const std::vector<VkFormat> &candidates,
                               const VkImageTiling &tiling,
                               const VkFormatFeatureFlags &features);
  void createDepthResources();
  void createFramebuffers();
  void createCommandPool();
  void createImage(const uint32_t &width, const uint32_t &height,
                   const VkFormat &format, const VkImageTiling &tiling,
                   const VkImageUsageFlags &usage,
                   const VmaMemoryUsage &memoryUsage,
                   const VmaAllocationCreateFlags &allocationFlags,
                   VkImage &image, VmaAllocation &allocation);
  void copyBuffer(const VkBuffer &srcBuffer, const VkBuffer &dstBuffer,
                  const VkDeviceSize &size);
  VkCommandBuffer beginSingleTimeCommands();
  void transitionImageLayout(const VkImage &image, const VkFormat &format,
                             const VkImageLayout &oldLayout,
                             const VkImageLayout &newLayout);
  void copyBufferImage(const VkBuffer &buffer, const VkImage &image,
                       const uint32_t &width, const uint32_t &height);
  void endSingleTimeCommands(VkCommandBuffer &commandBuffer);
  void createDescriptorSetLayouts();
  void createCommandBuffer();
  void createWireframePipeline();
  void createSyncObjects();

  void initImGui();

  static void framebufferResizeCallback(GLFWwindow *window, int width,
                                        int height);

  std::vector<const char *> getRequiredExtensions();
  void setupDebugMessenger();
  static VKAPI_ATTR VkBool32 VKAPI_CALL
  debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                VkDebugUtilsMessageTypeFlagsEXT messageType,
                const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
                void *pUserData);

  Pipeline createGraphicPipeline(
      const std::string &vertexShaderPath,
      const std::string &fragmentShaderPath,
      const std::vector<VkVertexInputBindingDescription>
          &vertexInputBindingDescriptions,
      const std::vector<VkVertexInputAttributeDescription>
          &vertexInputAttributeDescriptions,
      const VkPolygonMode &polygonMode, const VkCullModeFlags &cullMode,
      const std::vector<VkDescriptorSetLayout> &pipelineLayouts);

  VkShaderModule createShaderModule(const std::vector<char> &code);

  VkDescriptorSet allocateTextureDescriptorSet(const VkImageView &imageview,
                                               const VkSampler &sampler);
  void allocateTextureDescriptorSet(
      std::vector<std::shared_ptr<Engine::Image>> images);

  RenderObject
  loadMeshBuffers(std::vector<std::shared_ptr<Engine::MeshNode>> nodes);
  void createBuffer(const VkDeviceSize &size, const VkBufferUsageFlags &usage,
                    const VmaMemoryUsage &memoryUsage,
                    const VmaAllocationCreateFlags &allocationFlags,
                    VkBuffer &buffer, VmaAllocation &allocation);

  void drawScene();
  void updateUniformBuffer(const uint32_t &currentImage);
  void cleanupSwapChain();
  void recreateSwapChain();
  void cleanup();

  bool hasStencilComponent(VkFormat format);

  Gui m_gui;
  std::function<void(const std::string &path)> m_loadMeshEvent;

  std::function<void()> m_stopSimulation;
  std::function<void()> m_resumeSimulation;
  bool m_isSimulationStoped = true;
  GLFWwindow *m_window;
  VkSurfaceKHR m_surface;
  VkInstance m_instance;
  VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
  const std::vector<const char *> m_deviceExtensions = {
      VK_KHR_SWAPCHAIN_EXTENSION_NAME,
      VK_KHR_SHADER_NON_SEMANTIC_INFO_EXTENSION_NAME};
  VkDevice m_device;
  VkQueue m_queue;
  VkQueue m_presentQueue;
  QueueFamilyIndices m_queueFamilyIndices;
  VmaAllocator m_vmaAllocator;
  VkFormat m_swapChainImageFormat;
  std::vector<VkImage> m_swapChainImages;
  std::vector<VkImageView> m_swapChainImageViews;
  uint32_t m_imageCount;
  VkRenderPass m_renderPass;

  VkImage m_depthImage;
  VmaAllocation m_depthAllocation;
  VkSwapchainKHR m_swapchain;
  std::vector<VkFramebuffer> m_swapChainFramebuffers;
  VkImageView m_depthImageView;
  VkCommandPool m_commandPool;
  VkExtent2D swapChainExtent;
  VkDescriptorSetLayout m_mvpDescriptorSetLayout;
  VkDescriptorSetLayout m_textureDescriptorSetLayout;
  VkDescriptorSetLayout m_solidColorDescriptorSetLayout;
  VkDescriptorPool m_imguiDescriptorPool;
  std::vector<std::shared_ptr<VkDescriptorSet>> m_mvpDescriptorSets;
  std::vector<VkDescriptorSet> m_solidColorDescriptorSets;
  std::vector<VkCommandBuffer> m_commandBuffers;
  std::vector<VkSemaphore> m_imageAvailableSemaphores;
  std::vector<VkSemaphore> m_renderFinishedSemaphores;
  std::vector<VkSemaphore> m_submitSemaphore;
  std::vector<VkFence> m_inFlightFences;
  bool m_framebufferResized = false;

  std::vector<const std::string *> m_guiLabels;

  const u_int32_t WIDTH = 800;
  const u_int32_t HEIGHT = 600;
  const std::vector<const char *> validationLayers = {
      "VK_LAYER_KHRONOS_validation"};

#ifdef NDEBUG
  const bool enableValidationLayers = false;
#else
  const bool enableValidationLayers = true;
#endif
  std::vector<const char *> extensions;
  VkDebugUtilsMessengerEXT m_debugMessenger;

  uint32_t m_currentFrame = 0;
  std::vector<VkDescriptorPool> m_imageDescriptorPools;
  std::vector<std::shared_ptr<VkDescriptorSet>> m_imageDescriptorSetPtr;
  std::vector<Texture> m_loadedTextures;
  std::vector<std::shared_ptr<VulkanBuffer>> m_vertexBuffers;
  std::vector<std::shared_ptr<VulkanBuffer>> m_indexBuffers;

  std::vector<std::shared_ptr<VulkanBuffer>> m_mvpVulkanBuffers;
  uint32_t findMemoryType(const uint32_t &typeFilter,
                          const VkMemoryPropertyFlags &properties);
  ImGui_ImplVulkanH_Window m_imguiWindow;
  std::vector<BufferData> m_loadedBufferData;

  std::vector<std::shared_ptr<Drawing>> m_drawings;
  std::vector<std::shared_ptr<Pipeline>> m_pipelines;
  std::vector<std::shared_ptr<Drawing>> m_toDraw;
};

class RenderObject {
public:
  friend class Renderer;
  RenderObject() = delete;
  RenderObject(const std::vector<std::shared_ptr<Node>> &nodes,
               const tinygltf::Model &model);
  void setMatrix(const glm::mat4 &matrix, const uint32_t index);
  RenderObject(const RenderObject &other) = default;
  glm::vec3 getCenterOfMass(const bool &verbose) const;
  glm::mat4 getInertiaTensor(const bool &verbose) const;

private:
  std::vector<std::shared_ptr<Node>> m_nodes;
  tinygltf::Model m_model;
};
} // namespace Renderer
