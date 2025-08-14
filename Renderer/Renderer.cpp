#include "Renderer.h"

#include "GLFW/glfw3.h"
#include "ImGuiFileDialog/ImGuiFileDialog.h"
#include "Mesh.h"
#include "glm/ext/vector_float3.hpp"
#include "glm/fwd.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"
#include <memory>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#include <array>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <vulkan/vulkan_core.h>
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

#include "utils.h"
#include <algorithm> // Necessary for std::clamp
#include <cassert>
#include <cstdint> // Necessary for uint32_t
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits> // Necessary for std::numeric_limits
#include <set>
#include <stdexcept>
namespace Renderer {
Renderer::Renderer() { init(); }
std::vector<VkVertexInputBindingDescription> Vertex::getBindingDescriptions() {
  VkVertexInputBindingDescription bindingDescription{};
  bindingDescription.binding = 0;
  bindingDescription.stride = sizeof(Vertex);
  bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  std::vector<VkVertexInputBindingDescription> bindingDescriptions;
  bindingDescriptions.push_back(bindingDescription);
  return bindingDescriptions;
}

std::vector<VkVertexInputAttributeDescription>
Vertex::getAttributeDescriptions() {
  std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};
  attributeDescriptions.resize(3);
  // position attribute
  attributeDescriptions[0].binding = 0;
  attributeDescriptions[0].location = 0;
  attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
  attributeDescriptions[0].offset = offsetof(Vertex, position);

  // normal attribute
  attributeDescriptions[1].binding = 0;
  attributeDescriptions[1].location = 1;
  attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
  attributeDescriptions[1].offset = offsetof(Vertex, normal);

  // texture coord attribute
  attributeDescriptions[2].binding = 0;
  attributeDescriptions[2].location = 2;
  attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
  attributeDescriptions[2].offset = offsetof(Vertex, texCoord);
  return attributeDescriptions;
}

VulkanBuffer::VulkanBuffer(const VkBuffer &buffer,
                           const VmaAllocation &allocation,
                           void *const bufferPointer)
    : buffer(buffer), allocation(allocation), bufferPointer(bufferPointer) {}
bool QueueFamilyIndices::isComplete() const {
  return graphicsFamily.has_value() && presentFamily.has_value();
}

void Renderer::init() {
  initWindow();
  initVulkan();
  initImGui();
}
void Renderer::destroy() { cleanup(); }

void Renderer::addLoadSceneEvent(
    std::function<void(const std::string &path)> &loadFunc) {
  m_loadMeshEvent = loadFunc;
}
void Renderer::addSimulationControlEvent(
    std::function<void()> &resumeSimulation,
    std::function<void()> &stopSimulation) {
  m_resumeSimulation = resumeSimulation;
  m_stopSimulation = stopSimulation;
}
void Renderer::initWindow() {
  glfwInit();

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  if (!glfwVulkanSupported()) {
    throw std::runtime_error("Vulkan not supported!");
  }

  m_window = glfwCreateWindow(WIDTH, HEIGHT, "Vulkan", nullptr, nullptr);

  glfwSetWindowUserPointer(m_window, this);
  glfwSetFramebufferSizeCallback(m_window, framebufferResizeCallback);
}

void Renderer::framebufferResizeCallback(GLFWwindow *window, int width,
                                         int height) {
  auto app = reinterpret_cast<Renderer *>(glfwGetWindowUserPointer(window));
  app->m_framebufferResized = true;
}
bool Renderer::checkValidationLayerSupport() const {
  uint32_t layerCount;
  vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

  std::vector<VkLayerProperties> availableLayers(layerCount);
  vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

  for (const char *layerName : validationLayers) {
    bool layerFound = false;
    for (const auto &layerProperties : availableLayers) {
      if (strcmp(layerName, layerProperties.layerName) == 0) {
        layerFound = true;
        break;
      }
    }
    if (!layerFound) {
      return false;
    }
  }
  return true;
}

void Renderer::initVulkan() {
  createInstance();
  setupDebugMessenger();
  createSurface();
  pickPhysicalDevice();
  createLogicalDevice();
  createVMA();
  createSwapChain();
  createSyncObjects();
  createImageViews();
  createRenderPass();
  createDepthResources();
  createFramebuffers();
  createCommandPool();
  createDescriptorSetLayouts();
  createCommandBuffer();
}

std::vector<const char *> Renderer::getRequiredExtensions() {
  uint32_t glfwExtensionCount = 0;
  const char **glfwExtensions;
  glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
  std::vector<const char *> extensions(glfwExtensions,
                                       glfwExtensions + glfwExtensionCount);
  if (enableValidationLayers) {
    extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    // extensions.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
  }
  return extensions;
}

void Renderer::createInstance() {
  if (enableValidationLayers && !checkValidationLayerSupport()) {
    throw std::runtime_error("validation layers requested, but not available!");
  }
  VkApplicationInfo appInfo{};
  appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  appInfo.pApplicationName = "My Vulkan Tutorial";
  appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
  appInfo.pEngineName = "No engine";
  appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
  appInfo.apiVersion = VK_API_VERSION_1_0;

  VkInstanceCreateInfo instanceCreateInfo{};
  instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instanceCreateInfo.pApplicationInfo = &appInfo;

  uint32_t glfwExtensionCount = 0;

  const char **glfwExtensions =
      glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

  std::vector<const char *> extensions(glfwExtensions,
                                       glfwExtensions + glfwExtensionCount);

  extensions.push_back("VK_EXT_debug_utils");
  instanceCreateInfo.enabledExtensionCount =
      static_cast<uint32_t>(extensions.size());
  instanceCreateInfo.ppEnabledExtensionNames = extensions.data();
  instanceCreateInfo.enabledLayerCount = extensions.size();

  if (enableValidationLayers) {
    instanceCreateInfo.enabledLayerCount =
        static_cast<uint32_t>(validationLayers.size());
    instanceCreateInfo.ppEnabledLayerNames = validationLayers.data();
  } else {
    instanceCreateInfo.enabledLayerCount = 0;
  }

  if (vkCreateInstance(&instanceCreateInfo, nullptr, &m_instance)) {
    throw std::runtime_error("failed to create instance");
  }
}

void Renderer::setupDebugMessenger() {
  if (!enableValidationLayers)
    return;

  VkDebugUtilsMessengerCreateInfoEXT debugUtilsMessengerCreateInfo{};
  debugUtilsMessengerCreateInfo.sType =
      VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
  debugUtilsMessengerCreateInfo.messageSeverity =
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
  debugUtilsMessengerCreateInfo.messageType =
      VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
  debugUtilsMessengerCreateInfo.pfnUserCallback = debugCallback;
  debugUtilsMessengerCreateInfo.pUserData = nullptr;

  if (CreateDebugUtilsMessengerEXT(m_instance, &debugUtilsMessengerCreateInfo,
                                   nullptr, &m_debugMessenger) != VK_SUCCESS) {
    throw std::runtime_error("failed to set up debug messenger!");
  }
}

void Renderer::createSurface() {
  if (glfwCreateWindowSurface(m_instance, m_window, nullptr, &m_surface) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to create window surface!");
  }
}

void Renderer::pickPhysicalDevice() {
  uint32_t deviceCount = 0;
  vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);

  if (deviceCount == 0) {
    throw std::runtime_error("failed to find gpu with vulkan support");
  }

  std::vector<VkPhysicalDevice> devices(deviceCount);

  vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

  for (const auto &device : devices) {
    if (isDeviceSuitable(device)) {
      m_physicalDevice = device;
      break;
    }
  }

  if (m_physicalDevice == VK_NULL_HANDLE) {
    throw std::runtime_error("failed to find suitable gpu");
  }
}

bool Renderer::isDeviceSuitable(VkPhysicalDevice device) {
  QueueFamilyIndices indices = findQueueFamilies(device);

  bool extensionSupported = checkDeviceExtensionSupport(device);

  bool swapChainAdequate = false;

  if (extensionSupported) {
    SwapChainSupportDetails swapcChainSupport = querySwapChainSupport(device);
    swapChainAdequate = !swapcChainSupport.formats.empty() &&
                        !swapcChainSupport.presentModes.empty();
  }
  VkPhysicalDeviceFeatures supportedFeatures;
  vkGetPhysicalDeviceFeatures(device, &supportedFeatures);
  return indices.isComplete() && extensionSupported && swapChainAdequate &&
         supportedFeatures.samplerAnisotropy;
}

VkSurfaceFormatKHR Renderer::chooseSwapSurfaceFormat(
    const std::vector<VkSurfaceFormatKHR> &availableFormats) {
  for (const auto &availableFormat : availableFormats) {
    if (availableFormat.format == VK_FORMAT_R8G8B8_SRGB &&
        availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
      return availableFormat;
    }
  }
  return availableFormats[0];
}

VkPresentModeKHR Renderer::chooseSwapPresentMode(
    const std::vector<VkPresentModeKHR> &availablePresentModes) {
  for (const auto &availablePresentMode : availablePresentModes) {
    if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
      return availablePresentMode;
    }
  }
  return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D
Renderer::chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities) {
  if (capabilities.currentExtent.width !=
      std::numeric_limits<uint32_t>::max()) {
    return capabilities.currentExtent;
  } else {
    int width, height;

    glfwGetFramebufferSize(m_window, &width, &height);
    VkExtent2D actualExtent = {static_cast<uint32_t>(width),
                               static_cast<uint32_t>(height)};

    actualExtent.width =
        std::clamp(actualExtent.width, capabilities.maxImageExtent.width,
                   capabilities.minImageExtent.width);
    actualExtent.height =
        std::clamp(actualExtent.height, capabilities.maxImageExtent.height,
                   capabilities.minImageExtent.height);
    return actualExtent;
  }
}

bool Renderer::checkDeviceExtensionSupport(const VkPhysicalDevice &device) {
  uint32_t extensionCount;
  vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount,
                                       nullptr);
  std::vector<VkExtensionProperties> availableExtensions(extensionCount);
  vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount,
                                       availableExtensions.data());

  std::set<std::string> requiredExtensions(m_deviceExtensions.begin(),
                                           m_deviceExtensions.end());

  for (const auto &extension : availableExtensions) {
    requiredExtensions.erase(extension.extensionName);
  }
  return requiredExtensions.empty();
}

QueueFamilyIndices Renderer::findQueueFamilies(const VkPhysicalDevice &device) {
  QueueFamilyIndices queueFamilyIndices;
  uint32_t queueFamilyCount = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

  std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
  vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount,
                                           queueFamilies.data());

  int i = 0;
  for (const auto &queueFamily : queueFamilies) {
    if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
      queueFamilyIndices.graphicsFamily = i;
    }
    VkBool32 presentSupport = false;
    vkGetPhysicalDeviceSurfaceSupportKHR(device, i, m_surface, &presentSupport);
    if (presentSupport) {
      queueFamilyIndices.presentFamily = i;
    }
    if (queueFamilyIndices.graphicsFamily.has_value()) {
      break;
    }
    i++;
  }
  return queueFamilyIndices;
}

SwapChainSupportDetails
Renderer::querySwapChainSupport(const VkPhysicalDevice &device) {
  SwapChainSupportDetails details;
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, m_surface,
                                            &details.capabilities);

  uint32_t formatCount;
  vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &formatCount,
                                       nullptr);

  if (formatCount != 0) {
    details.formats.resize(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &formatCount,
                                         details.formats.data());
  }

  uint32_t presentModeCount;
  vkGetPhysicalDeviceSurfacePresentModesKHR(device, m_surface,
                                            &presentModeCount, nullptr);

  if (presentModeCount != 0) {
    details.presentModes.resize(presentModeCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(
        device, m_surface, &presentModeCount, details.presentModes.data());
  }
  return details;
}

void Renderer::createSwapChain() {
  SwapChainSupportDetails swapChainSupport =
      querySwapChainSupport(m_physicalDevice);

  VkSurfaceFormatKHR surfaceFormat =
      chooseSwapSurfaceFormat(swapChainSupport.formats);

  VkPresentModeKHR presentMode =
      chooseSwapPresentMode(swapChainSupport.presentModes);

  VkExtent2D extent = chooseSwapExtent(swapChainSupport.capabilities);

  m_imageCount = swapChainSupport.capabilities.minImageCount + 1;

  if (swapChainSupport.capabilities.maxImageCount != 0) {
    if (swapChainSupport.capabilities.minImageCount > 0 &&
        m_imageCount > swapChainSupport.capabilities.maxImageCount) {

      m_imageCount = swapChainSupport.capabilities.maxImageCount;
    }
  }

  VkSwapchainCreateInfoKHR swapchainCreateInfo{};
  swapchainCreateInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  swapchainCreateInfo.surface = m_surface;
  swapchainCreateInfo.minImageCount = m_imageCount;
  swapchainCreateInfo.imageFormat = surfaceFormat.format;
  swapchainCreateInfo.imageColorSpace = surfaceFormat.colorSpace;
  swapchainCreateInfo.imageExtent = extent;
  swapchainCreateInfo.imageArrayLayers = 1;
  swapchainCreateInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

  QueueFamilyIndices queueFamilyIndicesStruct =
      findQueueFamilies(m_physicalDevice);

  uint32_t queueFamilyIndices[] = {
      queueFamilyIndicesStruct.graphicsFamily.value(),
      queueFamilyIndicesStruct.presentFamily.value()};

  if (queueFamilyIndicesStruct.graphicsFamily !=
      queueFamilyIndicesStruct.presentFamily) {
    swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
    swapchainCreateInfo.queueFamilyIndexCount = 2;
    swapchainCreateInfo.pQueueFamilyIndices = queueFamilyIndices;
  } else {
    swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    swapchainCreateInfo.queueFamilyIndexCount = 0;
    swapchainCreateInfo.pQueueFamilyIndices = nullptr;
  }
  swapchainCreateInfo.preTransform =
      swapChainSupport.capabilities.currentTransform;
  swapchainCreateInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  swapchainCreateInfo.presentMode = presentMode;
  swapchainCreateInfo.clipped = VK_TRUE;
  swapchainCreateInfo.oldSwapchain = VK_NULL_HANDLE;

  if (vkCreateSwapchainKHR(m_device, &swapchainCreateInfo, nullptr,
                           &m_swapchain) != VK_SUCCESS) {
    throw std::runtime_error("failed to create swapchain!");
  }

  vkGetSwapchainImagesKHR(m_device, m_swapchain, &m_imageCount, nullptr);
  m_swapChainImages.resize(m_imageCount);
  vkGetSwapchainImagesKHR(m_device, m_swapchain, &m_imageCount,
                          m_swapChainImages.data());
  m_swapChainImageFormat = surfaceFormat.format;
  swapChainExtent = extent;
}

void Renderer::createImageViews() {
  m_swapChainImageViews.resize(m_swapChainImages.size());
  for (size_t i = 0; i < m_swapChainImageViews.size(); i++) {
    m_swapChainImageViews[i] =
        createImageView(m_swapChainImages[i], m_swapChainImageFormat,
                        VK_IMAGE_ASPECT_COLOR_BIT);
  }
}

VkShaderModule Renderer::createShaderModule(const std::vector<char> &code) {
  VkShaderModuleCreateInfo createInfoshaderModuleCreateInfo{};
  createInfoshaderModuleCreateInfo.sType =
      VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  createInfoshaderModuleCreateInfo.codeSize = code.size();
  createInfoshaderModuleCreateInfo.pCode =
      reinterpret_cast<const uint32_t *>(code.data());
  VkShaderModule shaderModule;
  if (vkCreateShaderModule(m_device, &createInfoshaderModuleCreateInfo, nullptr,
                           &shaderModule) != VK_SUCCESS) {
    throw std::runtime_error("failed to create shader module");
  }

  return shaderModule;
}

void Renderer::createRenderPass() {
  VkAttachmentDescription colorAttachment{};
  colorAttachment.format = m_swapChainImageFormat;
  colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
  colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

  VkAttachmentDescription depthAttachment{};
  depthAttachment.format = findDepthFormat();
  depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
  depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  depthAttachment.finalLayout =
      VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkAttachmentReference colorAttachmentRef{};
  colorAttachmentRef.attachment = 0;
  colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  VkAttachmentReference depthAttachmentRef{};
  depthAttachmentRef.attachment = 1;
  depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkSubpassDescription subpassDescription{};
  subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpassDescription.colorAttachmentCount = 1;
  subpassDescription.pColorAttachments = &colorAttachmentRef;
  subpassDescription.pDepthStencilAttachment = &depthAttachmentRef;

  std::array<VkAttachmentDescription, 2> attachments = {colorAttachment,
                                                        depthAttachment};

  VkRenderPassCreateInfo renderPasscreateInfo{};
  renderPasscreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  renderPasscreateInfo.attachmentCount =
      static_cast<uint32_t>(attachments.size());
  renderPasscreateInfo.pAttachments = attachments.data();
  renderPasscreateInfo.subpassCount = 1;
  renderPasscreateInfo.pSubpasses = &subpassDescription;

  VkSubpassDependency subpassDependency{};
  subpassDependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  subpassDependency.dstSubpass = 0;
  subpassDependency.srcStageMask =
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
      VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  subpassDependency.srcAccessMask = 0;
  subpassDependency.dstStageMask =
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
      VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  subpassDependency.dstAccessMask =
      VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
      VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

  renderPasscreateInfo.dependencyCount = 1;
  renderPasscreateInfo.pDependencies = &subpassDependency;

  if (vkCreateRenderPass(m_device, &renderPasscreateInfo, nullptr,
                         &m_renderPass) != VK_SUCCESS) {
    throw std::runtime_error("failed to create renderpass");
  }
}

Pipeline Renderer::createGraphicPipeline(
    const std::string &vertexShaderPath, const std::string &fragmentShaderPath,
    const std::vector<VkVertexInputBindingDescription>
        &vertexInputBindingDescriptions,
    const std::vector<VkVertexInputAttributeDescription>
        &vertexInputAttributeDescriptions,
    const VkPolygonMode &polygonMode, const VkCullModeFlags &cullMode,
    const std::vector<VkDescriptorSetLayout> &pipelineLayouts) {
  std::vector<char> vertShaderCode = util::readFile(vertexShaderPath);
  std::vector<char> fragShaderCode = util::readFile(fragmentShaderPath);

  VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
  VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

  VkPipelineShaderStageCreateInfo vertPipelineShaderStageCreateInfo{};
  vertPipelineShaderStageCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  vertPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
  vertPipelineShaderStageCreateInfo.module = vertShaderModule;
  vertPipelineShaderStageCreateInfo.pName = "main";

  VkPipelineShaderStageCreateInfo fragPipelineShaderStageCreateInfo{};
  fragPipelineShaderStageCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  fragPipelineShaderStageCreateInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  fragPipelineShaderStageCreateInfo.module = fragShaderModule;
  fragPipelineShaderStageCreateInfo.pName = "main";

  VkPipelineShaderStageCreateInfo shaderStages[] = {
      vertPipelineShaderStageCreateInfo, fragPipelineShaderStageCreateInfo};

  std::vector<VkDynamicState> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT,
                                               VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dynamicStatepipelineDynamicStateCreateInfo{};
  dynamicStatepipelineDynamicStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamicStatepipelineDynamicStateCreateInfo.dynamicStateCount =
      static_cast<uint32_t>(dynamicStates.size());
  dynamicStatepipelineDynamicStateCreateInfo.pDynamicStates =
      dynamicStates.data();

  VkPipelineVertexInputStateCreateInfo vertexPipelineInputStateCreateInfo{};
  vertexPipelineInputStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

  vertexPipelineInputStateCreateInfo.vertexBindingDescriptionCount =
      vertexInputBindingDescriptions.size();
  vertexPipelineInputStateCreateInfo.pVertexBindingDescriptions =
      vertexInputBindingDescriptions.data();

  vertexPipelineInputStateCreateInfo.vertexAttributeDescriptionCount =
      static_cast<uint32_t>(vertexInputAttributeDescriptions.size());
  vertexPipelineInputStateCreateInfo.pVertexAttributeDescriptions =
      vertexInputAttributeDescriptions.data();

  VkPipelineInputAssemblyStateCreateInfo pipelineInputAssemblyStateCreateInfo{};
  pipelineInputAssemblyStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  pipelineInputAssemblyStateCreateInfo.topology =
      VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  pipelineInputAssemblyStateCreateInfo.primitiveRestartEnable = VK_FALSE;

  VkViewport viewport{};
  viewport.x = 0.0f;
  viewport.y = 0.0f;
  viewport.width = static_cast<float>(swapChainExtent.width);
  viewport.height = static_cast<float>(swapChainExtent.height);
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;

  VkRect2D scissor{};
  scissor.offset = {0, 0};
  scissor.extent = swapChainExtent;

  VkPipelineViewportStateCreateInfo pipelineViewportStateCreateInfo{};
  pipelineViewportStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  pipelineViewportStateCreateInfo.scissorCount = 1;
  pipelineViewportStateCreateInfo.viewportCount = 1;
  pipelineViewportStateCreateInfo.pViewports = &viewport;

  VkPipelineRasterizationStateCreateInfo pipelineRasterizationStateCreateInfo{};
  pipelineRasterizationStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  pipelineRasterizationStateCreateInfo.depthClampEnable = VK_FALSE;
  pipelineRasterizationStateCreateInfo.rasterizerDiscardEnable = VK_FALSE;
  pipelineRasterizationStateCreateInfo.polygonMode = polygonMode;
  pipelineRasterizationStateCreateInfo.lineWidth = 1.0f;
  pipelineRasterizationStateCreateInfo.cullMode = VK_CULL_MODE_NONE;
  pipelineRasterizationStateCreateInfo.frontFace =
      VK_FRONT_FACE_COUNTER_CLOCKWISE;
  pipelineRasterizationStateCreateInfo.depthBiasEnable = VK_FALSE;
  pipelineRasterizationStateCreateInfo.depthBiasConstantFactor = 0.0f;
  pipelineRasterizationStateCreateInfo.depthBiasClamp = 0.0f;
  pipelineRasterizationStateCreateInfo.depthBiasSlopeFactor = 0.0f;

  VkPipelineMultisampleStateCreateInfo pipelineMultisampleStateCreateInfo{};
  pipelineMultisampleStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  pipelineMultisampleStateCreateInfo.sampleShadingEnable = VK_FALSE;
  pipelineMultisampleStateCreateInfo.rasterizationSamples =
      VK_SAMPLE_COUNT_1_BIT;
  pipelineMultisampleStateCreateInfo.minSampleShading = 1.0f;
  pipelineMultisampleStateCreateInfo.pSampleMask = nullptr;
  pipelineMultisampleStateCreateInfo.alphaToCoverageEnable = VK_FALSE;
  pipelineMultisampleStateCreateInfo.alphaToOneEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState colorBlendAttachment{};
  colorBlendAttachment.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
      VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  colorBlendAttachment.blendEnable = VK_FALSE;
  colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;  // Optional
  colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
  colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;             // Optional
  colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;  // Optional
  colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
  colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;             // Optional

  VkPipelineColorBlendStateCreateInfo pipelineColorBlendStateCreateInfo{};
  pipelineColorBlendStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  pipelineColorBlendStateCreateInfo.logicOpEnable = VK_FALSE;
  pipelineColorBlendStateCreateInfo.logicOp = VK_LOGIC_OP_COPY; // Optional
  pipelineColorBlendStateCreateInfo.attachmentCount = 1;
  pipelineColorBlendStateCreateInfo.pAttachments = &colorBlendAttachment;
  pipelineColorBlendStateCreateInfo.blendConstants[0] = 0.0f; // Optional
  pipelineColorBlendStateCreateInfo.blendConstants[1] = 0.0f; // Optional
  pipelineColorBlendStateCreateInfo.blendConstants[2] = 0.0f; // Optional
  pipelineColorBlendStateCreateInfo.blendConstants[3] = 0.0f; // Optional

  VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo{};
  pipelineLayoutCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipelineLayoutCreateInfo.setLayoutCount = pipelineLayouts.size();
  pipelineLayoutCreateInfo.pSetLayouts = pipelineLayouts.data();

  VkPipelineLayout pipelineLayout;
  if (vkCreatePipelineLayout(m_device, &pipelineLayoutCreateInfo, nullptr,
                             &pipelineLayout) != VK_SUCCESS) {
    throw std::runtime_error("failed to create pipeline layout");
  }

  VkPipelineDepthStencilStateCreateInfo pipelineDepthStencilStateCreateInfo{};
  pipelineDepthStencilStateCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  pipelineDepthStencilStateCreateInfo.depthTestEnable = VK_TRUE;
  pipelineDepthStencilStateCreateInfo.depthWriteEnable = VK_TRUE;
  pipelineDepthStencilStateCreateInfo.depthCompareOp = VK_COMPARE_OP_LESS;
  pipelineDepthStencilStateCreateInfo.depthBoundsTestEnable = VK_FALSE;
  pipelineDepthStencilStateCreateInfo.minDepthBounds = 0.0f;
  pipelineDepthStencilStateCreateInfo.maxDepthBounds = 1.0f;
  pipelineDepthStencilStateCreateInfo.stencilTestEnable = VK_FALSE;
  pipelineDepthStencilStateCreateInfo.front = {};
  pipelineDepthStencilStateCreateInfo.back = {};

  VkGraphicsPipelineCreateInfo graphicsPipelineCreateInfo{};
  graphicsPipelineCreateInfo.sType =
      VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  graphicsPipelineCreateInfo.stageCount = 2;
  graphicsPipelineCreateInfo.pVertexInputState =
      &vertexPipelineInputStateCreateInfo;
  graphicsPipelineCreateInfo.pInputAssemblyState =
      &pipelineInputAssemblyStateCreateInfo;
  graphicsPipelineCreateInfo.pViewportState = &pipelineViewportStateCreateInfo;
  graphicsPipelineCreateInfo.pRasterizationState =
      &pipelineRasterizationStateCreateInfo;
  graphicsPipelineCreateInfo.pMultisampleState =
      &pipelineMultisampleStateCreateInfo;
  graphicsPipelineCreateInfo.pDepthStencilState =
      &pipelineDepthStencilStateCreateInfo;
  graphicsPipelineCreateInfo.pStages = shaderStages;
  graphicsPipelineCreateInfo.pColorBlendState =
      &pipelineColorBlendStateCreateInfo;
  graphicsPipelineCreateInfo.pDynamicState =
      &dynamicStatepipelineDynamicStateCreateInfo;
  graphicsPipelineCreateInfo.layout = pipelineLayout;
  graphicsPipelineCreateInfo.renderPass = m_renderPass;
  graphicsPipelineCreateInfo.subpass = 0;
  graphicsPipelineCreateInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
  graphicsPipelineCreateInfo.basePipelineIndex = -1;              // Optional

  VkPipeline pipeline;
  if (vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1,
                                &graphicsPipelineCreateInfo, nullptr,
                                &pipeline) != VK_SUCCESS) {
    throw std::runtime_error("failed creating graphics pipeline");
  }

  vkDestroyShaderModule(m_device, vertShaderModule, nullptr);
  vkDestroyShaderModule(m_device, fragShaderModule, nullptr);

  return Pipeline(pipeline, pipelineLayout);
}

void Renderer::createFramebuffers() {
  m_swapChainFramebuffers.resize(m_swapChainImageViews.size());

  for (size_t i = 0; i < m_swapChainFramebuffers.size(); i++) {
    std::array<VkImageView, 2> attachments = {m_swapChainImageViews[i],
                                              m_depthImageView};
    VkFramebufferCreateInfo framebufferCreateInfo{};
    framebufferCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    framebufferCreateInfo.renderPass = m_renderPass;
    framebufferCreateInfo.attachmentCount =
        static_cast<uint32_t>(attachments.size());
    framebufferCreateInfo.pAttachments = attachments.data();
    framebufferCreateInfo.width = swapChainExtent.width;
    framebufferCreateInfo.height = swapChainExtent.height;
    framebufferCreateInfo.layers = 1;
    if (vkCreateFramebuffer(m_device, &framebufferCreateInfo, nullptr,
                            &m_swapChainFramebuffers[i]) != VK_SUCCESS) {
      throw std::runtime_error("failed to create  framebuffer");
    }
  }
}

void Renderer::createCommandPool() {
  QueueFamilyIndices queueFamilyIndices = findQueueFamilies(m_physicalDevice);

  VkCommandPoolCreateInfo commandPoolCreateInfo{};
  commandPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  commandPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  commandPoolCreateInfo.queueFamilyIndex =
      queueFamilyIndices.graphicsFamily.value();

  if (vkCreateCommandPool(m_device, &commandPoolCreateInfo, nullptr,
                          &m_commandPool) != VK_SUCCESS) {
    throw std::runtime_error("failed to create command pool");
  }
}

void Renderer::createDepthResources() {
  VkFormat depthFormat = findDepthFormat();
  createImage(swapChainExtent.width, swapChainExtent.height, depthFormat,
              VK_IMAGE_TILING_OPTIMAL,
              VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
              VMA_MEMORY_USAGE_AUTO, VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT,
              m_depthImage, m_depthAllocation);
  m_depthImageView =
      createImageView(m_depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);
}

VkFormat Renderer::findSupportedFormat(const std::vector<VkFormat> &candidates,
                                       const VkImageTiling &tiling,
                                       const VkFormatFeatureFlags &features) {
  for (const VkFormat &format : candidates) {
    VkFormatProperties properties;
    vkGetPhysicalDeviceFormatProperties(m_physicalDevice, format, &properties);
    if (tiling == VK_IMAGE_TILING_LINEAR &&
        (properties.linearTilingFeatures & features) == features) {
      return format;
    } else if (tiling == VK_IMAGE_TILING_OPTIMAL &&
               (properties.optimalTilingFeatures & features) == features) {
      return format;
    }
  }
  throw std::runtime_error("error to find supported format");
}

VkFormat Renderer::findDepthFormat() {
  return findSupportedFormat(
      {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT,
       VK_FORMAT_D24_UNORM_S8_UINT},
      VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
}

bool Renderer::hasStencilComponent(VkFormat format) {
  return format == VK_FORMAT_D32_SFLOAT_S8_UINT ||
         format == VK_FORMAT_D24_UNORM_S8_UINT;
}

VkImageView Renderer::createImageView(const VkImage &image,
                                      const VkFormat &format,
                                      const VkImageAspectFlags &aspectFlags) {
  VkImageViewCreateInfo imageViewCreateInfo{};
  imageViewCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  imageViewCreateInfo.image = image;
  imageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
  imageViewCreateInfo.format = format;
  imageViewCreateInfo.subresourceRange.aspectMask = aspectFlags;
  imageViewCreateInfo.subresourceRange.baseMipLevel = 0;
  imageViewCreateInfo.subresourceRange.levelCount = 1;
  imageViewCreateInfo.subresourceRange.baseArrayLayer = 0;
  imageViewCreateInfo.subresourceRange.layerCount = 1;

  VkImageView imageView;

  if (vkCreateImageView(m_device, &imageViewCreateInfo, nullptr, &imageView) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to create texture image view!");
  }
  return imageView;
}

VkCommandBuffer Renderer::beginSingleTimeCommands() {
  VkCommandBufferAllocateInfo commandBufferAllocateInfo{};
  commandBufferAllocateInfo.sType =
      VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  commandBufferAllocateInfo.commandPool = m_commandPool;
  commandBufferAllocateInfo.commandBufferCount = 1;

  VkCommandBuffer commandBuffer;
  vkAllocateCommandBuffers(m_device, &commandBufferAllocateInfo,
                           &commandBuffer);

  VkCommandBufferBeginInfo commandBufferBeginInfo{};
  commandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

  vkBeginCommandBuffer(commandBuffer, &commandBufferBeginInfo);
  return commandBuffer;
}

void Renderer::endSingleTimeCommands(VkCommandBuffer &commandBuffer) {
  vkEndCommandBuffer(commandBuffer);
  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &commandBuffer;
  vkQueueSubmit(m_queue, 1, &submitInfo, VK_NULL_HANDLE);
  vkQueueWaitIdle(m_queue);

  vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer);
}

void Renderer::copyBuffer(const VkBuffer &srcBuffer, const VkBuffer &dstBuffer,
                          const VkDeviceSize &size) {
  VkCommandBuffer commandBuffer = beginSingleTimeCommands();

  VkBufferCopy copyRegion{};
  copyRegion.size = size;

  vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);

  endSingleTimeCommands(commandBuffer);
}

void Renderer::transitionImageLayout(const VkImage &image,
                                     const VkFormat &format,
                                     const VkImageLayout &oldLayout,
                                     const VkImageLayout &newLayout) {
  VkCommandBuffer commandBuffer = beginSingleTimeCommands();

  VkImageMemoryBarrier imageMemoryBarrier{};
  imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  imageMemoryBarrier.oldLayout = oldLayout;
  imageMemoryBarrier.newLayout = newLayout;
  imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  imageMemoryBarrier.image = image;
  imageMemoryBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  imageMemoryBarrier.subresourceRange.baseMipLevel = 0;
  imageMemoryBarrier.subresourceRange.levelCount = 1;
  imageMemoryBarrier.subresourceRange.baseArrayLayer = 0;
  imageMemoryBarrier.subresourceRange.layerCount = 1;
  imageMemoryBarrier.srcAccessMask = 0;
  imageMemoryBarrier.dstAccessMask = 0;
  VkPipelineStageFlags sourcePipelineStageflags;
  VkPipelineStageFlags destPipelineStageFlags;

  if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED &&
      newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {

    imageMemoryBarrier.srcAccessMask = 0;
    imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

    sourcePipelineStageflags = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
    destPipelineStageFlags = VK_PIPELINE_STAGE_TRANSFER_BIT;

  } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL &&
             newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {

    imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    imageMemoryBarrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;

    sourcePipelineStageflags = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    destPipelineStageFlags = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;

  } else {
    throw std::invalid_argument("unsuported layout transition");
  }

  vkCmdPipelineBarrier(commandBuffer, sourcePipelineStageflags,
                       destPipelineStageFlags, 0, 0, nullptr, 0, nullptr, 1,
                       &imageMemoryBarrier);
  endSingleTimeCommands(commandBuffer);
}

void Renderer::copyBufferImage(const VkBuffer &buffer, const VkImage &image,
                               const uint32_t &width, const uint32_t &height) {
  VkCommandBuffer commandBuffer = beginSingleTimeCommands();
  VkBufferImageCopy regionBufferImageCopy{};
  regionBufferImageCopy.imageExtent.width = width;
  regionBufferImageCopy.imageExtent.height = height;
  regionBufferImageCopy.imageExtent.depth = 1;
  regionBufferImageCopy.bufferOffset = 0;
  regionBufferImageCopy.bufferRowLength = 0;
  regionBufferImageCopy.bufferImageHeight = 0;
  regionBufferImageCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  regionBufferImageCopy.imageSubresource.mipLevel = 0;
  regionBufferImageCopy.imageSubresource.baseArrayLayer = 0;
  regionBufferImageCopy.imageSubresource.layerCount = 1;
  regionBufferImageCopy.imageOffset = {0, 0, 0};
  vkCmdCopyBufferToImage(commandBuffer, buffer, image,
                         VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                         &regionBufferImageCopy);
  endSingleTimeCommands(commandBuffer);
}

void Renderer::createBuffer(const VkDeviceSize &size,
                            const VkBufferUsageFlags &usage,
                            const VmaMemoryUsage &memoryUsage,
                            const VmaAllocationCreateFlags &allocationFlags,
                            VkBuffer &buffer, VmaAllocation &allocation) {

  VkBufferCreateInfo bufferCreateInfo{};
  bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bufferCreateInfo.size = size;
  bufferCreateInfo.usage = usage;
  bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocationCreateInfo{};
  allocationCreateInfo.usage = memoryUsage;
  allocationCreateInfo.flags = allocationFlags;

  if (VK_SUCCESS != vmaCreateBuffer(m_vmaAllocator, &bufferCreateInfo,
                                    &allocationCreateInfo, &buffer, &allocation,
                                    nullptr)) {
    std::runtime_error("Error creating vma buffer");
  }
}

uint32_t Renderer::findMemoryType(const uint32_t &typeFilter,
                                  const VkMemoryPropertyFlags &properties) {
  VkPhysicalDeviceMemoryProperties memProperties;
  vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);

  for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
    if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags &
                                    properties) == properties) {
      return i;
    }
  }

  throw std::runtime_error("failed to find suitable memory type!");
}

void Renderer::createCommandBuffer() {
  m_commandBuffers.resize(m_imageCount);
  VkCommandBufferAllocateInfo commandBufferAllocInfo{};
  commandBufferAllocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  commandBufferAllocInfo.commandPool = m_commandPool;
  commandBufferAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  commandBufferAllocInfo.commandBufferCount = m_commandBuffers.size();

  if (vkAllocateCommandBuffers(m_device, &commandBufferAllocInfo,
                               m_commandBuffers.data()) != VK_SUCCESS) {
    throw std::runtime_error("failed to allocate command buffers");
  }
}

void Renderer::createWireframePipeline() {

  // VkVertexInputBindingDescription vertexInputBindingDescriptions;
  // VkVertexInputAttributeDescription vertexInputAttributeDescriptions;
  // VkPipeline pipeline = createGraphicPipeline(
  //     "./shaders/modelShader.vert", "./shaders/modelShader.frag",
  //     vertexInputBindingDescriptions, vertexInputAttributeDescriptions);
}

void Renderer::createSyncObjects() {
  m_imageAvailableSemaphores.resize(m_imageCount);
  m_renderFinishedSemaphores.resize(m_imageCount);
  m_inFlightFences.resize(m_imageCount);

  VkSemaphoreCreateInfo semaphoreCreateInfo{};
  semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

  VkFenceCreateInfo fenceCreateInfo{};
  fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fenceCreateInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

  for (size_t i = 0; i < m_imageCount; i++) {

    if (vkCreateSemaphore(m_device, &semaphoreCreateInfo, nullptr,
                          &m_imageAvailableSemaphores[i]) != VK_SUCCESS ||
        vkCreateSemaphore(m_device, &semaphoreCreateInfo, nullptr,
                          &m_renderFinishedSemaphores[i]) != VK_SUCCESS ||
        vkCreateFence(m_device, &fenceCreateInfo, nullptr,
                      &m_inFlightFences[i]) != VK_SUCCESS) {
      throw std::runtime_error("failed to create sync objects");
    }
  }
}

void Renderer::initImGui() {

  VkDescriptorPoolSize pool_sizes[] = {
      {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}};

  VkDescriptorPoolCreateInfo pool_info = {};
  pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
  pool_info.maxSets = 1000;
  pool_info.poolSizeCount = (uint32_t)std::size(pool_sizes);
  pool_info.pPoolSizes = pool_sizes;

  vkCreateDescriptorPool(m_device, &pool_info, nullptr, &m_imguiDescriptorPool);
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableGamepad; // Enable Gamepad Controls
  // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // IF using
  // Docking Branch

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForVulkan(m_window, true);
  ImGui_ImplVulkan_InitInfo init_info = {};
  init_info.Instance = m_instance;
  init_info.PhysicalDevice = m_physicalDevice;
  init_info.Device = m_device;
  init_info.QueueFamily = m_queueFamilyIndices.graphicsFamily.value();
  init_info.Queue = m_queue;
  // init_info.PipelineCache = YOUR_PIPELINE_CACHE;
  init_info.DescriptorPool = m_imguiDescriptorPool;
  init_info.MinImageCount = m_imageCount;
  init_info.ImageCount = m_imageCount;
  init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
  init_info.CheckVkResultFn = nullptr;

  init_info.RenderPass = m_renderPass;

  ImGui_ImplVulkan_Init(&init_info);
}

void Renderer::drawScene() {

  vkWaitForFences(m_device, 1, &m_inFlightFences[m_currentFrame], VK_TRUE,
                  UINT64_MAX);
  vkResetFences(m_device, 1, &m_inFlightFences[m_currentFrame]);

  uint32_t imageIndex = 0;
  VkResult result = vkAcquireNextImageKHR(
      m_device, m_swapchain, UINT64_MAX,
      m_imageAvailableSemaphores[m_currentFrame], VK_NULL_HANDLE, &imageIndex);

  if (result == VK_ERROR_OUT_OF_DATE_KHR) {
    m_framebufferResized = false;
    recreateSwapChain();
    return;
  } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
    throw std::runtime_error("failed to acquire swap chain image!");
  }

  vkResetCommandBuffer(m_commandBuffers[m_currentFrame], 0);

  VkCommandBuffer commandBuffer = m_commandBuffers[m_currentFrame];
  VkCommandBufferBeginInfo commandBufferBeginInfo{};
  commandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  commandBufferBeginInfo.flags = 0;                  // Optional
  commandBufferBeginInfo.pInheritanceInfo = nullptr; // Optional

  if (vkBeginCommandBuffer(commandBuffer, &commandBufferBeginInfo) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to begin recording command buffer");
  }
  VkPipelineStageFlags srcPipelineStageflags =
      VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
  VkPipelineStageFlags dstPipelineStageflags =
      VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
  VkDependencyFlags dependencyFlags = 0;
  uint32_t memoryBarrierCount = 1;
  VkMemoryBarrier memoryBarrier{};
  memoryBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
  memoryBarrier.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  memoryBarrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  uint32_t bufferMemoryBarrierCount = 0;
  VkBufferMemoryBarrier bufferMemoryBarrier{};
  uint32_t imageMemoryBarrierCount = 0;
  VkImageMemoryBarrier imageMemoryBarrier;
  vkCmdPipelineBarrier(commandBuffer, srcPipelineStageflags,
                       dstPipelineStageflags, dependencyFlags,
                       memoryBarrierCount, &memoryBarrier,
                       bufferMemoryBarrierCount, &bufferMemoryBarrier,
                       imageMemoryBarrierCount, &imageMemoryBarrier);
  VkRenderPassBeginInfo renderPassBeginInfo{};
  renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  renderPassBeginInfo.renderPass = m_renderPass;
  renderPassBeginInfo.framebuffer = m_swapChainFramebuffers[imageIndex];
  renderPassBeginInfo.renderArea.offset = {0, 0};
  renderPassBeginInfo.renderArea.extent = swapChainExtent;

  std::array<VkClearValue, 2> clearValues{};
  clearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
  clearValues[1].depthStencil = {1.0f, 0};

  renderPassBeginInfo.pClearValues = clearValues.data();
  renderPassBeginInfo.clearValueCount =
      static_cast<uint32_t>(clearValues.size());

  vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo,
                       VK_SUBPASS_CONTENTS_INLINE);
  for (const std::shared_ptr<Drawing> &drawing : m_toDraw) {
    drawing->bind(commandBuffer, m_currentFrame, swapChainExtent);
    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(swapChainExtent.width);
    viewport.height = static_cast<float>(swapChainExtent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = swapChainExtent;
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    vkCmdDrawIndexed(commandBuffer, drawing->numberOfVertices, 1, 0, 0, 0);
  }

  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();

  ImGui::NewFrame();
  // ImGui::ShowDemoWindow();
  for (const std::string *value : m_guiLabels) {
    ImGui::LabelText("label", "%s", "hello wordl");
  }
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Open")) {

        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey",
                                                "Choose File", ".gltf", config);
      }

      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Simulation")) {
      bool isSimulationStopped = m_isSimulationStoped;
      if (!isSimulationStopped) {
        ImGui::BeginDisabled();
      }
      if (ImGui::MenuItem("Resume Simulation")) {
        m_resumeSimulation();
        m_isSimulationStoped = false;
      }
      if (!isSimulationStopped) {
        ImGui::EndDisabled();
      }
      if (isSimulationStopped) {
        ImGui::BeginDisabled();
      }
      if (ImGui::MenuItem("Stop Simulation")) {
        m_stopSimulation();
        m_isSimulationStoped = true;
      }
      if (isSimulationStopped) {
        ImGui::EndDisabled();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  if (ImGuiFileDialog::Instance()->Display(
          "ChooseFileDlgKey")) {               // => will show a dialog
    if (ImGuiFileDialog::Instance()->IsOk()) { // action if OK
      std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
      std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
      std::cout << "loading: " << filePath << filePathName << std::endl;
      m_loadMeshEvent(filePathName);
    }

    ImGuiFileDialog::Instance()->Close();
  }
  ImGui::Render();
  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);

  vkCmdEndRenderPass(commandBuffer);
  if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
    throw std::runtime_error("failed to record command buffer");
  }

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

  VkPipelineStageFlags waitStages[] = {
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
  submitInfo.waitSemaphoreCount = 1;
  submitInfo.pWaitSemaphores = &m_imageAvailableSemaphores[m_currentFrame];
  submitInfo.pWaitDstStageMask = waitStages;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &m_commandBuffers[m_currentFrame];
  submitInfo.signalSemaphoreCount = 1;
  submitInfo.pSignalSemaphores = &m_renderFinishedSemaphores[m_currentFrame];

  if (vkQueueSubmit(m_queue, 1, &submitInfo,
                    m_inFlightFences[m_currentFrame]) != VK_SUCCESS) {
    throw std::runtime_error("failed to submit draw command buffer!");
  }
  VkPresentInfoKHR presentInfo{};
  presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  presentInfo.waitSemaphoreCount = 1;
  presentInfo.pWaitSemaphores = &m_renderFinishedSemaphores[m_currentFrame];

  VkSwapchainKHR swapchains[] = {m_swapchain};
  presentInfo.swapchainCount = 1;
  presentInfo.pSwapchains = swapchains;
  presentInfo.pImageIndices = &imageIndex;
  presentInfo.pResults = nullptr;

  result = vkQueuePresentKHR(m_queue, &presentInfo);
  if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR ||
      m_framebufferResized) {
    m_framebufferResized = false;
    recreateSwapChain();
  } else if (result != VK_SUCCESS) {
    throw std::runtime_error("failed to present swap chain image!");
  }
  m_currentFrame = (m_currentFrame + 1) % m_imageCount;
}
void Renderer::createLogicalDevice() {

  m_queueFamilyIndices = findQueueFamilies(m_physicalDevice);

  std::vector<VkDeviceQueueCreateInfo> deviceQueueCreateInfos;
  std::set<uint32_t> uniqueQueueFamilies = {
      m_queueFamilyIndices.graphicsFamily.value(),
      m_queueFamilyIndices.presentFamily.value()};

  float queuePriority = 1.0f;
  for (uint32_t queueFamily : uniqueQueueFamilies) {
    VkDeviceQueueCreateInfo deviceQueueCreateInfo{};
    deviceQueueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    deviceQueueCreateInfo.queueFamilyIndex = queueFamily;
    deviceQueueCreateInfo.queueCount = 1;
    deviceQueueCreateInfo.pQueuePriorities = &queuePriority;
    deviceQueueCreateInfos.push_back(deviceQueueCreateInfo);
  }

  VkPhysicalDeviceFeatures physicalDeviceCreateInfo{};
  physicalDeviceCreateInfo.samplerAnisotropy = VK_TRUE;
  physicalDeviceCreateInfo.fillModeNonSolid = VK_TRUE;

  VkDeviceCreateInfo deviceCreateinfo{};
  deviceCreateinfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  deviceCreateinfo.queueCreateInfoCount =
      static_cast<uint32_t>(deviceQueueCreateInfos.size());
  deviceCreateinfo.pQueueCreateInfos = deviceQueueCreateInfos.data();

  deviceCreateinfo.pEnabledFeatures = &physicalDeviceCreateInfo;

  if (enableValidationLayers) {
    deviceCreateinfo.enabledLayerCount =
        static_cast<uint32_t>(validationLayers.size());
    deviceCreateinfo.ppEnabledLayerNames = validationLayers.data();
  } else {
    deviceCreateinfo.enabledLayerCount = 0;
  }

  deviceCreateinfo.enabledExtensionCount =
      static_cast<uint32_t>(m_deviceExtensions.size());
  deviceCreateinfo.ppEnabledExtensionNames = m_deviceExtensions.data();

  if (vkCreateDevice(m_physicalDevice, &deviceCreateinfo, nullptr, &m_device) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to create logica device");
  }
  vkGetDeviceQueue(m_device, m_queueFamilyIndices.graphicsFamily.value(), 0,
                   &m_queue);
  vkGetDeviceQueue(m_device, m_queueFamilyIndices.presentFamily.value(), 0,
                   &m_presentQueue);
}

void Renderer::createVMA() {
  VmaVulkanFunctions vulkanFunctions{};
  vulkanFunctions.vkGetInstanceProcAddr = &vkGetInstanceProcAddr;
  vulkanFunctions.vkGetDeviceProcAddr = &vkGetDeviceProcAddr;

  VmaAllocatorCreateInfo allocatorCreateInfo{};
  allocatorCreateInfo.instance = m_instance;
  allocatorCreateInfo.physicalDevice = m_physicalDevice;
  allocatorCreateInfo.device = m_device;
  allocatorCreateInfo.vulkanApiVersion = VK_API_VERSION_1_0;
  allocatorCreateInfo.pVulkanFunctions = &vulkanFunctions;

  vmaCreateAllocator(&allocatorCreateInfo, &m_vmaAllocator);
}

bool Renderer::shouldExit() { return glfwWindowShouldClose(m_window); }

void Renderer::cleanupSwapChain() {
  for (auto &framebuffer : m_swapChainFramebuffers) {
    vkDestroyFramebuffer(m_device, framebuffer, nullptr);
  }
  for (auto &imageView : m_swapChainImageViews) {
    vkDestroyImageView(m_device, imageView, nullptr);
  }

  vkDestroySwapchainKHR(m_device, m_swapchain, nullptr);
}

void Renderer::recreateSwapChain() {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(m_window, &width, &height);
  while (width == 0 && height == 0) {
    glfwGetFramebufferSize(m_window, &width, &height);
    glfwWaitEvents();
  }
  vkDeviceWaitIdle(m_device);

  cleanupSwapChain();
  createSwapChain();
  createImageViews();
  createDepthResources();
  createFramebuffers();
}

void Renderer::cleanup() {
  vkDeviceWaitIdle(m_device);
  for (const auto &vertexBuffer : m_vertexBuffers) {
    vmaDestroyBuffer(m_vmaAllocator, vertexBuffer->buffer,
                     vertexBuffer->allocation);
  }
  for (const auto &vertexBuffer : m_indexBuffers) {
    vmaDestroyBuffer(m_vmaAllocator, vertexBuffer->buffer,
                     vertexBuffer->allocation);
  }
  vkDestroyDescriptorPool(m_device, m_imguiDescriptorPool, nullptr);
  for (const auto &drawing : m_drawings) {
    vkDestroyDescriptorPool(m_device, *drawing->mvpDescriptorPool, nullptr);
  }
  for (const auto &imageDescriptorPool : m_imageDescriptorPools) {
    vkDestroyDescriptorPool(m_device, imageDescriptorPool, nullptr);
  }

  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  vkDestroyImageView(m_device, m_depthImageView, nullptr);

  vmaDestroyImage(m_vmaAllocator, m_depthImage, m_depthAllocation);

  cleanupSwapChain();

  vkDestroyDescriptorSetLayout(m_device, m_textureDescriptorSetLayout, nullptr);
  vkDestroyDescriptorSetLayout(m_device, m_solidColorDescriptorSetLayout,
                               nullptr);

  for (size_t i = 0; i < m_imageCount; i++) {
    vkDestroySemaphore(m_device, m_renderFinishedSemaphores[i], nullptr);
    vkDestroySemaphore(m_device, m_imageAvailableSemaphores[i], nullptr);
    vkDestroyFence(m_device, m_inFlightFences[i], nullptr);
  }

  vkDestroyCommandPool(m_device, m_commandPool, nullptr);
  vmaDestroyAllocator(m_vmaAllocator);
  vkDestroyDevice(m_device, nullptr);
  vkDestroySurfaceKHR(m_instance, m_surface, nullptr);

  if (enableValidationLayers) {
    DestroyDebugUtilsMessengerEXT(m_instance, m_debugMessenger, nullptr);
  }

  vkDestroyInstance(m_instance, nullptr);

  glfwDestroyWindow(m_window);
}

VKAPI_ATTR VkBool32 VKAPI_CALL Renderer::debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
    void *pUserData) {
  std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
  return VK_FALSE;
}

void Renderer::createImage(const uint32_t &width, const uint32_t &height,
                           const VkFormat &format, const VkImageTiling &tiling,
                           const VkImageUsageFlags &usage,
                           const VmaMemoryUsage &memoryUsage,
                           const VmaAllocationCreateFlags &allocationFlags,
                           VkImage &image, VmaAllocation &allocation) {
  VkImageCreateInfo imageCreateInfo{};
  imageCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
  imageCreateInfo.extent.width = width;
  imageCreateInfo.extent.height = height;
  imageCreateInfo.extent.depth = 1;
  imageCreateInfo.mipLevels = 1;
  imageCreateInfo.arrayLayers = 1;
  imageCreateInfo.format = format;
  imageCreateInfo.tiling = tiling;
  imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  imageCreateInfo.usage = usage;
  imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
  imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocationCreateInfo{};
  allocationCreateInfo.usage = memoryUsage;
  allocationCreateInfo.flags = allocationFlags;

  vmaCreateImage(m_vmaAllocator, &imageCreateInfo, &allocationCreateInfo,
                 &image, &allocation, nullptr);
}

VkResult CreateDebugUtilsMessengerEXT(
    const VkInstance &instance,
    const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
    const VkAllocationCallbacks *pAllocator,
    VkDebugUtilsMessengerEXT *pDebugMessenger) {
  auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
      instance, "vkCreateDebugUtilsMessengerEXT");
  if (func != nullptr) {
    return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
  } else {
    return VK_ERROR_EXTENSION_NOT_PRESENT;
  }
}

void DestroyDebugUtilsMessengerEXT(
    const VkInstance &instance, const VkDebugUtilsMessengerEXT &debugMessenger,
    const VkAllocationCallbacks *pAllocator) {
  auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
      instance, "vkDestroyDebugUtilsMessengerEXT");
  if (func != nullptr) {
    func(instance, debugMessenger, pAllocator);
  }
}

Pipeline::Pipeline(const VkPipeline &pipeline,
                   const VkPipelineLayout &pipelineLayout) {
  m_pipeline = pipeline;
  m_pipelineLayout = pipelineLayout;
}

VkPipeline Pipeline::getPipeline() { return m_pipeline; }
VkPipelineLayout Pipeline::getPipelineLayout() { return m_pipelineLayout; }

Drawing::Drawing(
    const std::shared_ptr<Pipeline> &pipeline, const int numberOfVertices,
    const std::vector<std::shared_ptr<VulkanBuffer>> &vertexVulkanBuffers,
    const std::shared_ptr<VulkanBuffer> &indexVulkanBuffer,
    const std::shared_ptr<VkDescriptorPool> &mvpDescriptorPool,
    const std::vector<std::shared_ptr<VkDescriptorSet>> &mvpDescriptorSet,
    const std::vector<std::shared_ptr<VulkanBuffer>> mvpBuffers)
    : pipeline(pipeline), numberOfVertices(numberOfVertices),
      vertexVulkanBuffers(vertexVulkanBuffers),
      indexVulkanBuffer(indexVulkanBuffer),
      mvpDescriptorPool(mvpDescriptorPool), mvpDescriptorSet(mvpDescriptorSet),
      mvpBuffers(mvpBuffers) {}

void Drawing::setTranform(const glm::mat4 &transform) {
  m_tranform = transform;
}
glm::mat4 Drawing::getTranform() const { return m_tranform; }
TexturedDrawing::TexturedDrawing(
    const std::shared_ptr<Pipeline> &pipeline, const int numberOfVertices,
    const std::vector<std::shared_ptr<VulkanBuffer>> &vertexVulkanBuffers,
    const std::shared_ptr<VulkanBuffer> &indexVulkanBuffer,
    const std::shared_ptr<VkDescriptorPool> &mvpDescriptorPool,
    const std::vector<std::shared_ptr<VkDescriptorSet>> &mvpDescriptorSet,
    const std::vector<std::shared_ptr<VulkanBuffer>> mvpBuffers,
    const std::shared_ptr<VkDescriptorSet> &textureDescriptorSet)
    : Drawing(pipeline, numberOfVertices, vertexVulkanBuffers,
              indexVulkanBuffer, mvpDescriptorPool, mvpDescriptorSet,
              mvpBuffers),
      textureDescriptorSet(textureDescriptorSet) {}

void TexturedDrawing::bind(const VkCommandBuffer &cmd, const int currentImage,
                           const VkExtent2D &swapChainExtent) const {
  vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                    pipeline->getPipeline());
  std::vector<VkBuffer> vertexBuffers;
  std::vector<VkDeviceSize> sizes;
  for (const auto &buffer : vertexVulkanBuffers) {
    vertexBuffers.push_back(buffer->buffer);
    sizes.push_back(0);
  }
  vkCmdBindVertexBuffers(cmd, 0, vertexBuffers.size(), vertexBuffers.data(),
                         sizes.data());
  vkCmdBindIndexBuffer(cmd, indexVulkanBuffer->buffer, 0, VK_INDEX_TYPE_UINT16);

  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                          pipeline->getPipelineLayout(), 1, 1,
                          &*textureDescriptorSet, 0, nullptr);

  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                          pipeline->getPipelineLayout(), 0, 1,
                          &*mvpDescriptorSet.at(currentImage), 0, nullptr);
  UniformBufferObject mvpObject;
  mvpObject.model = getTranform();
  mvpObject.view =
      glm::lookAt(glm::vec3(2.0f, 2.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f),
                  glm::vec3(0.0f, 0.0f, 1.0f));
  mvpObject.projection = glm::perspective(
      glm::radians(45.0f),
      swapChainExtent.width / (float)swapChainExtent.height, 0.1f, 10.0f);
  mvpObject.projection[1][1] *= -1.0f;
  UniformBufferObject *mvpBufferPtr = static_cast<UniformBufferObject *>(
      mvpBuffers.at(currentImage)->bufferPointer);
  *mvpBufferPtr = mvpObject;
}

std::shared_ptr<Drawing> Renderer::loadModel(const Engine::MeshNode &meshNode) {

  // color texture descriptor
  std::vector<std::shared_ptr<Drawing>> drawings;
  std::shared_ptr<VkDescriptorSet> imageDescriptorSetPtr;

  VkDescriptorPoolSize sceneImageDescriptorSizes{};
  sceneImageDescriptorSizes.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  sceneImageDescriptorSizes.descriptorCount = 1;

  VkDescriptorPoolCreateInfo sceneImagesDescriptorPoolCreateInfo{};
  sceneImagesDescriptorPoolCreateInfo.sType =
      VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  sceneImagesDescriptorPoolCreateInfo.poolSizeCount = 1;
  sceneImagesDescriptorPoolCreateInfo.pPoolSizes = &sceneImageDescriptorSizes;
  sceneImagesDescriptorPoolCreateInfo.maxSets = 1;
  VkDescriptorPool sceneImagesDescriptorPool;
  vkCreateDescriptorPool(m_device, &sceneImagesDescriptorPoolCreateInfo,
                         nullptr, &sceneImagesDescriptorPool);
  m_imageDescriptorPools.push_back(sceneImagesDescriptorPool);

  std::vector<VkVertexInputBindingDescription> vertexInputBindingDescriptions;
  std::vector<VkVertexInputAttributeDescription>
      vertexInputAttributeDescriptions;
  std::vector<VkVertexInputAttributeDescription>
      onlyPosVertexInputAttributeDescriptions;

  VkBuffer stagingColorImageBuffer;
  VmaAllocation stagingColorImageAllocation;

  createBuffer(meshNode.colorTexture->data.size(),
               VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO,
               VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                   VMA_ALLOCATION_CREATE_MAPPED_BIT,
               stagingColorImageBuffer, stagingColorImageAllocation);

  vmaCopyMemoryToAllocation(m_vmaAllocator, meshNode.colorTexture->data.data(),
                            stagingColorImageAllocation, 0,
                            meshNode.colorTexture->data.size());

  VkFormat imageFormat;
  VkImage outputImage;
  VmaAllocation outputAllocation;
  switch (meshNode.colorTexture->numOfComponents) {
  case 3: {
    imageFormat = VK_FORMAT_R8G8B8_SRGB;
    break;
  }
  case 4: {
    imageFormat = VK_FORMAT_R8G8B8A8_SRGB;
    break;
  }
  default:
    throw std::runtime_error("number of components not supported");
    break;
  }
  createImage(meshNode.colorTexture->width, meshNode.colorTexture->height,
              imageFormat, VK_IMAGE_TILING_OPTIMAL,
              VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
              VMA_MEMORY_USAGE_AUTO, 0, outputImage, outputAllocation);

  transitionImageLayout(outputImage, imageFormat, VK_IMAGE_LAYOUT_UNDEFINED,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

  copyBufferImage(stagingColorImageBuffer, outputImage,
                  static_cast<uint32_t>(meshNode.colorTexture->width),
                  static_cast<uint32_t>(meshNode.colorTexture->height));

  transitionImageLayout(outputImage, imageFormat,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  vmaDestroyBuffer(m_vmaAllocator, stagingColorImageBuffer,
                   stagingColorImageAllocation);

  VkImageView outputImageView =
      createImageView(outputImage, imageFormat, VK_IMAGE_ASPECT_COLOR_BIT);

  VkSamplerCreateInfo samplerCreateinfo{};
  samplerCreateinfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  samplerCreateinfo.magFilter = VK_FILTER_LINEAR;
  samplerCreateinfo.minFilter = VK_FILTER_LINEAR;
  samplerCreateinfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  samplerCreateinfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  samplerCreateinfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;

  VkPhysicalDeviceProperties propertiesphysicalDeviceProperties{};
  vkGetPhysicalDeviceProperties(m_physicalDevice,
                                &propertiesphysicalDeviceProperties);

  samplerCreateinfo.anisotropyEnable = VK_TRUE;
  samplerCreateinfo.maxAnisotropy =
      propertiesphysicalDeviceProperties.limits.maxSamplerAnisotropy;
  samplerCreateinfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  samplerCreateinfo.unnormalizedCoordinates = VK_FALSE;
  samplerCreateinfo.compareOp = VK_COMPARE_OP_ALWAYS;
  samplerCreateinfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  samplerCreateinfo.mipLodBias = 0.0f;
  samplerCreateinfo.minLod = 0.0f;
  samplerCreateinfo.maxLod = 0.0f;

  VkSampler outputSampler;
  if (vkCreateSampler(m_device, &samplerCreateinfo, nullptr, &outputSampler) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to create texture sampler");
  }
  VkDescriptorSetAllocateInfo imageDescriptorAllocateInfo{};
  imageDescriptorAllocateInfo.sType =
      VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  imageDescriptorAllocateInfo.descriptorPool = sceneImagesDescriptorPool;
  imageDescriptorAllocateInfo.descriptorSetCount = 1;
  imageDescriptorAllocateInfo.pSetLayouts = &m_textureDescriptorSetLayout;
  VkDescriptorSet imageDescriptorSet;
  if (vkAllocateDescriptorSets(m_device, &imageDescriptorAllocateInfo,
                               &imageDescriptorSet) != VK_SUCCESS) {
    throw std::runtime_error("Error creating texture descriptor sets");
  }
  imageDescriptorSetPtr = std::make_shared<VkDescriptorSet>(imageDescriptorSet);

  m_imageDescriptorSetPtr.push_back(imageDescriptorSetPtr);
  VkDescriptorImageInfo descriptorImageInfo{};
  descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  descriptorImageInfo.imageView = outputImageView;
  descriptorImageInfo.sampler = outputSampler;

  VkWriteDescriptorSet writeDescriptorSet{};
  writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  writeDescriptorSet.dstSet = imageDescriptorSet;
  writeDescriptorSet.dstBinding = 0;
  writeDescriptorSet.dstArrayElement = 0;
  writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  writeDescriptorSet.descriptorCount = 1;
  writeDescriptorSet.pImageInfo = &descriptorImageInfo;
  vkUpdateDescriptorSets(m_device, 1, &writeDescriptorSet, 0, nullptr);

  m_loadedTextures.push_back(Texture(outputImage, outputImageView,
                                     outputAllocation, outputSampler,
                                     imageDescriptorSet));

  // index buffer
  std::vector<VulkanBuffer> vertexVulkanBuffers;
  std::shared_ptr<VulkanBuffer> indexVulkanBufferPtr;
  m_indexBuffers.push_back(indexVulkanBufferPtr);
  VkDeviceSize bufferSize = meshNode.meshData.indices.size() * 2;
  VkBuffer outputIndexBuffer;
  VmaAllocation outputIndexAllocation;

  VkBuffer stagingBuffer;
  VmaAllocation stagingAllocation;
  createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
               VMA_MEMORY_USAGE_AUTO,
               VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                   VMA_ALLOCATION_CREATE_MAPPED_BIT,
               stagingBuffer, stagingAllocation);

  void *data;

  vmaMapMemory(m_vmaAllocator, stagingAllocation, &data);
  memcpy(data, meshNode.meshData.indices.data(), (size_t)bufferSize);
  vmaUnmapMemory(m_vmaAllocator, stagingAllocation);

  createBuffer(
      bufferSize,
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
      VMA_MEMORY_USAGE_AUTO, 0, outputIndexBuffer, outputIndexAllocation);

  copyBuffer(stagingBuffer, outputIndexBuffer, bufferSize);

  vmaDestroyBuffer(m_vmaAllocator, stagingBuffer, stagingAllocation);

  indexVulkanBufferPtr = std::make_shared<VulkanBuffer>(
      outputIndexBuffer, outputIndexAllocation, data);
  // vertexBuffers
  struct VertexBufferData {
    const std::vector<float> &buffer;
    int numberOfComponets;
    int locationAndBinding;
  };
  std::array<VertexBufferData, 4> vertexBuffersData = {
      VertexBufferData{.buffer = meshNode.meshData.position,
                       .numberOfComponets = 3,
                       .locationAndBinding = 0},
      VertexBufferData{.buffer = meshNode.meshData.normal,
                       .numberOfComponets = 3,
                       .locationAndBinding = 1},
      VertexBufferData{.buffer = meshNode.meshData.texCoord,
                       .numberOfComponets = 2,
                       .locationAndBinding = 2},
      VertexBufferData{.buffer = meshNode.meshData.tangent,
                       .numberOfComponets = 3,
                       .locationAndBinding = 3}};
  std::vector<std::shared_ptr<VulkanBuffer>> vertexBuffers;
  for (const VertexBufferData &vertexBufferData : vertexBuffersData) {
    if (vertexBufferData.buffer.size() != 0) {
      const std::vector<float> buffer = vertexBufferData.buffer;
      VkDeviceSize bufferSize = buffer.size() * 4;
      VkBuffer vertexBuffer;
      VmaAllocation vertexAllocation;

      VkBuffer stagingBuffer;
      VmaAllocation stagingAllocation;
      createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                   VMA_MEMORY_USAGE_AUTO,
                   VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                       VMA_ALLOCATION_CREATE_MAPPED_BIT,
                   stagingBuffer, stagingAllocation);

      void *data;

      vmaMapMemory(m_vmaAllocator, stagingAllocation, &data);
      memcpy(data, buffer.data(), (size_t)bufferSize);
      vmaUnmapMemory(m_vmaAllocator, stagingAllocation);

      createBuffer(bufferSize,
                   VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                       VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                   VMA_MEMORY_USAGE_AUTO, 0, vertexBuffer, vertexAllocation);

      copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

      vmaDestroyBuffer(m_vmaAllocator, stagingBuffer, stagingAllocation);
      std::shared_ptr<VulkanBuffer> vertexBufferPtr =
          std::make_shared<VulkanBuffer>(vertexBuffer, vertexAllocation, data);
      vertexBuffers.push_back(vertexBufferPtr);
      m_vertexBuffers.push_back(vertexBufferPtr);

      VkVertexInputBindingDescription vertexInputBindingDescription{};
      vertexInputBindingDescription.binding =
          vertexBufferData.locationAndBinding;
      vertexInputBindingDescription.stride =
          sizeof(float) * vertexBufferData.numberOfComponets;
      vertexInputBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
      vertexInputBindingDescriptions.push_back(vertexInputBindingDescription);

      VkVertexInputAttributeDescription vertexInputAttributeDescription{};
      vertexInputAttributeDescription.binding =
          vertexBufferData.locationAndBinding;
      vertexInputAttributeDescription.offset = 0;
      switch (vertexBufferData.numberOfComponets) {
      case 3: {
        vertexInputAttributeDescription.format = VK_FORMAT_R32G32B32_SFLOAT;
        break;
      }
      case 2: {
        vertexInputAttributeDescription.format = VK_FORMAT_R32G32_SFLOAT;
        break;
      }
      }
      vertexInputAttributeDescription.location =
          vertexBufferData.locationAndBinding;
      vertexInputAttributeDescriptions.push_back(
          vertexInputAttributeDescription);
      if (vertexBufferData.locationAndBinding == 0) {
        onlyPosVertexInputAttributeDescriptions.push_back(
            vertexInputAttributeDescription);
      }
    }
  }
  std::vector<VkDescriptorSetLayout> texturedLayouts = {
      m_mvpDescriptorSetLayout, m_textureDescriptorSetLayout};
  // pipelines
  Pipeline pipelineTextured = createGraphicPipeline(
      "./shaders/" + std::string(TEXTURED_VERTEX_SHADER_NAME),
      "./shaders/" + std::string(TEXTURED_FRAGMENT_SHADER_NAME),
      vertexInputBindingDescriptions, vertexInputAttributeDescriptions,
      VK_POLYGON_MODE_FILL, VK_CULL_MODE_BACK_BIT, texturedLayouts);

  std::vector<VkDescriptorSetLayout> solidColorLayouts = {
      m_mvpDescriptorSetLayout, m_solidColorDescriptorSetLayout};

  Pipeline pipelineSolid = createGraphicPipeline(
      "./shaders/" + std::string(SOLID_VERTEX_SHADER_NAME),
      "./shaders/" + std::string(SOLID_FRAGMENT_SHADER_NAME),
      vertexInputBindingDescriptions, onlyPosVertexInputAttributeDescriptions,
      VK_POLYGON_MODE_LINE, VK_CULL_MODE_NONE, solidColorLayouts);
  std::shared_ptr<Pipeline> pipelineTexturedPtr =
      std::make_shared<Pipeline>(pipelineTextured);
  std::shared_ptr<Pipeline> pipelineSolidPtr =
      std::make_shared<Pipeline>(pipelineSolid);
  m_pipelines.push_back(pipelineTexturedPtr);
  m_pipelines.push_back(pipelineSolidPtr);

  // mvp desctriptors
  VkDescriptorPoolSize mvpDescriptorSizes{};
  mvpDescriptorSizes.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  mvpDescriptorSizes.descriptorCount = m_imageCount;

  VkDescriptorPoolCreateInfo mvpDescriptorPoolCreateInfo{};
  mvpDescriptorPoolCreateInfo.sType =
      VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  mvpDescriptorPoolCreateInfo.poolSizeCount = 1;
  mvpDescriptorPoolCreateInfo.pPoolSizes = &mvpDescriptorSizes;
  mvpDescriptorPoolCreateInfo.maxSets = m_imageCount;

  VkDescriptorPool mvpDescriptorPool;
  vkCreateDescriptorPool(m_device, &mvpDescriptorPoolCreateInfo, nullptr,
                         &mvpDescriptorPool);
  std::vector<std::shared_ptr<VkDescriptorSet>> mvpDescriptorSets;
  mvpDescriptorSets.reserve(m_imageCount);
  std::vector<std::shared_ptr<VulkanBuffer>> mvpBuffers;
  for (size_t i = 0; i < m_imageCount; ++i) {
    VkDescriptorSetAllocateInfo descriptorSetAllocateInfo{};
    descriptorSetAllocateInfo.sType =
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    descriptorSetAllocateInfo.descriptorPool = mvpDescriptorPool;
    descriptorSetAllocateInfo.descriptorSetCount = 1;
    descriptorSetAllocateInfo.pSetLayouts = &m_mvpDescriptorSetLayout;
    VkDescriptorSet mvpDescriptorSet;
    if (vkAllocateDescriptorSets(m_device, &descriptorSetAllocateInfo,
                                 &mvpDescriptorSet) != VK_SUCCESS) {
      throw std::runtime_error("Error allocating ubo descriptor set");
    }
    std::shared_ptr<VkDescriptorSet> mvpDescriptorSetPtr =
        std::make_shared<VkDescriptorSet>(mvpDescriptorSet);
    m_mvpDescriptorSets.push_back(mvpDescriptorSetPtr);
    mvpDescriptorSets.push_back(mvpDescriptorSetPtr);
    VkDeviceSize mvpBufferSize = sizeof(UniformBufferObject);
    VkBuffer mvpBuffer;
    VmaAllocation mvpAllocation;
    createBuffer(mvpBufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                 VMA_MEMORY_USAGE_AUTO,
                 VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                     VMA_ALLOCATION_CREATE_MAPPED_BIT,
                 mvpBuffer, mvpAllocation);
    void *mvpBufferPointer;

    if (vmaMapMemory(m_vmaAllocator, mvpAllocation, &mvpBufferPointer) !=
        VK_SUCCESS) {
      throw std::runtime_error(
          "Error mapping uniform buffer on start allocation!");
    }

    std::shared_ptr<VulkanBuffer> mvpVulkanBuffer =
        std::make_shared<VulkanBuffer>(mvpBuffer, mvpAllocation,
                                       mvpBufferPointer);
    m_mvpVulkanBuffers.push_back(mvpVulkanBuffer);
    mvpBuffers.push_back(mvpVulkanBuffer);
    VkDescriptorBufferInfo mvpDescriptorBufferInfo{};
    mvpDescriptorBufferInfo.buffer = mvpBuffer;
    mvpDescriptorBufferInfo.offset = 0;
    mvpDescriptorBufferInfo.range = mvpBufferSize;

    VkWriteDescriptorSet mvpWriteDescriptor{};
    mvpWriteDescriptor.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    mvpWriteDescriptor.dstSet = *mvpDescriptorSets.at(i);
    mvpWriteDescriptor.dstBinding = 0;
    mvpWriteDescriptor.dstArrayElement = 0;
    mvpWriteDescriptor.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    mvpWriteDescriptor.descriptorCount = 1;
    mvpWriteDescriptor.pBufferInfo = &mvpDescriptorBufferInfo;
    vkUpdateDescriptorSets(m_device, 1, &mvpWriteDescriptor, 0, nullptr);
  }

  const int numberOfVertices = meshNode.meshData.indices.size();
  auto texture = std::make_shared<TexturedDrawing>(
      std::make_shared<Pipeline>(pipelineTextured), numberOfVertices,
      vertexBuffers, indexVulkanBufferPtr,
      std::make_shared<VkDescriptorPool>(mvpDescriptorPool), mvpDescriptorSets,
      mvpBuffers, imageDescriptorSetPtr);
  return texture;
}
Drawble::Drawble(const BufferData &bufferData, const Pipeline &pipelineTextured,
                 const Pipeline &pipelineSolid, const uint32_t indexOffset,
                 const uint32_t count, const VkIndexType &indexType,
                 const uint32_t &indexToTexture,
                 const uint32_t &indexToDescriptor)
    : m_bufferData(bufferData), m_pipelineTextured(pipelineTextured),
      m_pipelineSolid(pipelineSolid), m_indexOffset(indexOffset),
      m_count(count), m_indexType(indexType), m_indexToTexture(indexToTexture),
      m_indexToDescriptor(indexToDescriptor) {}

Pipeline Drawble::getPipelineSolid() { return m_pipelineSolid; }
Pipeline Drawble::getPipelineTextured() { return m_pipelineTextured; }
Texture::Texture(const VkImage &image, const VkImageView &imageView,
                 const VmaAllocation &allocation, const VkSampler &sampler,
                 const VkDescriptorSet &descriptorSet)
    : m_image(image), m_imageView(imageView), m_allocation(allocation),
      m_sampler(sampler), m_descriptorSet(descriptorSet) {}

void Texture::clean(const VkDevice &device,
                    const VmaAllocator &allocator) const {
  vmaDestroyImage(allocator, m_image, m_allocation);
  vkDestroySampler(device, m_sampler, nullptr);
}

MemoryData::MemoryData(const VkBuffer &buffer, const VmaAllocation &allocation)
    : m_buffer(buffer), m_allocation(allocation) {}

Node::Node(const std::string &name, const std::vector<Drawble> &drawbles,
           const glm::mat4 &initialMatrix)
    : m_matrix(initialMatrix), m_name(name), m_drawbles(drawbles) {}

void Renderer::createDescriptorSetLayouts() {

  {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    VkDescriptorSetLayoutCreateInfo layoutCreateInfo{};
    layoutCreateInfo.sType =
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutCreateInfo.pBindings = &binding;
    layoutCreateInfo.bindingCount = 1;

    vkCreateDescriptorSetLayout(m_device, &layoutCreateInfo, nullptr,
                                &m_mvpDescriptorSetLayout);
  }
  {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    binding.descriptorCount = 1;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;

    VkDescriptorSetLayoutCreateInfo layoutCreateInfo{};
    layoutCreateInfo.sType =
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutCreateInfo.pBindings = &binding;
    layoutCreateInfo.bindingCount = 1;

    vkCreateDescriptorSetLayout(m_device, &layoutCreateInfo, nullptr,
                                &m_textureDescriptorSetLayout);
  }
  {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    binding.descriptorCount = 1;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;

    VkDescriptorSetLayoutCreateInfo layoutCreateInfo{};
    layoutCreateInfo.sType =
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutCreateInfo.pBindings = &binding;
    layoutCreateInfo.bindingCount = 1;

    vkCreateDescriptorSetLayout(m_device, &layoutCreateInfo, nullptr,
                                &m_solidColorDescriptorSetLayout);
  }
}

void Renderer::allocateTextureDescriptorSet(
    std::vector<std::shared_ptr<Engine::Image>> images) {
  m_imageDescriptorPools.reserve(images.size() + m_imageDescriptorPools.size());

  VkDescriptorPoolSize sceneImageDescriptorSizes{};
  sceneImageDescriptorSizes.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  sceneImageDescriptorSizes.descriptorCount = images.size() * m_imageCount;

  VkDescriptorPoolCreateInfo sceneImagesDescriptorPool{};
  sceneImagesDescriptorPool.sType =
      VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  sceneImagesDescriptorPool.poolSizeCount = 1;
  sceneImagesDescriptorPool.pPoolSizes = &sceneImageDescriptorSizes;
  sceneImagesDescriptorPool.maxSets = images.size() * m_imageCount;
  VkDescriptorPool imageSceneDescriptorPool;
  vkCreateDescriptorPool(m_device, &sceneImagesDescriptorPool, nullptr,
                         &imageSceneDescriptorPool);
  m_imageDescriptorPools.push_back(imageSceneDescriptorPool);

  for (auto currentImage : images) {

    VkBuffer stagingColorImageBuffer;
    VmaAllocation stagingColorImageAllocation;

    createBuffer(currentImage->data.size(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VMA_MEMORY_USAGE_AUTO,
                 VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                     VMA_ALLOCATION_CREATE_MAPPED_BIT,
                 stagingColorImageBuffer, stagingColorImageAllocation);

    vmaCopyMemoryToAllocation(m_vmaAllocator, currentImage->data.data(),
                              stagingColorImageAllocation, 0,
                              currentImage->data.size());

    VkFormat imageFormat;
    VkImage outputImage;
    VmaAllocation outputAllocation;
    switch (currentImage->numOfComponents) {
    case 3: {
      imageFormat = VK_FORMAT_R8G8B8_SRGB;
      break;
    }
    case 4: {
      imageFormat = VK_FORMAT_R8G8B8A8_SRGB;
      break;
    }
    default:
      throw std::runtime_error("number of components not supported");
      break;
    }
    createImage(currentImage->width, currentImage->height, imageFormat,
                VK_IMAGE_TILING_OPTIMAL,
                VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VMA_MEMORY_USAGE_AUTO, 0, outputImage, outputAllocation);

    transitionImageLayout(outputImage, imageFormat, VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

    copyBufferImage(stagingColorImageBuffer, outputImage,
                    static_cast<uint32_t>(currentImage->width),
                    static_cast<uint32_t>(currentImage->height));

    transitionImageLayout(outputImage, imageFormat,
                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    vmaDestroyBuffer(m_vmaAllocator, stagingColorImageBuffer,
                     stagingColorImageAllocation);

    VkImageView outputImageView =
        createImageView(outputImage, imageFormat, VK_IMAGE_ASPECT_COLOR_BIT);

    VkSamplerCreateInfo samplerCreateinfo{};
    samplerCreateinfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerCreateinfo.magFilter = VK_FILTER_LINEAR;
    samplerCreateinfo.minFilter = VK_FILTER_LINEAR;
    samplerCreateinfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerCreateinfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerCreateinfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;

    VkPhysicalDeviceProperties propertiesphysicalDeviceProperties{};
    vkGetPhysicalDeviceProperties(m_physicalDevice,
                                  &propertiesphysicalDeviceProperties);

    samplerCreateinfo.anisotropyEnable = VK_TRUE;
    samplerCreateinfo.maxAnisotropy =
        propertiesphysicalDeviceProperties.limits.maxSamplerAnisotropy;
    samplerCreateinfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    samplerCreateinfo.unnormalizedCoordinates = VK_FALSE;
    samplerCreateinfo.compareOp = VK_COMPARE_OP_ALWAYS;
    samplerCreateinfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerCreateinfo.mipLodBias = 0.0f;
    samplerCreateinfo.minLod = 0.0f;
    samplerCreateinfo.maxLod = 0.0f;

    VkSampler outputSampler;
    if (vkCreateSampler(m_device, &samplerCreateinfo, nullptr,
                        &outputSampler) != VK_SUCCESS) {
      throw std::runtime_error("failed to create texture sampler");
    }
    VkDescriptorSetAllocateInfo imageDescriptorAllocateInfo{};
    imageDescriptorAllocateInfo.sType =
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    imageDescriptorAllocateInfo.descriptorPool = imageSceneDescriptorPool;
    imageDescriptorAllocateInfo.descriptorSetCount = 1;
    imageDescriptorAllocateInfo.pSetLayouts = &m_textureDescriptorSetLayout;
    VkDescriptorSet imageDescriptorSet;
    if (vkAllocateDescriptorSets(m_device, &imageDescriptorAllocateInfo,
                                 &imageDescriptorSet) != VK_SUCCESS) {
      throw std::runtime_error("Error creating texture descriptor sets");
    }
    m_imageDescriptorSetPtr.push_back(
        std::make_shared<VkDescriptorSet>(imageDescriptorSet));

    VkDescriptorImageInfo descriptorImageInfo{};
    descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    descriptorImageInfo.imageView = outputImageView;
    descriptorImageInfo.sampler = outputSampler;

    VkWriteDescriptorSet writeDescriptorSet{};
    writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet.dstSet = imageDescriptorSet;
    writeDescriptorSet.dstBinding = 0;
    writeDescriptorSet.dstArrayElement = 0;
    writeDescriptorSet.descriptorType =
        VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.pImageInfo = &descriptorImageInfo;
    vkUpdateDescriptorSets(m_device, 1, &writeDescriptorSet, 0, nullptr);

    m_loadedTextures.push_back(Texture(outputImage, outputImageView,
                                       outputAllocation, outputSampler,
                                       imageDescriptorSet));
  }
}

RenderObject Renderer::loadMeshBuffers(
    std::vector<std::shared_ptr<Engine::MeshNode>> nodes) {
  std::vector<std::weak_ptr<VulkanBuffer>> vulkanBuffers;
  for (const std::shared_ptr<Engine::MeshNode> &node : nodes) {
    Engine::MeshData meshData = node->meshData;
    VkDeviceSize bufferSize = meshData.indices.size() / 2;
    VkBuffer indexBuffer;
    VmaAllocation indexAllocation;

    VkBuffer stagingBuffer;
    VmaAllocation stagingAllocation;
    createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VMA_MEMORY_USAGE_AUTO,
                 VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                     VMA_ALLOCATION_CREATE_MAPPED_BIT,
                 stagingBuffer, stagingAllocation);

    void *data;

    vmaMapMemory(m_vmaAllocator, stagingAllocation, &data);
    memcpy(data, meshData.indices.data(), (size_t)bufferSize);
    vmaUnmapMemory(m_vmaAllocator, stagingAllocation);

    createBuffer(bufferSize,
                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                     VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                 VMA_MEMORY_USAGE_AUTO, 0, indexBuffer, indexAllocation);

    copyBuffer(stagingBuffer, indexBuffer, bufferSize);

    vmaDestroyBuffer(m_vmaAllocator, stagingBuffer, stagingAllocation);
    VulkanBuffer indexVulkanBuffer(indexBuffer, indexAllocation, &data);
    m_indexBuffers.push_back(std::make_shared<VulkanBuffer>(indexVulkanBuffer));
    std::vector<VkVertexInputBindingDescription> vertexInputBindingDescriptions;
    std::vector<VkVertexInputAttributeDescription>
        vertexInputAttributeDescriptions;
    std::vector<VkVertexInputAttributeDescription>
        onlyPosVertexInputAttributeDescriptions;
    struct VertexBufferData {
      const std::vector<float> &buffer;
      int numberOfComponets;
    };
    std::array<VertexBufferData, 4> vertexBuffers = {
        VertexBufferData{.buffer = meshData.position, .numberOfComponets = 3},
        VertexBufferData{.buffer = meshData.normal, .numberOfComponets = 3},
        VertexBufferData{.buffer = meshData.texCoord, .numberOfComponets = 2},
        VertexBufferData{.buffer = meshData.tangent, .numberOfComponets = 3}};
    for (const VertexBufferData &vertexBuffer : vertexBuffers) {
      if (vertexBuffer.buffer.size() != 0) {
        const std::vector<float> buffer = vertexBuffer.buffer;
        VkDeviceSize bufferSize = buffer.size() * 4;
        VkBuffer outputBuffer;
        VmaAllocation outputAllocation;

        VkBuffer stagingBuffer;
        VmaAllocation stagingAllocation;
        createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                     VMA_MEMORY_USAGE_AUTO,
                     VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                         VMA_ALLOCATION_CREATE_MAPPED_BIT,
                     stagingBuffer, stagingAllocation);

        void *data;

        vmaMapMemory(m_vmaAllocator, stagingAllocation, &data);
        memcpy(data, buffer.data(), (size_t)bufferSize);
        vmaUnmapMemory(m_vmaAllocator, stagingAllocation);

        createBuffer(bufferSize,
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                     VMA_MEMORY_USAGE_AUTO, 0, outputBuffer, outputAllocation);

        copyBuffer(stagingBuffer, outputBuffer, bufferSize);

        vmaDestroyBuffer(m_vmaAllocator, stagingBuffer, stagingAllocation);

        std::shared_ptr<VulkanBuffer> vertexVulkanBuffers =
            std::make_shared<VulkanBuffer>(outputBuffer, outputAllocation,
                                           data);
        vulkanBuffers.push_back(vertexVulkanBuffers);
        m_vertexBuffers.push_back(vertexVulkanBuffers);

        VkVertexInputBindingDescription vertexInputBindingDescription{};
        vertexInputBindingDescription.binding = 0;
        vertexInputBindingDescription.stride = sizeof(float) * 3;
        vertexInputBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        vertexInputBindingDescriptions.push_back(vertexInputBindingDescription);

        VkVertexInputAttributeDescription vertexInputAttributeDescription{};
        vertexInputAttributeDescription.binding = 0;
        vertexInputAttributeDescription.offset = 0;
        switch (vertexBuffer.numberOfComponets) {
        case 3: {
          vertexInputAttributeDescription.format = VK_FORMAT_R32G32B32_SFLOAT;
          break;
        }
        case 2: {
          vertexInputAttributeDescription.format = VK_FORMAT_R32G32_SFLOAT;
          break;
        }
        }
        vertexInputAttributeDescription.location = 0;
        vertexInputAttributeDescriptions.push_back(
            vertexInputAttributeDescription);
        onlyPosVertexInputAttributeDescriptions.push_back(
            vertexInputAttributeDescription);
      }
    }
  }
}

VkDescriptorSet
Renderer::allocateTextureDescriptorSet(const VkImageView &imageview,
                                       const VkSampler &sampler) {
  // VkDescriptorSet descriptorSet;
  // std::vector<VkWriteDescriptorSet> writes(m_imageCount);
  // for (size_t frameIndex = 0; frameIndex < m_imageCount; frameIndex++) {
  //   VkDescriptorSetAllocateInfo descriptorSetAllocateInfo{};
  //   descriptorSetAllocateInfo.sType =
  //       VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  //   descriptorSetAllocateInfo.descriptorPool = m_mvpDescriptorPool;
  //   descriptorSetAllocateInfo.descriptorSetCount = 1;
  //   descriptorSetAllocateInfo.pSetLayouts = &m_textureDescriptorSetLayout;
  //
  //   if (vkAllocateDescriptorSets(m_device, &descriptorSetAllocateInfo,
  //                                &descriptorSet) != VK_SUCCESS) {
  //     throw std::runtime_error("Error creating texture descriptor sets");
  //   }
  //   VkDescriptorImageInfo descriptorImageInfo{};
  //   descriptorImageInfo.imageLayout =
  //   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; descriptorImageInfo.imageView =
  //   imageview; descriptorImageInfo.sampler = sampler;
  //
  //   VkWriteDescriptorSet writeDescriptorSet{};
  //   writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  //   writeDescriptorSet.dstSet = descriptorSet;
  //   writeDescriptorSet.dstBinding = 0;
  //   writeDescriptorSet.dstArrayElement = 0;
  //   writeDescriptorSet.descriptorType =
  //       VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  //   writeDescriptorSet.descriptorCount = 1;
  //   writeDescriptorSet.pImageInfo = &descriptorImageInfo;
  //   writes.at(frameIndex) = writeDescriptorSet;
  // }
  // vkUpdateDescriptorSets(m_device, writes.size(), writes.data(), 0, nullptr);
  // return descriptorSet;
}

RenderObject::RenderObject(const std::vector<std::shared_ptr<Node>> &nodes,
                           const tinygltf::Model &model)
    : m_nodes(nodes), m_model(model) {}
void RenderObject::setMatrix(const glm::mat4 &matrix, const uint32_t index) {
  m_nodes.at(0)->setMatrix(matrix);
}

glm::mat4 Node::getMatrix() const { return m_matrix; }

void Renderer::draw(const std::shared_ptr<Drawing> &drawing) {
  m_toDraw.push_back(drawing);
}
std::string Node::getName() const { return m_name; }

void Renderer::drawLabel(const std::string *label) {
  m_guiLabels.push_back(label);
}

void Node::setMatrix(const glm::mat4 &matrix) { m_matrix = matrix; }

void Node::setName(const std::string &name) { m_name = name; }

std::vector<Drawble> Node::getDrawbles() const { return m_drawbles; }

ColoredNode::ColoredNode(const std::shared_ptr<Node> &node,
                         const glm::vec3 &color)
    : Node(node->getName(), node->getDrawbles(), node->getMatrix()) {
  m_color = color;
}
glm::vec3 ColoredNode::getColor() const { return m_color; }
void Renderer::endFrame() {
  drawScene();
  m_toDraw.clear();
  m_guiLabels.clear();
  glfwPollEvents();
}

glm::vec3 RenderObject::getCenterOfMass(const bool &verbose) const {
  tinygltf::Model model = m_model;
  tinygltf::Buffer buffer = model.buffers.at(0);
  tinygltf::Node node = model.nodes.at(0);
  tinygltf::Mesh mesh = model.meshes.at(node.mesh);
  tinygltf::Primitive primitive = mesh.primitives.at(0);
  tinygltf::Accessor indicesAccessor = model.accessors.at(primitive.indices);
  tinygltf::BufferView indicesBufferView =
      model.bufferViews.at(indicesAccessor.bufferView);

  size_t numOfVertices = indicesAccessor.count;

  uint16_t *meshIndices = reinterpret_cast<uint16_t *>(
      buffer.data.data() + indicesAccessor.byteOffset +
      indicesBufferView.byteOffset);

  tinygltf::Accessor positionAccessor =
      model.accessors.at(primitive.attributes["POSITION"]);
  tinygltf::BufferView positionBufferView =
      model.bufferViews.at(positionAccessor.bufferView);
  float *vertices = reinterpret_cast<float *>(buffer.data.data() +
                                              positionAccessor.byteOffset +
                                              positionBufferView.byteOffset);

  glm::vec3 centerOfMass = glm::vec3(0.0f);
  float totalArea = 0.0f;

  if (verbose) {
    std::cout << "numOfTriangles: " << numOfVertices << std::endl;
  }
  for (size_t vertexIndex = 0; vertexIndex < numOfVertices; vertexIndex += 3) {
    if (verbose) {
      std::cout << "vertexIndex: " << vertexIndex << std::endl;
      std::cout << "indices" << std::endl;
      std::cout << meshIndices[vertexIndex] << std::endl;
      std::cout << meshIndices[vertexIndex + 1] << std::endl;
      std::cout << meshIndices[vertexIndex + 2] << std::endl;
    }

    float x1 = vertices[meshIndices[vertexIndex] * 3];
    float y1 = vertices[meshIndices[vertexIndex] * 3 + 1];
    float z1 = vertices[meshIndices[vertexIndex] * 3 + 2];
    glm::vec3 vertex1 = glm::vec3(x1, y1, z1);
    if (verbose) {
      std::cout << "vec1" << std::endl;
      std::cout << x1 << " " << y1 << " " << z1 << std::endl;
    }
    float x2 = vertices[meshIndices[vertexIndex + 1] * 3];
    float y2 = vertices[meshIndices[vertexIndex + 1] * 3 + 1];
    float z2 = vertices[meshIndices[vertexIndex + 1] * 3 + 2];
    glm::vec3 vertex2 = glm::vec3(x2, y2, z2);
    if (verbose) {
      std::cout << "vec2" << std::endl;
      std::cout << x2 << " " << y2 << " " << z2 << std::endl;
    }
    float x3 = vertices[meshIndices[vertexIndex + 2] * 3];
    float y3 = vertices[meshIndices[vertexIndex + 2] * 3 + 1];
    float z3 = vertices[meshIndices[vertexIndex + 2] * 3 + 2];
    glm::vec3 vertex3 = glm::vec3(x3, y3, z3);
    if (verbose) {
      std::cout << "vec3" << std::endl;
      std::cout << x3 << " " << y3 << " " << z3 << std::endl;
    }

    glm::vec3 triangleCentroid = (vertex1 + vertex2 + vertex3) / 3.0f;

    if (verbose) {
      std::cout << "centroid" << std::endl;
      std::cout << triangleCentroid.x << " " << triangleCentroid.y << " "
                << triangleCentroid.z << std::endl;
    }
    glm::vec3 a = vertex2 - vertex1;
    glm::vec3 b = vertex3 - vertex1;
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

glm::mat4 RenderObject::getInertiaTensor(const bool &verbose) const {
  //
  // tinygltf::Model model = m_model;
  // tinygltf::Buffer buffer = model.buffers.at(0);
  // tinygltf::Node node = model.nodes.at(0);
  // tinygltf::Mesh mesh = model.meshes.at(node.mesh);
  // tinygltf::Primitive primitive = mesh.primitives.at(0);
  // tinygltf::Accessor indicesAccessor = model.accessors.at(primitive.indices);
  // tinygltf::BufferView indicesBufferView =
  //     model.bufferViews.at(indicesAccessor.bufferView);
  //
  // size_t numOfVertices = indicesAccessor.count;
  //
  // uint16_t *meshIndices = reinterpret_cast<uint16_t *>(
  //     buffer.data.data() + indicesAccessor.byteOffset +
  //     indicesBufferView.byteOffset);
  //
  // tinygltf::Accessor positionAccessor =
  //     model.accessors.at(primitive.attributes["POSITION"]);
  // tinygltf::BufferView positionBufferView =
  //     model.bufferViews.at(positionAccessor.bufferView);
  // float *vertices = reinterpret_cast<float *>(buffer.data.data() +
  //                                             positionAccessor.byteOffset +
  //                                             positionBufferView.byteOffset);
  //
  // glm::mat3 acumulatedInertialTensor = glm::mat3(0.0f);
  //
  // uint32_t numsOfTetrahedrons = 0;
  // for (size_t vertexIndex = 0; vertexIndex < numOfVertices; vertexIndex += 3)
  // {
  //
  //   glm::vec3 vertex1 = glm::vec3(0.0f);
  //   float x2 = vertices[meshIndices[vertexIndex] * 3];
  //   float y2 = vertices[meshIndices[vertexIndex] * 3 + 1];
  //   float z2 = vertices[meshIndices[vertexIndex] * 3 + 2];
  //   glm::vec3 vertex2 = glm::vec3(x2, y2, z2);
  //   float x3 = vertices[meshIndices[vertexIndex + 1] * 3];
  //   float y3 = vertices[meshIndices[vertexIndex + 1] * 3 + 1];
  //   float z3 = vertices[meshIndices[vertexIndex + 1] * 3 + 2];
  //   glm::vec3 vertex3 = glm::vec3(x3, y3, z3);
  //   float x4 = vertices[meshIndices[vertexIndex + 2] * 3];
  //   float y4 = vertices[meshIndices[vertexIndex + 2] * 3 + 1];
  //   float z4 = vertices[meshIndices[vertexIndex + 2] * 3 + 2];
  //   glm::vec3 vertex4 = glm::vec3(x4, y4, z4);
  //
  //   glm::mat3 jacobian = glm::mat3(vertex2, vertex3, vertex4);
  //   float det = glm::determinant(jacobian);
  //
  //   if (verbose) {
  //     std::cout << "First Vertex: " << std::endl;
  //     std::cout << vertex2.x << "\t" << vertex2.y << "\t" << vertex2.z
  //               << std::endl;
  //     std::cout << "Second Vertex: " << std::endl;
  //     std::cout << vertex3.x << "\t" << vertex3.y << "\t" << vertex3.z
  //               << std::endl;
  //     std::cout << "Third Vertex: " << std::endl;
  //     std::cout << vertex4.x << "\t" << vertex4.y << "\t" << vertex4.z
  //               << std::endl;
  //
  //     std::cout << "Jacobian:" << std::endl;
  //     std::cout << jacobian[0][0] << "\t" << jacobian[0][1] << "\t"
  //               << jacobian[0][2] << "\t" << std::endl;
  //
  //     std::cout << jacobian[1][0] << "\t" << jacobian[1][1] << "\t"
  //               << jacobian[1][2] << "\t" << std::endl;
  //
  //     std::cout << jacobian[2][0] << "\t" << jacobian[2][1] << "\t"
  //               << jacobian[2][2] << "\t" << std::endl;
  //     std::cout << "Determinant of the jacobian : " << det << std::endl;
  //   }
  //
  //   float abcx = vertex1.x * vertex1.x + vertex1.x * vertex2.x +
  //                vertex2.x * vertex2.x + vertex1.x * vertex3.x +
  //                vertex2.x * vertex3.x + vertex3.x * vertex3.x +
  //                vertex1.x * vertex4.x + vertex2.x * vertex4.x +
  //                vertex3.x * vertex4.x + vertex4.x * vertex4.x;
  //
  //   float abcy = vertex1.y * vertex1.y + vertex1.y * vertex2.y +
  //                vertex2.y * vertex2.y + vertex1.y * vertex3.y +
  //                vertex2.y * vertex3.y + vertex3.y * vertex3.y +
  //                vertex1.y * vertex4.y + vertex2.y * vertex4.y +
  //                vertex3.y * vertex4.y + vertex4.y * vertex4.y;
  //
  //   float abcz = vertex1.z * vertex1.z + vertex1.z * vertex2.z +
  //                vertex2.z * vertex2.z + vertex1.z * vertex3.z +
  //                vertex2.z * vertex3.z + vertex3.z * vertex3.z +
  //                vertex1.z * vertex4.z + vertex2.z * vertex4.z +
  //                vertex3.z * vertex4.z + vertex4.z * vertex4.z;
  //   float a = (det * (abcy + abcz) / 60.0f) / (det / 6.0);
  //   float b = (det * (abcx + abcz) / 60.0f) / (det / 6.0);
  //   float c = (det * (abcx + abcy) / 60.0f) / (det / 6.0);
  //
  //   float abcxp =
  //       vertex2.y * vertex1.z + vertex3.y * vertex1.z + vertex4.y * vertex1.z
  //       + vertex1.y * vertex2.z + vertex3.y * vertex2.z + vertex4.y *
  //       vertex2.z + vertex1.y * vertex3.z + vertex2.y * vertex3.z + vertex4.y
  //       * vertex3.z + vertex1.y * vertex4.z + vertex2.y * vertex4.z +
  //       vertex3.y * vertex4.z + 2.0f * vertex1.y * vertex1.z + 2.0f *
  //       vertex2.y * vertex2.z + 2.0f * vertex3.y * vertex3.z + 2.0f *
  //       vertex4.y * vertex4.z;
  //
  //   float abcyp =
  //       vertex2.x * vertex1.z + vertex3.x * vertex1.z + vertex4.x * vertex1.z
  //       + vertex1.x * vertex2.z + vertex3.x * vertex2.z + vertex4.x *
  //       vertex2.z + vertex1.x * vertex3.z + vertex2.x * vertex3.z + vertex4.x
  //       * vertex3.z + vertex1.x * vertex4.z + vertex2.x * vertex4.z +
  //       vertex3.x * vertex4.z + 2.0f * vertex1.x * vertex1.z + 2.0f *
  //       vertex2.x * vertex2.z + 2.0f * vertex3.x * vertex3.z + 2.0f *
  //       vertex4.x * vertex4.z;
  //
  //   float abczp =
  //       vertex2.x * vertex1.y + vertex3.x * vertex1.y + vertex4.x * vertex1.y
  //       + vertex1.x * vertex2.y + vertex3.x * vertex2.y + vertex4.x *
  //       vertex2.y + vertex1.x * vertex3.y + vertex2.x * vertex3.y + vertex4.x
  //       * vertex3.y + vertex1.x * vertex4.y + vertex2.x * vertex4.y +
  //       vertex3.x * vertex4.y + 2.0f * vertex1.x * vertex1.y + 2.0f *
  //       vertex2.x * vertex2.y + 2.0f * vertex3.x * vertex3.y + 2.0f *
  //       vertex4.x * vertex4.y;
  //
  //   float ap = (det * abcxp / 120.0f) / (det / 6.0);
  //   float bp = (det * abcyp / 120.0f) / (det / 6.0);
  //   float cp = (det * abczp / 120.0f) / (det / 6.0);
  //
  //   float tetrahedronInertiaTensorArray[] = {a,   -bp, -cp, -bp, b,
  //                                            -ap, -cp, -ap, c};
  //   glm::mat3 tetrahedronInertiaTensor =
  //       glm::make_mat3(tetrahedronInertiaTensorArray);
  //   if (verbose) {
  //
  //     std::cout << "Acumulated tetrahedron inertial tensor:" << std::endl;
  //     std::cout << tetrahedronInertiaTensor[0][0] << "\t"
  //               << tetrahedronInertiaTensor[0][1] << "\t"
  //               << tetrahedronInertiaTensor[0][2] << "\t" << std::endl;
  //
  //     std::cout << tetrahedronInertiaTensor[1][0] << "\t"
  //               << tetrahedronInertiaTensor[1][1] << "\t"
  //               << tetrahedronInertiaTensor[1][2] << "\t" << std::endl;
  //
  //     std::cout << tetrahedronInertiaTensor[2][0] << "\t"
  //               << tetrahedronInertiaTensor[2][1] << "\t"
  //               << tetrahedronInertiaTensor[2][2] << "\t" << std::endl;
  //   }
  //   acumulatedInertialTensor += tetrahedronInertiaTensor;
  //   numsOfTetrahedrons++;
  //   std::cout << "Number of tetrahedrons: " << numsOfTetrahedrons <<
  //   std::endl;
  // }
  // return acumulatedInertialTensor / static_cast<float>(numsOfTetrahedrons);
}
} // namespace Renderer
