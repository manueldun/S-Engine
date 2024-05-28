project('vulkan-project','c','cpp')

cmake = import('cmake')

glfw_subproject = cmake.subproject('glfw')
glm_subproject = cmake.subproject('glm')
vma_subproject = cmake.subproject('VulkanMemoryAllocator')

glfw_dependency = glfw_subproject.dependency('glfw')
glm_dependency = glm_subproject.dependency('glm')
vma_dependency = vma_subproject.dependency('VulkanMemoryAllocator')
vulkan_dependency = dependency('vulkan')

subdir('shaders')
subdir('textures')


executable('vulkanProgram',['main.cpp','Renderer.h','Renderer.cpp','stb_image.h','stb_image_write.h','json.hpp','tiny_gltf.h',vertex_shader_custom_target,fragment_shader_custom_target],
    dependencies:[glfw_dependency,glm_dependency,vulkan_dependency],
    override_options : ['cpp_std=c++20'])
