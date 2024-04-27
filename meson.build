project('vulkan-project','c','cpp')

cmake = import('cmake')

cc = meson.get_compiler('c')

glfw_subproject = cmake.subproject('glfw')
glm_subproject = cmake.subproject('glm')


glfw_dependency = glfw_subproject.dependency('glfw')
glm_dependency = glm_subproject.dependency('glm')
vulkan_dependency = dependency('vulkan')

glfw_include_dir = include_directories('./subprojects/glfw/include')
glm_include_dir = include_directories('./subprojects/glm')
stb_image_include_dir = include_directories('./include_stb')


subdir('shaders')
subdir('textures')


executable('vulkanProgram',['main.cpp',vertex_shader_custom_target,fragment_shader_custom_target],
    dependencies:[glfw_dependency,glm_dependency,vulkan_dependency],
    include_directories : [glfw_include_dir,glm_include_dir],
    override_options : ['cpp_std=c++20'])
