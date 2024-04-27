
vertex_shader_custom_target  = custom_target(
    'vertexShader',
    input:'shader.vert',
    output:'vert.spv',
    command:['glslang','-V','-o','@OUTPUT@','-gVS','@INPUT@']
)
   
fragment_shader_custom_target =  custom_target(
    'fragmentShader',
    input:'shader.frag',
    output:'frag.spv',
    command:['glslang','-V','-o','@OUTPUT@','-gVS','@INPUT@'])
