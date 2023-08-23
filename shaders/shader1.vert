#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor; // 新增颜色顶点属性

uniform mat4 view;
uniform mat4 projection;

flat out vec3 Color; // 输出颜色 GRAPH COLORING 4 COLORING PROBLEM

void main()
{
    gl_Position = projection * view * vec4(aPos, 1.0);
    Color = aColor;
}