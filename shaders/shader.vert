#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 view;
uniform mat4 projection;

uniform vec3 lightDir;
uniform vec3 viewPos;

out vec4 color;

void main()
{
    vec3 diffuseColor = vec3(0.8f, 0.8f, 0.8f);
    vec3 ambientColor = vec3(0.3f, 0.3f, 0.3f); 
    vec3 lightColor = vec3(0.8f, 0.8f, 0.8f);
    
    float specularStrength = 0.5f;
    float specularExponent = 32.0f;

    vec3 FragPos = vec3(view * vec4(aPos, 1.0));
    vec3 Normal = aNormal;
    vec3 viewDir = normalize(viewPos - FragPos);

    // Ambient
    vec3 ambient = ambientColor * diffuseColor;

    // Diffuse
    float diff = max(dot(lightDir, Normal), 0.0);
    vec3 diffuse = diff * diffuseColor * lightColor;

    // Specular
    vec3 reflectDir = reflect(-lightDir, Normal);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), specularExponent);
    vec3 specular = specularStrength * spec * lightColor;  

    vec3 colorCalc = ambient + diffuse + specular;
    color = vec4(colorCalc, 1.0f);

    gl_Position = projection * view * vec4(aPos, 1.0);
}