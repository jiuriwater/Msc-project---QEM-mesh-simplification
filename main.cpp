#include "QEM.h"
#include "Draw.h"

#include "libraries/glm/glm/gtc/matrix_transform.hpp"
#include "libraries/glm/glm/gtc/quaternion.hpp"

#include "libraries/imgui/imgui.h"
#include "libraries/imgui/imgui_impl_opengl3.h"
#include "libraries/imgui/imgui_impl_glfw.h"

#include <random>

glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
float cameraSpeed = 0.1f;
bool firstMouse = true;

float yaw = -90.0f;
float pitch = 0.0f;
float sensitivity = 0.2f;

bool isWireframe = false;
bool isColor = false;
bool isNormal = false;

float Rate = 0.01f;
float EPS = 0.01f;

bool rotateEnabled = false;
bool isMousePressed = false;

glm::quat rotation;

std::string path = "./models/";
std::string model = "dragon.obj";
std::string inputModel = path + model;
std::string outputModel = "./models/output_model.obj";

glm::vec3 lightDir = glm::vec3(0.0f, 0.0f, 1.0f);

bool simplificationTriggered = false;
bool loadOriginalModelTriggered = false;
bool similarityTriggered = false;
bool canSimplify = true;

std::vector<glm::vec3> positions;
std::vector<glm::vec3> normals;
std::vector<glm::vec3> colors;
std::vector<int> indices;

int width = 1600;
int height = 900;

float lastMouseX = width/2;
float lastMouseY = height/2;

bool LoadObj(const std::string& filename, std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<int>& indices, std::vector<glm::vec3>& colors) {
    // Open the file
    std::ifstream file(filename);
    // If the file is not open, output an error and return false
    if (!file.is_open()) {
        std::cout << "Failed to open the file: " << filename << std::endl;
        return false;
    }

    std::string line;
    // Create a vector to store the normals of each face
    std::vector<glm::vec3> faceNormals;

    // Read the file line by line
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        std::string input;
        lineStream >> input;

        // If the line defines a vertex
        if (input == "v") {
            glm::vec3 position;
            lineStream >> position.x >> position.y >> position.z;
            // Add the vertex to the positions vector
            positions.push_back(position);
        }
        // If the line defines a face
        else if (input == "f") {
            int v0, v1, v2;
            lineStream >> v0 >> v1 >> v2;
            v0--; v1--; v2--;

            // Add the indices of the vertices that form the face to the indices vector
            indices.push_back(v0);
            indices.push_back(v1);
            indices.push_back(v2);

            // Compute the normal of the face
            glm::vec3 edge1 = positions[v1] - positions[v0];
            glm::vec3 edge2 = positions[v2] - positions[v0];
            glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));

            // Add the normal to the faceNormals vector
            faceNormals.push_back(normal);

        }
    }

    // Initialize the normals vector with zeros
    normals.resize(positions.size(), glm::vec3(0.0f));
    // For each index in the indices vector
    for (size_t i = 0; i < indices.size(); i++) {
        int vertexIndex = indices[i];
        // Add the normal of the face to which the vertex belongs
        normals[vertexIndex] += faceNormals[i / 3];
    }

    // Normalize each normal in the normals vector
    for (glm::vec3& normal : normals) {
        normal = glm::normalize(normal);
    }

    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0, 1.0);

    // After processing all lines in the OBJ file:
    colors.resize(indices.size());
    for (size_t i = 0; i < indices.size(); i += 3) {
        // Generate a random color for the face
        glm::vec3 color(distribution(generator), distribution(generator), distribution(generator));

        // Assign the color to each vertex of the face
        colors[i] = color;
        colors[i + 1] = color;
        colors[i + 2] = color;
    }

    // Return true if the .obj file was successfully loaded
    return true;
}

float computeHausdorffDistance(const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB) {

    float maxDistance = 0.0f;

    // For each vertex in model A, find the nearest vertex in model B
    for (const glm::vec3& vertexA : verticesA) {
        float minDistance = std::numeric_limits<float>::max();

        for (const glm::vec3& vertexB : verticesB) {
            float distance = glm::distance(vertexA, vertexB);  // Calculate the distance between two vertices

            if (distance < minDistance) {
                minDistance = distance;
            }
        }
        if (minDistance > maxDistance) {
            maxDistance = minDistance;
        }
    }
    return maxDistance;
}

float evaluateSimilarity(const std::string& modelAFile, const std::string& modelBFile) {

    // Load the vertices of model A
    std::vector<glm::vec3> verticesA;
    std::ifstream modelAStream(modelAFile);

    if (!modelAStream.is_open()) {
        std::cerr << "Error: Failed to open model A file: " << modelAFile << std::endl;
        return -1.0f;  // return an error value or exit based on your preference
    }

    std::string line;

    while (std::getline(modelAStream, line)) {
        if (line.empty()) continue;
        if (line[0] == 'v') {
            std::istringstream iss(line.substr(1));
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            verticesA.push_back(vertex);
        }
    }

    // Load the vertices of model B
    std::vector<glm::vec3> verticesB;
    std::ifstream modelBStream(modelBFile);

    if (!modelBStream.is_open()) {
        std::cerr << "Error: Failed to open model B file: " << modelBFile << std::endl;
        return -1.0f;  // return an error value or exit based on your preference
    }

    while (std::getline(modelBStream, line)) {
        if (line.empty()) continue;
        if (line[0] == 'v') {
            std::istringstream iss(line.substr(1));
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            verticesB.push_back(vertex);
        }
    }

    // Compute the Hausdorff distance between model A and model B
    float hausdorffDistanceAB = computeHausdorffDistance(verticesA, verticesB);
    // Compute the Hausdorff distance between model B and model A
    float hausdorffDistanceBA = computeHausdorffDistance(verticesB, verticesA);
    // Determine the overall similarity based on the maximum Hausdorff distance
    float maxHausdorffDistance = std::max(hausdorffDistanceAB, hausdorffDistanceBA);

    return maxHausdorffDistance;
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    ImGuiIO& io = ImGui::GetIO();

    // Update the mouse position for ImGui
    io.MousePos = ImVec2(static_cast<float>(xpos), static_cast<float>(ypos));

    int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    if (state == GLFW_PRESS)
    {
        io.MouseDown[0] = true;
        isMousePressed = true;
    }
    else if (state == GLFW_RELEASE)
    {
        io.MouseDown[0] = false;
        isMousePressed = false;
    }

    // Check ImGui wants to capture mouse events
    if (!io.WantCaptureMouse && rotateEnabled)
    {
        if (firstMouse)
        {
            // Initialize lastMouseX and lastMouseY with the current mouse position
            lastMouseX = xpos;
            lastMouseY = ypos;
            // Indicate that the next mouse input won't be the first one
            firstMouse = false;
        }

        glm::vec3 right = glm::normalize(glm::cross(cameraFront, cameraUp));
        glm::vec3 up = glm::normalize(glm::cross(right, cameraFront));

        // Calculate how much the mouse has moved since the last frame
        float xoffset = xpos - lastMouseX;
        float yoffset = lastMouseY - ypos; // reversed since y-coordinates go from bottom to top

        // Update last mouse position with the current position
        lastMouseX = xpos;
        lastMouseY = ypos;

        // Apply sensitivity to mouse movement
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        // Calculate pitch and yaw quaternions based on mouse movement
        glm::quat pitchQuat = glm::angleAxis(glm::radians(yoffset), right);
        glm::quat yawQuat = glm::angleAxis(glm::radians(xoffset), up);

        // Calculate smoothness factor for interpolation
        float smoothness = 1.0f;  // Change this value to whatever feels right for your application

        // Interpolate between the current rotation and the target rotation
        glm::quat targetRotation = glm::normalize(rotation * pitchQuat * yawQuat);
        rotation = glm::slerp(rotation, targetRotation, smoothness);

        // Convert quaternion to rotation matrix
        glm::mat4 rotationMat = glm::mat4_cast(rotation);

        // Use the rotation matrix to rotate the camera's front vector
        cameraFront = glm::normalize(glm::mat3(rotationMat) * glm::vec3(0.0f, 0.0f, -1.0f));

        // Calculate yaw and pitch in degrees from the rotation quaternion
        yaw = glm::degrees(glm::yaw(rotation));
        pitch = glm::degrees(glm::pitch(rotation));
    }
}

void processInput(GLFWwindow* window)
{
    static int lastRightButtonState = GLFW_RELEASE;

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) { //Flat shading
        cameraPos += cameraSpeed * cameraFront;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) // press space to go up
        cameraPos += cameraSpeed * cameraUp;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) // press left shift to go down
        cameraPos -= cameraSpeed * cameraUp;

    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        lightDir = glm::vec3(0.0f,0.0f,-1.0f); // move light upwards
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        lightDir = glm::vec3(0.0f, 0.0f, 1.0f); // move light downwards

    int rightButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    if (rightButtonState == GLFW_PRESS && lastRightButtonState == GLFW_RELEASE) {
        rotateEnabled = !rotateEnabled; // Toggle rotation state when the right button is clicked

        // Start rotation
        if (rotateEnabled) {
            firstMouse = true;
        }
        // End rotation
        else {
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            lastMouseX = xpos;
            lastMouseY = ypos;
        }
    }
    lastRightButtonState = rightButtonState;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    // Update viewport
    glViewport(0, 0, width, height);
}

int main() {

    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> colors;
    std::vector<int> indices;
    std::string drawModel = inputModel;

    char modelInput[64] = "";

    LoadObj(drawModel, positions, normals, indices, colors);

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(width, height, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    glfwSetCursorPosCallback(window, mouse_callback);

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    const char* vertexShaderPath = "./shaders/shader.vert";
    const char* fragmentShaderPath = "./shaders/shader.frag";

    Shader shader(vertexShaderPath, fragmentShaderPath);

    const char* vertexPath = "./shaders/shader1.vert";
    const char* fragmentPath = "./shaders/shader1.frag";

    Shader shaderColor(vertexPath, fragmentPath);

    const char* NormalvertexPath = "./shaders/shader2.vert";
    const char* NormalfragmentPath = "./shaders/shader2.frag";

    Shader shaderNomral(NormalvertexPath, NormalfragmentPath);

    glViewport(0, 0, 1600, 900);

    // Buffers
    unsigned int VAO, VBO, EBO, NBO, CBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenBuffers(1, &NBO);
    glGenBuffers(1, &CBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), positions.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, NBO);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, CBO);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec3), colors.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(2);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    while (!glfwWindowShouldClose(window))
    {
        //glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {
            ImGui::Begin("Settings", NULL); // Create a window called "Settings"

            // Checkboxes for render modes
            ImGui::Checkbox("Flat", &isNormal);
            ImGui::Checkbox("Color", &isColor);
            ImGui::Checkbox("Wireframe", &isWireframe);

            // Adjust parameters
            ImGui::InputFloat("Mouse Speed", &cameraSpeed, 0.001f, 0.01f, "%.4f");
            ImGui::InputFloat("Simp_rate", &Rate, 0.001f, 0.01f, "%.4f");
            ImGui::InputFloat("Distance_EPS", &EPS, 0.001f, 0.01f, "%.4f");

            // Checkboxes for operations
            ImGui::Checkbox("Simplification", &simplificationTriggered);
            ImGui::Checkbox("Reset Model", &loadOriginalModelTriggered);
            ImGui::Checkbox("Similarity", &similarityTriggered);

            ImGui::InputText("Model name", modelInput, IM_ARRAYSIZE(modelInput));

            if (ImGui::Button("Load Model")) { // Reload model
                model = std::string(modelInput) + ".obj";
                inputModel = path + model;
                std::cout << "Model name set to: " << inputModel << std::endl;
                loadOriginalModelTriggered = true;
            }

            ImGui::End();
        }

        processInput(window);

        if (simplificationTriggered && canSimplify) {
            // Simplify the model and load the simplified model

            simplification(inputModel, outputModel, Rate, EPS);

            positions.clear();
            indices.clear();
            normals.clear();
            colors.clear();
            drawModel = outputModel;

            LoadObj(drawModel, positions, normals, indices, colors);

            // Update buffer
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), positions.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, NBO);
            glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, CBO);
            glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec3), colors.data(), GL_STATIC_DRAW);

            canSimplify = false;

            simplificationTriggered = false;
        }

        if (loadOriginalModelTriggered) {
            // Load the original model
            positions.clear();
            indices.clear();
            normals.clear();
            colors.clear();
            drawModel = inputModel;

            LoadObj(drawModel, positions, normals, indices, colors);

            clearData();

            // Update buffer
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), positions.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, NBO);
            glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, CBO);
            glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec3), colors.data(), GL_STATIC_DRAW);

            canSimplify = true;

            loadOriginalModelTriggered = false;
        }

        if (similarityTriggered) {
            std::string modelAFile = inputModel;
            std::string modelBFile = outputModel;


            float similarity = evaluateSimilarity(modelAFile, modelBFile);

            std::cout << "Similarity: " << similarity << std::endl;

            similarityTriggered = false;
        }

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Set the view and projection matrix in the shader
        shader.use();
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1600.0f / 900.0f, 0.1f, 1000.0f);

        glm::vec3 viewPos = cameraPos;
        shader.setVec3("lightDir", lightDir);
        shader.setVec3("viewPos", viewPos);

        shader.setMat4("view", view);
        shader.setMat4("projection", projection);

        if (isColor) {

            shaderColor.use();

            shaderColor.setMat4("view", view);
            shaderColor.setMat4("projection", projection);
        }
        else if (isNormal) {
            shaderNomral.use();
            shaderNomral.setVec3("lightDir", lightDir);
            shaderNomral.setVec3("viewPos", viewPos);

            shaderNomral.setMat4("view", view);
            shaderNomral.setMat4("projection", projection);
        }
        else {
            shader.use();
        }

        if (isWireframe) { // If wireframe mode is enabled
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Frame 
        }
        else { // If fill mode is enabled
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Fill
        }

        glLineWidth(2.0f);

        // Draw your object
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteBuffers(1, &NBO);
    glDeleteBuffers(1, &CBO);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
}