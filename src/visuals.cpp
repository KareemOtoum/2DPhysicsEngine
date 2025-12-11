// Visuals.cpp

#include "visuals/Visuals.hpp"
#include "core/Transform.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// Shader sources live in the .cpp to avoid multiple definition problems
static const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec2 aPos;

uniform float uAspect;
uniform float uZoom;

void main() {
    // Zoom: <1 = zoom out, >1 = zoom in
    vec2 scaled = aPos * uZoom;

    // Correct for aspect ratio so squares stay square
    vec2 corrected = vec2(scaled.x / uAspect, scaled.y);
    gl_Position = vec4(corrected, 0.0, 1.0);
}
)";

static const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 uColor;

void main() {
    FragColor = vec4(uColor, 1.0);
}
)";

Visuals::Visuals(){
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        ok = false;
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    window = glfwCreateWindow(winWidth, winHeight, "Physics Engine", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        ok = false;
        return;
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        ok = false;
        return;
    }

    // Query actual framebuffer size (important on Retina / HiDPI)
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);

    // Compile shaders
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    glUseProgram(shaderProgram);

    // Uniform locations
    colourLoc = glGetUniformLocation(shaderProgram, "uColor");
    aspectLoc = glGetUniformLocation(shaderProgram, "uAspect");
    zoomLoc   = glGetUniformLocation(shaderProgram, "uZoom");

    // Create VAO/VBO once
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    ok = true;
}

Visuals::~Visuals(){
    if (vao != 0) glDeleteVertexArrays(1, &vao);
    if (vbo != 0) glDeleteBuffers(1, &vbo);
    if (shaderProgram != 0) glDeleteProgram(shaderProgram);

    if (window) {
        glfwDestroyWindow(window);
        glfwTerminate();
    }
}

// Draw a single rigid body (assumes body.transformedVertices already updated)
void Visuals::drawRigidBody(RigidBody& body){
   
    if (!ok) return;

    // Handle framebuffer size each frame
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);
    float aspect = static_cast<float>(fbWidth) / static_cast<float>(fbHeight);

    glUseProgram(shaderProgram);
    glUniform1f(aspectLoc, aspect);
    glUniform1f(zoomLoc,   zoom);

    // Flatten world-space vertices into a float buffer
    std::vector<float> buffer;
    buffer.reserve(body.transformedVertices.size() * 2);

    physEng::worldSpace(body);
    for (const Vec2& v : body.transformedVertices) {
        buffer.push_back(v.x);
        buffer.push_back(v.y);
    }

    // Set colour for this body
    glUniform3f(
        colourLoc,
        body.colour.r,
        body.colour.g,
        body.colour.b
    );

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glBufferData(
        GL_ARRAY_BUFFER,
        buffer.size() * sizeof(float),
        buffer.data(),
        GL_DYNAMIC_DRAW
    );

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,           
        2,           
        GL_FLOAT,
        GL_FALSE,
        2 * sizeof(float),
        (void*)0
    );

    glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(body.transformedVertices.size()));

    glBindVertexArray(0);

}

// Render Loop 
void Visuals::renderLoop(World& world){

    int frameRate=120; // 120 fps desired 
    float lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {

        double frameStart = glfwGetTime();
        // Basic input
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, true);

        // Zoom controls: Q to zoom in, E to zoom out
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
            zoom *= 1.01f; // zoom in
        }
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
            zoom /= 1.01f; // zoom out
        }

        // Handle framebuffer size changes (retina / window resize)
        glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
        glViewport(0, 0, fbWidth, fbHeight);
        float aspect = static_cast<float>(fbWidth) / static_cast<float>(fbHeight);

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shaderProgram);
        glUniform1f(aspectLoc, aspect);
        glUniform1f(zoomLoc, zoom);

        // Draw bodies 
        for (auto& body:world.getBodies()){
            drawRigidBody(body);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();

       
           // --- PHYSICS STEP ---

        // Option A: use *actual* dt (variable timestep)
        double now = glfwGetTime();
        double dt = now - lastTime;
        lastTime = now;
        world.step(static_cast<float>(dt));

        // Option B (if you want fixed timestep): 
        // world.step(static_cast<float>(targetFrameTime));

        // --- FRAME LIMITING TO 120 FPS ---
        double frameEnd = glfwGetTime();
        double frameDuration = frameEnd - frameStart;
        double sleepTime = (1/frameRate) - frameDuration;

        if (sleepTime > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
        }
        
    }

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteProgram(shaderProgram);
    glfwTerminate();

};