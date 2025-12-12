// visuals.cpp, created Andrew Gossen.
// Handles the OpenGL rendering of all rigid bodies.

#include "visuals/Visuals.hpp"
#include "core/Transform.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// Shader sources live in the .cpp to avoid violating ODR 
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

Visuals::Visuals(World& world) : world(world){

    // Constructor which initialises required OpenGL objects before the main render loop is called 

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        m_ok = false;
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    m_window = glfwCreateWindow(m_winWidth, m_winHeight, "Physics Engine", nullptr, nullptr);
    if (!m_window) {
        std::cerr << "Failed to create m_window\n";
        glfwTerminate();
        m_ok = false;
        return;
    }

    glfwMakeContextCurrent(m_window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        m_ok = false;
        return;
    }

    // Query actual framebuffer size (important on Retina / HiDPI)
    int m_fbWidth, fbHeight;
    glfwGetFramebufferSize(m_window, &m_fbWidth, &fbHeight);
    glViewport(0, 0, m_fbWidth, fbHeight);

    // Compile shaders
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    m_shaderProgram = glCreateProgram();
    glAttachShader(m_shaderProgram, vertexShader);
    glAttachShader(m_shaderProgram, fragmentShader);
    glLinkProgram(m_shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    glUseProgram(m_shaderProgram);

    // Uniform locations
    m_colourLoc = glGetUniformLocation(m_shaderProgram, "uColor");
    m_aspectLoc = glGetUniformLocation(m_shaderProgram, "uAspect");
    m_zoomLoc   = glGetUniformLocation(m_shaderProgram, "uZoom");

    // Create VAO/VBO once
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    m_ok = true;

}

Visuals::~Visuals(){

    // Deconstructor used for when the window is being closed 

    if (m_vao != 0) glDeleteVertexArrays(1, &m_vao);
    if (m_vbo != 0) glDeleteBuffers(1, &m_vbo);
    if (m_shaderProgram != 0) glDeleteProgram(m_shaderProgram);

    if (m_window) {
        glfwDestroyWindow(m_window);
        glfwTerminate();
    }

}

void Visuals::drawRigidBody(const RigidBody& body){

    // Draws a single rigid body using the active shader and geometry buffers.
    // Assumes the body's world-space vertices are up-to-date.
    // Body is const, does not modify physics state.
   
    if (!m_ok) return;

    // Handle framebuffer size each frame
    glfwGetFramebufferSize(m_window, &m_fbWidth, &m_fbHeight);
    glViewport(0, 0, m_fbWidth, m_fbHeight);
    float aspect = static_cast<float>(m_fbWidth) / static_cast<float>(m_fbHeight);

    glUseProgram(m_shaderProgram);
    glUniform1f(m_aspectLoc, aspect);
    glUniform1f(m_zoomLoc,   m_zoom);

    // Flatten world-space vertices into a float buffer
    std::vector<float> buffer;
    buffer.reserve(body.transformedVertices.size() * 2);
    
    for (const Vec2& v : body.transformedVertices) {
        buffer.push_back(v.x);
        buffer.push_back(v.y);
    }

    // Set colour for this body
    glUniform3f(
        m_colourLoc,
        body.colour.r,
        body.colour.g,
        body.colour.b
    );

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);

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


void Visuals::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;

    auto* visuals = static_cast<Visuals*>(glfwGetWindowUserPointer(window));
    if (!visuals) return;

    double sx, sy;
    glfwGetCursorPos(window, &sx, &sy);

    int winW, winH;
    glfwGetWindowSize(window, &winW, &winH);

    int fbW, fbH;
    glfwGetFramebufferSize(window, &fbW, &fbH);

    double sx_fb = sx * (double)fbW / (double)winW;
    double sy_fb = sy * (double)fbH / (double)winH;

    float xNDC =  2.0f * float(sx_fb) / float(fbW) - 1.0f;
    float yNDC =  1.0f - 2.0f * float(sy_fb) / float(fbH);

    float aspect = float(fbW) / float(fbH);
    Vec2 worldPos{
        xNDC * aspect / visuals->m_zoom,
        yNDC / visuals->m_zoom
    };

    RigidBody body(4, 1.0f, 2.0f);
    body.snapTo(worldPos);
    body.update = true;                
    body.colour = Colour{0.0f, 255.0f, 0.0f};
    body.restitution = 0.2f;

    visuals->world.getBodies().push_back(body);

}

void Visuals::renderLoop(){

    // Main render loop.
    // Handles input, renders all rigid bodies, and advances the physics simulation each frame.
    // Runs until the window is closed and attempts to cap the frame rate.
    // Must be called from the thread that owns the OpenGL context ( main.cpp with current setup).

    int frameRate=120; // 120 fps desired 
    float lastTime = glfwGetTime();


    auto* visuals = static_cast<Visuals*>(glfwGetWindowUserPointer(m_window));
    glfwSetWindowUserPointer(m_window, this);
    glfwSetMouseButtonCallback(m_window,mouseButtonCallback);

    while (!glfwWindowShouldClose(m_window)) {


        double frameStart = glfwGetTime();
        // Basic input
        if (glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(m_window, true);

        // Zoom controls: Q to zoom in, E to zoom out
        if (glfwGetKey(m_window, GLFW_KEY_Q) == GLFW_PRESS) {
            m_zoom *= 1.01f; // Zooms in
        }
        if (glfwGetKey(m_window, GLFW_KEY_E) == GLFW_PRESS) {
            m_zoom /= 1.01f; // Zooms out
        }

        // Handle framebuffer size changes (retina / m_window resize)
        glfwGetFramebufferSize(m_window, &m_fbWidth, &m_fbHeight);
        glViewport(0, 0, m_fbWidth, m_fbHeight);
        float aspect = static_cast<float>(m_fbWidth) / static_cast<float>(m_fbHeight);

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(m_shaderProgram);
        glUniform1f(m_aspectLoc, aspect);
        glUniform1f(m_zoomLoc, m_zoom);

        // Draw each rigid body in the world  
        for (auto& body:world.getBodies()){
            drawRigidBody(body);
        }

        glfwSwapBuffers(m_window);
        glfwPollEvents();

        double now = glfwGetTime();
        double dt = now - lastTime; // Calculate elapsed time from last frame for the dt value in world step 
        lastTime = now;
        world.step(static_cast<float>(dt));

        // Run at the desired frame rate 
        double frameEnd = glfwGetTime();
        double frameDuration = frameEnd - frameStart;
        double sleepTime = (1/frameRate) - frameDuration;

        if (sleepTime > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
        }

        
    }

    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
    glDeleteProgram(m_shaderProgram);
    glfwTerminate();
    glfwPollEvents();

};