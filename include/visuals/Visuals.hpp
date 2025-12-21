// Visuals.hpp, created by Andrew Gossen.

// ------------------------------------------------------------
//  OpenGL/GLFW rendering wrapper for the physics engine.

// Ownership & Lifetime:
// - Visuals owns the GLFWwindow and OpenGL resources (shader, VAO, VBO).
// - Resources are acquired in the constructor and released in the destructor ( RAII ).
// - For now copying  is disabled to prevent double-free of GL resources.

// Usage Contract:
// - renderLoop() assumes therei s a valid GL context on the calling thread.
// - All methods must be called from the same thread that owns the GL context

// Error Handling:
// - If initialization fails m_ok is false and visuals.cpp returns, GL resources may be nullptr 
// ------------------------------------------------------------

#pragma once
#include "core/RigidBody.hpp"
#include "core/World.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Visuals {

public:

    Visuals(World& world);
    ~Visuals();
    
    Visuals(const Visuals&) = delete;
    Visuals& operator=(const Visuals&) = delete;
    
    World& world; 

    bool isValid() const { return m_ok; }

    // Draws a single rigid body using internal VAO/VBO + shader
    // Does not modify physics state.
    void drawRigidBody(const RigidBody& body);

    // Runs the render loop
    // Blocks until the window closes
    void renderLoop();
    GLFWwindow* window() const { return m_window; }

    void setZoom(float z) { m_zoom = z; }
    float zoom() const { return m_zoom; }

private:

    // Transform the outWorldPos ( A mouse pos ) to world-space, returns success
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

    GLFWwindow* m_window = nullptr;

    int m_winWidth  = 800;
    int m_winHeight = 600;
    int m_fbWidth   = 0;
    int m_fbHeight  = 0;

    GLuint m_shaderProgram = 0;
    GLint  m_colourLoc     = -1;
    GLint  m_aspectLoc     = -1;
    GLint  m_zoomLoc       = -1;

    GLuint m_vao = 0;
    GLuint m_vbo = 0;

    float m_zoom = 0.07f;
    bool  m_ok   = false;

};
