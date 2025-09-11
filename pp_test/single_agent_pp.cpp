#include <iostream>
#include <vector>
#include <memory>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Forward declarations
class World;
class Entity;
class Agent;
class Obstacle;
class Camera;
class PathPlanningStrategy;

// Strategy Pattern: Abstract strategy for path planning
class PathPlanningStrategy {
public:
    virtual ~PathPlanningStrategy() = default;
    virtual std::vector<glm::vec3> planPath(const glm::vec3& start, const glm::vec3& goal, const std::vector<Obstacle*>& obstacles) = 0;
};

// Concrete strategies would be implemented in separate files later

// Entity class - base for all objects in the world
class Entity {
protected:
    glm::vec3 position;
    glm::vec3 scale;
    glm::vec3 color;

public:
    Entity(const glm::vec3& pos = glm::vec3(0.0f), 
           const glm::vec3& scl = glm::vec3(1.0f), 
           const glm::vec3& clr = glm::vec3(1.0f)) 
        : position(pos), scale(scl), color(clr) {}
    
    virtual ~Entity() = default;
    
    virtual void update(float deltaTime) {}
    virtual void render() = 0;
    
    void setPosition(const glm::vec3& pos) { position = pos; }
    const glm::vec3& getPosition() const { return position; }
    
    void setScale(const glm::vec3& scl) { scale = scl; }
    const glm::vec3& getScale() const { return scale; }
    
    void setColor(const glm::vec3& clr) { color = clr; }
    const glm::vec3& getColor() const { return color; }
};

// Obstacle class
class Obstacle : public Entity {
public:
    Obstacle(const glm::vec3& pos = glm::vec3(0.0f),
             const glm::vec3& scl = glm::vec3(1.0f),
             const glm::vec3& clr = glm::vec3(0.7f, 0.3f, 0.3f))
        : Entity(pos, scl, clr) {}
    
    void render() override {
        // Save current state
        glPushMatrix();
        
        // Apply transformations
        glTranslatef(position.x, position.y, position.z);
        glScalef(scale.x, scale.y, scale.z);
        
        // Set color
        glColor3f(color.r, color.g, color.b);
        
        // Draw cube
        glutSolidCube(1.0f);
        
        // Restore state
        glPopMatrix();
    }
};

// Agent class
class Agent : public Entity {
private:
    glm::vec3 goal;
    std::vector<glm::vec3> path;
    std::unique_ptr<PathPlanningStrategy> pathStrategy;
    float speed;
    
public:
    Agent(const glm::vec3& pos = glm::vec3(0.0f),
          const glm::vec3& gl = glm::vec3(10.0f, 0.0f, 10.0f),
          float spd = 2.0f,
          const glm::vec3& clr = glm::vec3(0.2f, 0.7f, 0.2f))
        : Entity(pos, glm::vec3(0.5f), clr), goal(gl), speed(spd) {}
    
    void setPathPlanningStrategy(std::unique_ptr<PathPlanningStrategy> strategy) {
        pathStrategy = std::move(strategy);
    }
    
    void setGoal(const glm::vec3& newGoal) {
        goal = newGoal;
    }
    
    void planPath(const std::vector<Obstacle*>& obstacles) {
        if (pathStrategy) {
            path = pathStrategy->planPath(position, goal, obstacles);
        }
    }
    
    void update(float deltaTime) override {
        // Simple path following logic
        if (!path.empty()) {
            glm::vec3 targetPoint = path[0];
            glm::vec3 direction = targetPoint - position;
            
            // Normalize direction and scale by speed and deltaTime
            float distance = glm::length(direction);
            if (distance > 0.1f) {
                direction = glm::normalize(direction) * speed * deltaTime;
                position += direction;
            } else {
                // Reached waypoint, remove it
                path.erase(path.begin());
            }
        }
    }
    
    void render() override {
        // Save current state
        glPushMatrix();
        
        // Apply transformations
        glTranslatef(position.x, position.y, position.z);
        glScalef(scale.x, scale.y, scale.z);
        
        // Set color
        glColor3f(color.r, color.g, color.b);
        
        // Draw agent as a sphere
        glutSolidSphere(1.0f, 16, 16);
        
        // Restore state
        glPopMatrix();
        
        // Render path
        if (!path.empty()) {
            glColor3f(0.9f, 0.9f, 0.0f);
            glBegin(GL_LINE_STRIP);
            glVertex3f(position.x, position.y, position.z);
            for (const auto& point : path) {
                glVertex3f(point.x, point.y, point.z);
            }
            glEnd();
        }
        
        // Render goal
        glPushMatrix();
        glTranslatef(goal.x, goal.y, goal.z);
        glColor3f(0.9f, 0.1f, 0.1f);
        glutSolidSphere(0.3f, 8, 8);
        glPopMatrix();
    }
};

// Camera class
class Camera {
private:
    glm::vec3 position;
    glm::vec3 target;
    glm::vec3 up;
    
public:
    Camera(const glm::vec3& pos = glm::vec3(0.0f, 10.0f, 15.0f),
           const glm::vec3& tgt = glm::vec3(0.0f),
           const glm::vec3& up = glm::vec3(0.0f, 1.0f, 0.0f))
        : position(pos), target(tgt), up(up) {}
    
    void apply() {
        gluLookAt(position.x, position.y, position.z,
                  target.x, target.y, target.z,
                  up.x, up.y, up.z);
    }
};

// World class - container for all entities and simulation logic
class World {
private:
    std::vector<std::unique_ptr<Entity>> entities;
    std::vector<Obstacle*> obstacles;  // Non-owning pointers for quick access
    Agent* agent;  // Non-owning pointer
    Camera camera;
    float worldSize;
    
public:
    World(float size = 20.0f) : worldSize(size), agent(nullptr) {
        setupWorld();
    }
    
    void setupWorld() {
        // Create agent
        auto agentPtr = std::make_unique<Agent>(glm::vec3(0.0f, 0.5f, 0.0f), 
                                               glm::vec3(8.0f, 0.5f, 8.0f));
        agent = agentPtr.get();
        entities.push_back(std::move(agentPtr));
        
        // Create obstacles
        for (int i = 0; i < 10; ++i) {
            float x = static_cast<float>(rand() % static_cast<int>(worldSize)) - worldSize/2;
            float z = static_cast<float>(rand() % static_cast<int>(worldSize)) - worldSize/2;
            auto obstaclePtr = std::make_unique<Obstacle>(
                glm::vec3(x, 0.5f, z), 
                glm::vec3(1.0f + static_cast<float>(rand() % 3), 
                         1.0f, 
                         1.0f + static_cast<float>(rand() % 3))
            );
            obstacles.push_back(obstaclePtr.get());
            entities.push_back(std::move(obstaclePtr));
        }
        
        // Setup camera
        camera = Camera(glm::vec3(0.0f, worldSize/1.5f, worldSize/1.2f), 
                        glm::vec3(0.0f, 0.0f, 0.0f));
    }
    
    void update(float deltaTime) {
        for (auto& entity : entities) {
            entity->update(deltaTime);
        }
    }
    
    void render() {
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Setup camera
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        camera.apply();
        
        // Draw world boundaries
        renderWorldBoundaries();
        
        // Draw entities
        for (auto& entity : entities) {
            entity->render();
        }
    }
    
    void renderWorldBoundaries() {
        glColor3f(0.5f, 0.5f, 0.5f);
        float halfSize = worldSize / 2.0f;
        
        // Draw ground grid
        glBegin(GL_LINES);
        for (float i = -halfSize; i <= halfSize; i += 1.0f) {
            // Lines along X axis
            glVertex3f(i, 0.0f, -halfSize);
            glVertex3f(i, 0.0f, halfSize);
            
            // Lines along Z axis
            glVertex3f(-halfSize, 0.0f, i);
            glVertex3f(halfSize, 0.0f, i);
        }
        glEnd();
        
        // Draw world boundary
        glColor3f(0.8f, 0.8f, 0.8f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(-halfSize, 0.0f, -halfSize);
        glVertex3f(halfSize, 0.0f, -halfSize);
        glVertex3f(halfSize, 0.0f, halfSize);
        glVertex3f(-halfSize, 0.0f, halfSize);
        glEnd();
    }
    
    Agent* getAgent() { return agent; }
    const std::vector<Obstacle*>& getObstacles() const { return obstacles; }
};

// Global variables
World* world = nullptr;
int windowWidth = 800;
int windowHeight = 600;
float lastFrameTime = 0.0f;

// Function prototypes
void display();
void reshape(int width, int height);
void keyboard(unsigned char key, int x, int y);
void idle();
void cleanup();

// Main function
int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("Path Planning Simulation");
    
    // Initialize OpenGL
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    
    // Setup world
    world = new World(20.0f);
    
    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);
    
    // Start main loop
    glutMainLoop();
    
    // Cleanup
    cleanup();
    
    return 0;
}

// Callback functions
void display() {
    if (world) {
        world->render();
    }
    glutSwapBuffers();
}

void reshape(int width, int height) {
    windowWidth = width;
    windowHeight = height;
    
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, static_cast<float>(width)/static_cast<float>(height), 0.1f, 100.0f);
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:  // ESC key
            cleanup();
            exit(0);
            break;
        case 'r':
        case 'R':
            // Reset simulation
            if (world) {
                delete world;
                world = new World(20.0f);
            }
            break;
        default:
            break;
    }
}

void idle() {
    // Calculate delta time
    float currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
    float deltaTime = currentTime - lastFrameTime;
    lastFrameTime = currentTime;
    
    // Update world
    if (world) {
        world->update(deltaTime);
    }
    
    // Trigger redisplay
    glutPostRedisplay();
}

void cleanup() {
    if (world) {
        delete world;
        world = nullptr;
    }
}
