#include <iostream>
#include <vector>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "path_planning_interface.hpp"

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
    bool isMoving;
    
public:
    Agent(const glm::vec3& pos = glm::vec3(0.0f),
          const glm::vec3& gl = glm::vec3(10.0f, 0.0f, 10.0f),
          float spd = 2.0f,
          const glm::vec3& clr = glm::vec3(0.2f, 0.7f, 0.2f))
        : Entity(pos, glm::vec3(0.5f), clr), goal(gl), speed(spd), isMoving(false) {}
    
    void setPathPlanningStrategy(std::unique_ptr<PathPlanningStrategy> strategy) {
        pathStrategy = std::move(strategy);
    }
    
    void setGoal(const glm::vec3& newGoal) {
        goal = newGoal;
    }
    
    const glm::vec3& getGoal() const {
        return goal;
    }
    
    void setPath(const std::vector<glm::vec3>& newPath) {
        path = newPath;
        isMoving = !path.empty();
    }
    
    void startMoving() {
        isMoving = true;
    }
    
    void stopMoving() {
        isMoving = false;
    }
    
    bool getIsMoving() const {
        return isMoving;
    }
    
    void update(float deltaTime) override {
        // Only move if explicitly told to move and has a path
        if (isMoving && !path.empty()) {
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
                if (path.empty()) {
                    isMoving = false;  // Stop when path is completed
                }
            }
        }
    }
    
    void render() override {
        // Save current state
        glPushMatrix();
        
        // Apply transformations
        glTranslatef(position.x, position.y, position.z);
        glScalef(scale.x, scale.y, scale.z);
        
        // Set color - different color if moving
        if (isMoving) {
            glColor3f(0.2f, 0.9f, 0.2f);  // Bright green when moving
        } else {
            glColor3f(color.r, color.g, color.b);  // Default color when stationary
        }
        
        // Draw agent as a sphere
        glutSolidSphere(1.0f, 16, 16);
        
        // Restore state
        glPopMatrix();
        
        // Draw path if it exists
        if (!path.empty()) {
            glColor3f(0.9f, 0.9f, 0.0f);
            glBegin(GL_LINE_STRIP);
            glVertex3f(position.x, position.y, position.z);
            for (const auto& point : path) {
                glVertex3f(point.x, point.y, point.z);
            }
            glEnd();
        } else {
            // Draw direct line to goal when no path is set
            glColor3f(0.5f, 0.5f, 0.5f);
            glBegin(GL_LINES);
            glVertex3f(position.x, position.y, position.z);
            glVertex3f(goal.x, goal.y, goal.z);
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
    glm::vec3 target;
    glm::vec3 up;
    float radius;
    float theta;  // Horizontal angle (around Y axis)
    float phi;    // Vertical angle (elevation)
    float minRadius;
    float maxRadius;
    float minPhi;
    float maxPhi;
    
    void updatePosition() {
        // Convert spherical coordinates to cartesian
        float x = radius * sin(phi) * cos(theta);
        float y = radius * cos(phi);
        float z = radius * sin(phi) * sin(theta);
        position = target + glm::vec3(x, y, z);
    }
    
    glm::vec3 position;
    
public:
    Camera(const glm::vec3& tgt = glm::vec3(0.0f),
           float r = 15.0f,
           float th = 0.0f,
           float ph = glm::radians(45.0f))
        : target(tgt), up(glm::vec3(0.0f, 1.0f, 0.0f)), 
          radius(r), theta(th), phi(ph),
          minRadius(5.0f), maxRadius(50.0f),
          minPhi(glm::radians(10.0f)), maxPhi(glm::radians(170.0f)) {
        updatePosition();
    }
    
    void apply() {
        gluLookAt(position.x, position.y, position.z,
                  target.x, target.y, target.z,
                  up.x, up.y, up.z);
    }
    
    void rotate(float deltaTheta, float deltaPhi) {
        theta += deltaTheta;
        phi += deltaPhi;
        
        // Clamp phi to prevent flipping
        phi = glm::clamp(phi, minPhi, maxPhi);
        
        updatePosition();
    }
    
    void zoom(float deltaRadius) {
        radius += deltaRadius;
        radius = glm::clamp(radius, minRadius, maxRadius);
        updatePosition();
    }
    
    void setTarget(const glm::vec3& newTarget) {
        target = newTarget;
        updatePosition();
    }
    
    const glm::vec3& getPosition() const { return position; }
    const glm::vec3& getTarget() const { return target; }
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
        // Seed random number generator for different scenarios each run
        srand(static_cast<unsigned int>(time(nullptr)));
        
        // Spawn agent in one corner and goal in diagonally opposite corner (3D diagonal)
        float halfSize = worldSize / 2.0f;
        float maxHeight = worldSize / 2.0f; // World height limit
        float cornerOffset = halfSize * 0.8f; // Stay a bit away from the exact edges
        
        // Randomly choose which 3D diagonal to use
        bool diagonal = rand() % 2;
        
        float agentX, agentY, agentZ, goalX, goalY, goalZ;
        if (diagonal) {
            // Agent in (-,-,-) corner, goal in (+,+,+) corner (3D diagonal)
            agentX = -cornerOffset;
            agentY = 2.0f; // Low position
            agentZ = -cornerOffset;
            goalX = cornerOffset;
            goalY = maxHeight - 2.0f; // High position but within bounds
            goalZ = cornerOffset;
        } else {
            // Agent in (-,+,-) corner, goal in (+,-,+) corner (3D diagonal)
            agentX = -cornerOffset;
            agentY = maxHeight - 2.0f; // High position but within bounds
            agentZ = -cornerOffset;
            goalX = cornerOffset;
            goalY = 2.0f; // Low position
            goalZ = cornerOffset;
        }
        
        auto agentPtr = std::make_unique<Agent>(glm::vec3(agentX, agentY, agentZ), 
                                               glm::vec3(goalX, goalY, goalZ));
        agent = agentPtr.get();
        entities.push_back(std::move(agentPtr));
        
        // Create obstacles in 3D space, avoiding agent and goal positions
        glm::vec3 agentPos(agentX, agentY, agentZ);
        glm::vec3 goalPos(goalX, goalY, goalZ);
        float minDistance = 3.0f; // Minimum distance from agent/goal
        
        int obstaclesPlaced = 0;
        int maxAttempts = 100; // Increase attempts for better obstacle placement
        
        while (obstaclesPlaced < 15 && maxAttempts > 0) {
            // Ensure obstacles spawn within proper bounds
            float x = -halfSize + static_cast<float>(rand()) / RAND_MAX * (2.0f * halfSize);
            float y = 1.0f + static_cast<float>(rand()) / RAND_MAX * (maxHeight - 2.0f); // Within height bounds
            float z = -halfSize + static_cast<float>(rand()) / RAND_MAX * (2.0f * halfSize);
            
            glm::vec3 obstaclePos(x, y, z);
            
            // Check if obstacle is far enough from both agent and goal
            if (glm::distance(obstaclePos, agentPos) >= minDistance && 
                glm::distance(obstaclePos, goalPos) >= minDistance) {
                
                float sizeX = 1.0f + static_cast<float>(rand()) / RAND_MAX * 2.0f;
                float sizeY = 1.0f + static_cast<float>(rand()) / RAND_MAX * 2.0f;
                float sizeZ = 1.0f + static_cast<float>(rand()) / RAND_MAX * 2.0f;
                
                auto obstaclePtr = std::make_unique<Obstacle>(
                    obstaclePos, 
                    glm::vec3(sizeX, sizeY, sizeZ)
                );
                obstacles.push_back(obstaclePtr.get());
                entities.push_back(std::move(obstaclePtr));
                obstaclesPlaced++;
            }
            maxAttempts--;
        }
        
        std::cout << "Spawned agent at (" << agentX << ", " << agentY << ", " << agentZ << ")" << std::endl;
        std::cout << "Spawned goal at (" << goalX << ", " << goalY << ", " << goalZ << ")" << std::endl;
        std::cout << "Placed " << obstaclesPlaced << " obstacles" << std::endl;
        
        // Setup camera with spherical coordinates
        camera = Camera(glm::vec3(0.0f, worldSize/4, 0.0f), // Target at mid-height
                       worldSize * 1.2f,                     // Initial radius
                       0.0f,                                  // Initial theta (horizontal angle)
                       glm::radians(45.0f));                 // Initial phi (elevation angle)
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
        glColor3f(0.3f, 0.3f, 0.3f);
        float halfSize = worldSize / 2.0f;
        float height = worldSize / 2.0f;
        
        glBegin(GL_LINES);
        
        // Draw ground grid (XZ plane at y=0)
        for (float i = -halfSize; i <= halfSize; i += 2.0f) {
            // Lines along X axis
            glVertex3f(i, 0.0f, -halfSize);
            glVertex3f(i, 0.0f, halfSize);
            
            // Lines along Z axis
            glVertex3f(-halfSize, 0.0f, i);
            glVertex3f(halfSize, 0.0f, i);
        }
        
        // Draw vertical grid lines at intervals
        for (float i = -halfSize; i <= halfSize; i += 4.0f) {
            for (float j = -halfSize; j <= halfSize; j += 4.0f) {
                // Vertical lines
                glVertex3f(i, 0.0f, j);
                glVertex3f(i, height, j);
            }
        }
        
        // Draw ceiling grid (XZ plane at y=height)
        glColor3f(0.2f, 0.2f, 0.2f);
        for (float i = -halfSize; i <= halfSize; i += 4.0f) {
            // Lines along X axis
            glVertex3f(i, height, -halfSize);
            glVertex3f(i, height, halfSize);
            
            // Lines along Z axis
            glVertex3f(-halfSize, height, i);
            glVertex3f(halfSize, height, i);
        }
        
        glEnd();
        
        // Draw 3D world boundary box
        glColor3f(0.6f, 0.6f, 0.6f);
        glBegin(GL_LINES);
        
        // Bottom face
        glVertex3f(-halfSize, 0.0f, -halfSize);
        glVertex3f(halfSize, 0.0f, -halfSize);
        
        glVertex3f(halfSize, 0.0f, -halfSize);
        glVertex3f(halfSize, 0.0f, halfSize);
        
        glVertex3f(halfSize, 0.0f, halfSize);
        glVertex3f(-halfSize, 0.0f, halfSize);
        
        glVertex3f(-halfSize, 0.0f, halfSize);
        glVertex3f(-halfSize, 0.0f, -halfSize);
        
        // Top face
        glVertex3f(-halfSize, height, -halfSize);
        glVertex3f(halfSize, height, -halfSize);
        
        glVertex3f(halfSize, height, -halfSize);
        glVertex3f(halfSize, height, halfSize);
        
        glVertex3f(halfSize, height, halfSize);
        glVertex3f(-halfSize, height, halfSize);
        
        glVertex3f(-halfSize, height, halfSize);
        glVertex3f(-halfSize, height, -halfSize);
        
        // Vertical edges
        glVertex3f(-halfSize, 0.0f, -halfSize);
        glVertex3f(-halfSize, height, -halfSize);
        
        glVertex3f(halfSize, 0.0f, -halfSize);
        glVertex3f(halfSize, height, -halfSize);
        
        glVertex3f(halfSize, 0.0f, halfSize);
        glVertex3f(halfSize, height, halfSize);
        
        glVertex3f(-halfSize, 0.0f, halfSize);
        glVertex3f(-halfSize, height, halfSize);
        
        glEnd();
    }
    
    Agent* getAgent() { return agent; }
    const std::vector<Obstacle*>& getObstacles() const { return obstacles; }
    Camera& getCamera() { return camera; }
    
    // Interface for path planning module
    void triggerPathPlanning() {
        if (agent) {
            // Create structured environment data
            PathPlanning::Environment env;
            
            // Set world bounds
            float maxHeight = worldSize / 2.0f;
            env.worldBounds = PathPlanning::BoundingBox(
                glm::vec3(0.0f, maxHeight/2.0f, 0.0f),  // Center of world
                glm::vec3(worldSize, maxHeight, worldSize)  // Full world size
            );
            
            // Set agent info
            env.agentStart = agent->getPosition();
            env.goalPosition = agent->getGoal();
            env.agentRadius = 0.5f;
            env.goalTolerance = 1.0f;
            env.stepSize = 0.8f;
            
            // Convert obstacles to bounding boxes
            env.obstacles.reserve(obstacles.size());
            for (const auto& obstacle : obstacles) {
                PathPlanning::BoundingBox box(obstacle->getPosition(), obstacle->getScale());
                env.obstacles.push_back(box);
            }
            
            // Use the structured interface
            std::vector<glm::vec3> path = PathPlanning::planPath(env);
            agent->setPath(path);
        }
    }
    
    void startAgentMovement() {
        if (agent) {
            agent->startMoving();
        }
    }
    
    void stopAgentMovement() {
        if (agent) {
            agent->stopMoving();
        }
    }
};

// Global variables
World* world = nullptr;
int windowWidth = 800;
int windowHeight = 600;
float lastFrameTime = 0.0f;

// Mouse control variables
bool mousePressed = false;
int lastMouseX = 0;
int lastMouseY = 0;
float mouseSensitivity = 0.01f;
float scrollSensitivity = 1.0f;

// Function prototypes
void display();
void reshape(int width, int height);
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void idle();
void cleanup();

// Main function
int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("3D Agent Navigation Simulation");
    
    // Initialize OpenGL
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    
    // Initialize path planning module
    PathPlanning::initialize();
    
    // Setup world
    world = new World(20.0f);
    
    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);
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
        case 'c':
        case 'C':
            // Reset camera to default position
            if (world) {
                world->getCamera() = Camera(glm::vec3(0.0f, 10.0f, 0.0f), 24.0f, 0.0f, glm::radians(45.0f));
            }
            break;
        case '+':
        case '=':
            // Zoom in
            if (world) {
                world->getCamera().zoom(-2.0f);
            }
            break;
        case '-':
        case '_':
            // Zoom out
            if (world) {
                world->getCamera().zoom(2.0f);
            }
            break;
        case 'p':
        case 'P':
            // Trigger path planning (placeholder for your module)
            if (world) {
                world->triggerPathPlanning();
            }
            break;
        case 's':
        case 'S':
            // Start agent movement
            if (world) {
                world->startAgentMovement();
            }
            break;
        case 't':
        case 'T':
            // Stop agent movement
            if (world) {
                world->stopAgentMovement();
            }
            break;
        default:
            break;
    }
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            mousePressed = true;
            lastMouseX = x;
            lastMouseY = y;
        } else if (state == GLUT_UP) {
            mousePressed = false;
        }
    }
}

void mouseMotion(int x, int y) {
    if (mousePressed && world) {
        int deltaX = x - lastMouseX;
        int deltaY = y - lastMouseY;
        
        // Convert mouse movement to camera rotation
        float deltaTheta = deltaX * mouseSensitivity;
        float deltaPhi = deltaY * mouseSensitivity;
        
        world->getCamera().rotate(deltaTheta, deltaPhi);
        
        lastMouseX = x;
        lastMouseY = y;
        
        glutPostRedisplay();
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
    
    // Shutdown path planning module
    PathPlanning::shutdown();
}
