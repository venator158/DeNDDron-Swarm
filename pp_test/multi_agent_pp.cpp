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
#include "strategies/apf_mapf_strategy.hpp"

// Forward declarations
class MultiAgentWorld;
class Entity;
class Agent;
class Obstacle;
class Camera;

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
    void setColor(const glm::vec3& clr) { color = clr; }
    const glm::vec3& getColor() const { return color; }
};

// Simulation state shared among agents for real-time force calculations
struct SimulationState {
    std::vector<Agent*> agents;
    std::vector<PathPlanning::BoundingBox> obstacles;
    PathPlanning::APFMAPFStrategy* apfStrategy;
    bool useRealTimeForces;
    float agentRadius;
    
    SimulationState() : apfStrategy(nullptr), useRealTimeForces(true), agentRadius(0.5f) {}
};

// Agent class
class Agent : public Entity {
private:
    glm::vec3 goalPosition;
    std::vector<glm::vec3> path;
    size_t currentPathIndex;
    bool isMoving;
    bool hasReachedGoal;
    float speed;
    int agentId;
    SimulationState* simState;
    
public:
    Agent(int id, const glm::vec3& pos = glm::vec3(0.0f),
          const glm::vec3& goal = glm::vec3(0.0f),
          const glm::vec3& clr = glm::vec3(0.0f, 1.0f, 0.0f))
        : Entity(pos, glm::vec3(0.5f), clr), goalPosition(goal), 
          currentPathIndex(0), isMoving(false), hasReachedGoal(false), 
          speed(2.0f), agentId(id), simState(nullptr) {}
    
    void setSimulationState(SimulationState* state) { simState = state; }
    
    void setPath(const std::vector<glm::vec3>& newPath) {
        path = newPath;
        currentPathIndex = 0;
        hasReachedGoal = false;
    }
    
    void startMovement() { isMoving = true; }
    void stopMovement() { isMoving = false; }
    bool getIsMoving() const { return isMoving; }
    bool getHasReachedGoal() const { return hasReachedGoal; }
    const glm::vec3& getGoalPosition() const { return goalPosition; }
    const std::vector<glm::vec3>& getPath() const { return path; }
    int getId() const { return agentId; }
    
    void update(float deltaTime) override {
        if (!isMoving || hasReachedGoal) return;
        
        // Check if we've reached the goal
        float distanceToGoal = glm::length(goalPosition - position);
        if (distanceToGoal < 0.2f) {
            hasReachedGoal = true;
            isMoving = false;
            std::cout << "Agent " << agentId << " reached goal!" << std::endl;
            return;
        }
        
        glm::vec3 force(0.0f);
        
        if (simState && simState->useRealTimeForces && simState->apfStrategy) {
            // Use real-time force calculations
            std::vector<glm::vec3> agentPositions;
            std::vector<glm::vec3> goalPositions;
            std::vector<float> agentRadii;
            
            // Gather all agent positions and goals
            for (const auto& agent : simState->agents) {
                agentPositions.push_back(agent->getPosition());
                goalPositions.push_back(agent->getGoalPosition());
                agentRadii.push_back(simState->agentRadius);
            }
            
            // Calculate total force for this agent
            force = simState->apfStrategy->calculateTotalForceForAgent(
                agentId, agentPositions, goalPositions, simState->obstacles, agentRadii);
                
            // Apply force with damping
            if (glm::length(force) > 0.0f) {
                glm::vec3 acceleration = glm::normalize(force) * speed;
                position += acceleration * deltaTime;
            }
        } else {
            // Fallback to waypoint following
            if (path.empty() || currentPathIndex >= path.size()) return;
            
            glm::vec3 target = path[currentPathIndex];
            glm::vec3 direction = target - position;
            float distance = glm::length(direction);
            
            if (distance < 0.1f) {
                currentPathIndex++;
                return;
            }
            
            direction = glm::normalize(direction);
            position += direction * speed * deltaTime;
        }
    }
    
    void render() override {
        glPushMatrix();
        
        // Translate to agent position
        glTranslatef(position.x, position.y, position.z);
        
        // Set agent color
        glColor3f(color.r, color.g, color.b);
        
        // Draw agent as a sphere
        GLUquadric* quad = gluNewQuadric();
        gluSphere(quad, scale.x, 16, 16);
        gluDeleteQuadric(quad);
        
        glPopMatrix();
        
        // Draw goal position (in world coordinates)
        glPushMatrix();
        glTranslatef(goalPosition.x, goalPosition.y, goalPosition.z);
        glColor3f(color.r * 0.5f, color.g * 0.5f, color.b * 0.5f);
        GLUquadric* goalQuad = gluNewQuadric();
        gluSphere(goalQuad, scale.x * 0.3f, 8, 8);
        gluDeleteQuadric(goalQuad);
        glPopMatrix();
        
        // Draw path (in world coordinates)
        if (!path.empty()) {
            glColor3f(color.r * 0.7f, color.g * 0.7f, color.b * 0.7f);
            glLineWidth(2.0f);
            glBegin(GL_LINE_STRIP);
            for (const auto& waypoint : path) {
                glVertex3f(waypoint.x, waypoint.y, waypoint.z);
            }
            glEnd();
            glLineWidth(1.0f);
        }
    }
};

// Obstacle class
class Obstacle : public Entity {
public:
    Obstacle(const glm::vec3& pos, const glm::vec3& size, 
             const glm::vec3& clr = glm::vec3(0.6f, 0.3f, 0.1f))
        : Entity(pos, size, clr) {}
    
    void render() override {
        glPushMatrix();
        glTranslatef(position.x, position.y, position.z);
        glColor3f(color.r, color.g, color.b);
        glScalef(scale.x, scale.y, scale.z);
        
        // Draw a cube
        glutSolidCube(1.0f);
        
        glPopMatrix();
    }
};

// Camera class for 3D navigation
class Camera {
private:
    glm::vec3 target;
    float radius;
    float theta;
    float phi;
    
public:
    Camera(const glm::vec3& tgt = glm::vec3(0.0f), float r = 10.0f, 
           float t = 0.0f, float p = glm::radians(45.0f))
        : target(tgt), radius(r), theta(t), phi(p) {}
    
    void rotate(float deltaTheta, float deltaPhi) {
        theta += deltaTheta;
        phi += deltaPhi;
        
        // Clamp phi to avoid flipping
        phi = glm::clamp(phi, glm::radians(5.0f), glm::radians(175.0f));
    }
    
    void zoom(float deltaRadius) {
        radius += deltaRadius;
        radius = glm::max(radius, 2.0f);
    }
    
    void apply() {
        // Convert spherical coordinates to Cartesian
        float x = target.x + radius * sin(phi) * cos(theta);
        float y = target.y + radius * cos(phi);
        float z = target.z + radius * sin(phi) * sin(theta);
        
        gluLookAt(x, y, z,
                  target.x, target.y, target.z,
                  0.0f, 1.0f, 0.0f);
    }
};

// Multi-Agent World class
class MultiAgentWorld {
private:
    std::vector<std::unique_ptr<Entity>> entities;
    std::vector<Agent*> agents;  // Non-owning pointers for easy access
    std::vector<Obstacle*> obstacles;  // Non-owning pointers
    Camera camera;
    float worldSize;
    bool pathsPlanned;
    bool agentsMoving;
    SimulationState simState;
    
public:
    MultiAgentWorld(float size = 20.0f) : worldSize(size), pathsPlanned(false), agentsMoving(false) {
        // Initialize APF strategy
        simState.apfStrategy = new PathPlanning::APFMAPFStrategy();
        setupMultiAgentScenario();
    }
    
    ~MultiAgentWorld() { 
        delete simState.apfStrategy;
    }
    
    void setupMultiAgentScenario() {
        // Clear existing entities
        entities.clear();
        agents.clear();
        obstacles.clear();
        pathsPlanned = false;
        agentsMoving = false;
        
        // Define agent colors
        std::vector<glm::vec3> agentColors = {
            glm::vec3(1.0f, 0.2f, 0.2f), // Red
            glm::vec3(0.2f, 1.0f, 0.2f), // Green
            glm::vec3(0.2f, 0.2f, 1.0f), // Blue
            glm::vec3(1.0f, 1.0f, 0.2f), // Yellow
            glm::vec3(1.0f, 0.2f, 1.0f)  // Magenta
        };
        
        // Create multiple agents with challenging scenarios
        float cornerOffset = worldSize * 0.4f;
        
        // Agent 0: Bottom-left to top-right
        auto agent0 = std::make_unique<Agent>(0,
            glm::vec3(-cornerOffset, 2.0f, -cornerOffset),
            glm::vec3(cornerOffset, worldSize * 0.8f, cornerOffset),
            agentColors[0]);
        agents.push_back(agent0.get());
        entities.push_back(std::move(agent0));
        
        // Agent 1: Bottom-right to top-left
        auto agent1 = std::make_unique<Agent>(1,
            glm::vec3(cornerOffset, 2.0f, -cornerOffset),
            glm::vec3(-cornerOffset, worldSize * 0.8f, cornerOffset),
            agentColors[1]);
        agents.push_back(agent1.get());
        entities.push_back(std::move(agent1));
        
        // Agent 2: Center-bottom to center-top
        auto agent2 = std::make_unique<Agent>(2,
            glm::vec3(0.0f, 2.0f, 0.0f),
            glm::vec3(0.0f, worldSize * 0.8f, 0.0f),
            agentColors[2]);
        agents.push_back(agent2.get());
        entities.push_back(std::move(agent2));
        
        // Create obstacles
        int obstaclesPlaced = 0;
        int maxObstacles = 10;
        int maxAttempts = 100;
        
        while (obstaclesPlaced < maxObstacles && maxAttempts > 0) {
            // Random position
            float x = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * worldSize * 0.8f;
            float y = static_cast<float>(rand()) / RAND_MAX * (worldSize * 0.6f) + 2.0f;
            float z = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * worldSize * 0.8f;
            glm::vec3 obstaclePos(x, y, z);
            
            // Random size
            float sizeX = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f;
            float sizeY = static_cast<float>(rand()) / RAND_MAX * 3.0f + 1.0f;
            float sizeZ = static_cast<float>(rand()) / RAND_MAX * 2.0f + 1.0f;
            
            // Check if obstacle is too close to any agent start/goal
            bool tooClose = false;
            float minDistance = 4.0f;
            
            for (const auto& agent : agents) {
                if (glm::length(obstaclePos - agent->getPosition()) < minDistance ||
                    glm::length(obstaclePos - agent->getGoalPosition()) < minDistance) {
                    tooClose = true;
                    break;
                }
            }
            
            if (!tooClose) {
                auto obstacle = std::make_unique<Obstacle>(obstaclePos, glm::vec3(sizeX, sizeY, sizeZ));
                obstacles.push_back(obstacle.get());
                entities.push_back(std::move(obstacle));
                obstaclesPlaced++;
            }
            maxAttempts--;
        }
        
        std::cout << "Multi-Agent Scenario Setup:" << std::endl;
        for (size_t i = 0; i < agents.size(); ++i) {
            const auto& pos = agents[i]->getPosition();
            const auto& goal = agents[i]->getGoalPosition();
            std::cout << "  Agent " << i << ": (" << pos.x << ", " << pos.y << ", " << pos.z 
                      << ") -> (" << goal.x << ", " << goal.y << ", " << goal.z << ")" << std::endl;
        }
        std::cout << "  Obstacles: " << obstaclesPlaced << std::endl;
        
        // Configure simulation state for real-time force calculations
        simState.agents = agents;
        simState.obstacles.clear();
        for (const auto& obstacle : obstacles) {
            PathPlanning::BoundingBox bbox;
            bbox.center = obstacle->getPosition();
            bbox.size = glm::vec3(2.0f, 4.0f, 2.0f); // Match obstacle scale
            bbox.min = bbox.center - bbox.size * 0.5f;
            bbox.max = bbox.center + bbox.size * 0.5f;
            simState.obstacles.push_back(bbox);
        }
        
        // Configure APF strategy
        PathPlanning::Config config;
        config.attractiveForceGain = 1.0f;
        config.repulsiveForceGain = 1.0f;
        config.influenceRadius = 3.0f;
        config.stepSize = 0.1f;
        config.maxIterations = 1000;
        config.goalTolerance = 0.2f;
        config.apfExponentialDecay = 0.8f;
        config.apfInverseSquareScale = 1.0f;
        simState.apfStrategy->configure(config);
        
        // Link agents to simulation state
        for (auto& agent : agents) {
            agent->setSimulationState(&simState);
        }
        
        // Setup camera to look at the center of action (midpoint between agents)
        glm::vec3 cameraTarget(0.0f, worldSize * 0.4f, 0.0f); // Focus on middle height where agents are
        camera = Camera(cameraTarget, worldSize * 1.5f, 0.0f, glm::radians(45.0f));
    }
    
    void triggerMultiAgentPathPlanning() {
        if (pathsPlanned) {
            std::cout << "Paths already planned. Press 'R' to reset scenario." << std::endl;
            return;
        }
        
        std::cout << "\n=== MULTI-AGENT PATH PLANNING ===" << std::endl;
        
        // Create environments for each agent
        std::vector<PathPlanning::Environment> environments;
        
        for (const auto& agent : agents) {
            PathPlanning::Environment env;
            env.agentStart = agent->getPosition();
            env.goalPosition = agent->getGoalPosition();
            env.agentRadius = 0.5f;
            env.goalTolerance = 0.5f;
            
            // Set world bounds
            env.worldBounds.min = glm::vec3(-worldSize/2, 0.0f, -worldSize/2);
            env.worldBounds.max = glm::vec3(worldSize/2, worldSize, worldSize/2);
            
            // Add obstacles
            for (const auto& obstacle : obstacles) {
                PathPlanning::BoundingBox bbox;
                bbox.center = obstacle->getPosition();
                bbox.size = glm::vec3(2.0f, 3.0f, 2.0f); // Approximate size
                bbox.min = bbox.center - bbox.size * 0.5f;
                bbox.max = bbox.center + bbox.size * 0.5f;
                env.obstacles.push_back(bbox);
            }
            
            environments.push_back(env);
        }
        
        // Enable MAPF and set configuration
        PathPlanning::Config config;
        config.enableMAPF = true;
        config.algorithm = PathPlanning::Config::Algorithm::APF_MAPF;
        config.stepSize = 0.1f;
        config.maxIterations = 1000;
        config.attractiveForceGain = 1.0f;
        config.repulsiveForceGain = 1.0f;
        config.influenceRadius = 3.0f;
        PathPlanning::setConfig(config);
        
        // Plan paths for all agents
        auto paths = PathPlanning::planMultiplePaths(environments);
        
        // Assign paths to agents
        for (size_t i = 0; i < std::min(paths.size(), agents.size()); ++i) {
            agents[i]->setPath(paths[i]);
            std::cout << "Agent " << i << " assigned path with " << paths[i].size() << " waypoints" << std::endl;
        }
        
        pathsPlanned = true;
        std::cout << "Multi-agent path planning completed!" << std::endl;
        std::cout << "Press 'S' to start agent movement" << std::endl;
    }
    
    void startAgentMovement() {
        if (!pathsPlanned) {
            std::cout << "Plan paths first (press 'P')" << std::endl;
            return;
        }
        
        for (auto& agent : agents) {
            agent->startMovement();
        }
        agentsMoving = true;
        std::cout << "All agents started moving!" << std::endl;
    }
    
    void stopAgentMovement() {
        for (auto& agent : agents) {
            agent->stopMovement();
        }
        agentsMoving = false;
        std::cout << "All agents stopped" << std::endl;
    }
    
void switchStrategy() {
    // Toggle between algorithms
    static int algorithmIndex = 0;
    
    PathPlanning::Config config = PathPlanning::getConfig();
    
    const char* algorithmNames[] = {
        "APF MAPF (Multi-Agent Potential Fields)",
        "ORCA (Optimal Reciprocal Collision Avoidance)",
        "APF (Sequential Planning)"
    };
    
    algorithmIndex = (algorithmIndex + 1) % 3;
    
    switch (algorithmIndex) {
        case 0:
            config.algorithm = PathPlanning::Config::Algorithm::APF_MAPF;
            break;
        case 1:
            config.algorithm = PathPlanning::Config::Algorithm::ORCA;
            break;
        case 2:
            config.algorithm = PathPlanning::Config::Algorithm::APF;
            break;
    }
    
    std::cout << "Switched to: " << algorithmNames[algorithmIndex] << std::endl;
    PathPlanning::setConfig(config);
    
    pathsPlanned = false;
    stopAgentMovement();
    std::cout << "Press 'P' to plan paths with new strategy" << std::endl;
}
    
    void toggleForceMode() {
        simState.useRealTimeForces = !simState.useRealTimeForces;
        if (simState.useRealTimeForces) {
            std::cout << "Switched to Real-Time Force Calculations" << std::endl;
            std::cout << "Agents will use APF forces in real-time!" << std::endl;
        } else {
            std::cout << "Switched to Waypoint Following" << std::endl;
            std::cout << "Agents will follow pre-calculated paths" << std::endl;
        }
    }
    
    void update(float deltaTime) {
        for (auto& entity : entities) {
            entity->update(deltaTime);
        }
        
        // Check if all agents reached their goals
        if (agentsMoving) {
            bool allReached = true;
            for (const auto& agent : agents) {
                if (!agent->getHasReachedGoal()) {
                    allReached = false;
                    break;
                }
            }
            
            if (allReached) {
                std::cout << "All agents reached their goals!" << std::endl;
                agentsMoving = false;
            }
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
        for (const auto& entity : entities) {
            entity->render();
        }
        
        // Draw UI text
        renderUI();
        
        glutSwapBuffers();
    }
    
    void renderWorldBoundaries() {
        glColor3f(0.5f, 0.5f, 0.5f);
        glLineWidth(1.0f);
        
        float half = worldSize / 2.0f;
        
        // Draw grid on the floor
        glBegin(GL_LINES);
        for (int i = -10; i <= 10; ++i) {
            float pos = i * (worldSize / 20.0f);
            // X lines
            glVertex3f(-half, 0.0f, pos);
            glVertex3f(half, 0.0f, pos);
            // Z lines
            glVertex3f(pos, 0.0f, -half);
            glVertex3f(pos, 0.0f, half);
        }
        glEnd();
        
        // Draw world box
        glBegin(GL_LINE_LOOP);
        glVertex3f(-half, 0.0f, -half);
        glVertex3f(half, 0.0f, -half);
        glVertex3f(half, 0.0f, half);
        glVertex3f(-half, 0.0f, half);
        glEnd();
        
        glBegin(GL_LINE_LOOP);
        glVertex3f(-half, worldSize, -half);
        glVertex3f(half, worldSize, -half);
        glVertex3f(half, worldSize, half);
        glVertex3f(-half, worldSize, half);
        glEnd();
        
        // Vertical edges
        glBegin(GL_LINES);
        glVertex3f(-half, 0.0f, -half); glVertex3f(-half, worldSize, -half);
        glVertex3f(half, 0.0f, -half); glVertex3f(half, worldSize, -half);
        glVertex3f(half, 0.0f, half); glVertex3f(half, worldSize, half);
        glVertex3f(-half, 0.0f, half); glVertex3f(-half, worldSize, half);
        glEnd();
    }
    
    void renderUI() {
        // Switch to 2D rendering for UI
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT), -1, 1);
        
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        
        glDisable(GL_DEPTH_TEST);
        glColor3f(1.0f, 1.0f, 1.0f);
        
        // Draw instructions
        glRasterPos2f(10, glutGet(GLUT_WINDOW_HEIGHT) - 20);
        std::string instructions = "Controls: P=Plan Paths, S=Start, T=Stop, A=Switch Algorithm, F=Toggle Forces, R=Reset, ESC=Exit";
        for (char c : instructions) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
        
        // Draw status
        glRasterPos2f(10, glutGet(GLUT_WINDOW_HEIGHT) - 40);
        std::string status = "Status: ";
        if (!pathsPlanned) status += "Ready for planning";
        else if (agentsMoving) status += "Agents moving";
        else status += "Paths planned - press S to start";
        
        for (char c : status) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
        
        glEnable(GL_DEPTH_TEST);
        
        // Restore matrices
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    
    Camera& getCamera() { return camera; }
};

// Global variables
MultiAgentWorld* world = nullptr;
bool mousePressed = false;
int lastMouseX = 0, lastMouseY = 0;
float lastFrameTime = 0.0f;
const float mouseSensitivity = 0.01f;

// Function declarations
void display();
void reshape(int width, int height);
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void idle();
void cleanup();

int main(int argc, char** argv) {
    // Initialize random seed
    srand(static_cast<unsigned int>(time(nullptr)));
    
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1200, 800);
    glutCreateWindow("Multi-Agent Path Planning Simulation");
    
    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }
    
    // Setup OpenGL
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    
    // Setup lighting
    GLfloat lightPos[] = {10.0f, 20.0f, 10.0f, 1.0f};
    GLfloat lightAmbient[] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat lightDiffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    
    // Initialize path planning module
    PathPlanning::initialize();
    
    // Create world
    world = new MultiAgentWorld(20.0f);
    
    // Setup GLUT callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);
    glutIdleFunc(idle);
    
    std::cout << "\n=== MULTI-AGENT PATH PLANNING SIMULATION ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  P - Plan paths for all agents" << std::endl;
    std::cout << "  S - Start agent movement" << std::endl;
    std::cout << "  T - Stop agent movement" << std::endl;
    std::cout << "  A - Switch between APF MAPF and Regular APF and ORCA " << std::endl;
    std::cout << "  F - Toggle between Real-Time Forces and Waypoint Following" << std::endl;
    std::cout << "  R - Reset scenario" << std::endl;
    std::cout << "  +/- - Zoom in/out" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;
    std::cout << "  Mouse drag - Rotate camera" << std::endl;
    
    // Start main loop
    glutMainLoop();
    
    return 0;
}

void display() {
    if (world) {
        world->render();
    }
}

void reshape(int width, int height) {
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
                world = new MultiAgentWorld(20.0f);
            }
            break;
        case 'c':
        case 'C':
            // Reset camera to default position
            if (world) {
                world->getCamera() = Camera(glm::vec3(0.0f, 8.0f, 0.0f), 30.0f, 0.0f, glm::radians(45.0f));
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
            // Trigger multi-agent path planning
            if (world) {
                world->triggerMultiAgentPathPlanning();
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
        case 'a':
        case 'A':
            // Switch algorithm
            if (world) {
                world->switchStrategy();
            }
            break;
        case 'f':
        case 'F':
            // Toggle force mode
            if (world) {
                world->toggleForceMode();
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
