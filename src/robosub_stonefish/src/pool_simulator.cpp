#include <stonefish_ros2/ROS2SimulationManager.h>
#include <Stonefish/core/ConsoleSimulationApp.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/graphics/OpenGLTrackball.h>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
volatile std::sig_atomic_t stopRequested = 0;
void RequestStop(int) { stopRequested = 1; }
}

class PoolApp : public sf::GraphicalSimulationApp {
public:
    PoolApp(const std::string& data, sf::RenderSettings settings, sf::SimulationManager* manager,
            float length, float width, float depth)
        : GraphicalSimulationApp("Duke RoboSub | Pool", data, settings, sf::HelperSettings(), manager),
          length_(length), width_(width), depth_(depth) {}

    void Start() {
        setMaxPhysicsThreads(2);
        Init();
        ResetView(false);
        HideHUD();
        StartSimulation();
    }

    void Tick() { LoopInternal(); }

    void KeyDown(SDL_Event* event) override {
        if (event->key.keysym.sym == SDLK_r) ResetView(false);
        else if (event->key.keysym.sym == SDLK_u) ResetView(true);
        else if (event->key.keysym.sym != SDLK_ESCAPE) GraphicalSimulationApp::KeyDown(event);
    }

    void ResetView(bool underwater) {
        auto* camera = getSimulationManager()->getTrackball();
        camera->GlueToMoving(nullptr);
        const float radius = underwater ? std::min(length_, width_) * 0.25f : std::max(length_, width_) * 1.35f;
        const glm::vec3 direction = underwater ? glm::normalize(glm::vec3(1.f, 0.45f, 0.08f))
                                              : glm::normalize(glm::vec3(0.45f, 0.7f, 0.85f));
        // Trackball's default up is -Z (NED). Rotate that basis toward the desired view direction.
        const glm::quat basis = glm::rotation(glm::vec3(0, 0, -1.f), glm::vec3(0, 0, 1.f));
        const glm::vec3 right = glm::normalize(glm::cross(direction, glm::vec3(0, 0, -1.f)));
        const glm::vec3 up = glm::cross(right, direction);
        const glm::quat target = glm::quat_cast(glm::mat3(
            glm::vec3(-right.x, -direction.x, up.x),
            glm::vec3(-right.y, -direction.y, up.y),
            glm::vec3(-right.z, -direction.z, up.z)));
        camera->Rotate(glm::inverse(basis) * target);
        const glm::vec3 centre(0, 0, underwater ? depth_ * 0.5f : depth_ * 0.25f);
        const glm::vec3 oldCentre = camera->GetEyePosition() + currentRadius_ * camera->GetLookingDirection();
        camera->MoveCenter(centre - oldCentre);
        // The engine exposes zoom through a multiplicative scroll operation.
        camera->MouseScroll(15.f * (radius / currentRadius_ - 1.f));
        currentRadius_ = radius;
        camera->UpdateTransform();
    }

    void MouseScroll(SDL_Event* event) override {
        currentRadius_ = std::max(0.05f, currentRadius_ * (1.f - event->wheel.y / 15.f));
    }

private:
    float length_, width_, depth_;
    float currentRadius_ = 5.f;
};

class PhysicsApp : public sf::ConsoleSimulationApp {
public:
    PhysicsApp(const std::string& data, sf::SimulationManager* manager)
        : ConsoleSimulationApp("Duke RoboSub | Physics", data, manager) {}
    void Start() {
        setMaxPhysicsThreads(2);
        Init();
        StartSimulation();
    }
};

int main(int argc, char** argv) {
    // Keep the ROS clock alive until the physics thread has stopped.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);
    auto node = std::make_shared<rclcpp::Node>("stonefish_simulator");
    const auto data = node->declare_parameter<std::string>("simulation_data", "");
    const auto scene = node->declare_parameter<std::string>("scenario_desc", "");
    const auto rate = node->declare_parameter<double>("simulation_rate", 100.0);
    const auto headless = node->declare_parameter<bool>("headless", false);
    sf::RenderSettings settings;
    settings.verticalSync = true;
    settings.windowW = node->declare_parameter<int>("window_res_x", 1280);
    settings.windowH = node->declare_parameter<int>("window_res_y", 720);
    const auto quality = node->declare_parameter<std::string>("rendering_quality", "low");
    const auto length = node->declare_parameter<double>("pool_length", 25.0);
    const auto width = node->declare_parameter<double>("pool_width", 15.0);
    const auto depth = node->declare_parameter<double>("pool_depth", 3.0);
    const auto renderRate = node->declare_parameter<double>("render_rate", 20.0);
    if (data.empty() || scene.empty() || !std::isfinite(rate) || rate <= 0 ||
        !std::isfinite(renderRate) || renderRate <= 0) {
        RCLCPP_ERROR(node->get_logger(), "Provide scenario_desc, simulation_data and positive update rates");
        rclcpp::shutdown();
        return 1;
    }
    auto renderQuality = quality == "high" ? sf::RenderQuality::HIGH :
                         quality == "medium" ? sf::RenderQuality::MEDIUM : sf::RenderQuality::LOW;
    settings.shadows = settings.atmosphere = settings.ocean = settings.aa = renderQuality;
    settings.ao = settings.ssr = sf::RenderQuality::DISABLED;

    auto manager = std::make_unique<sf::ROS2SimulationManager>(rate, scene, node);
    std::unique_ptr<sf::SimulationApp> app;
    std::shared_ptr<PoolApp> graphical;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    int result = 0;
    try {
        if (headless) {
            auto physics = std::make_unique<PhysicsApp>(data, manager.get());
            physics->Start();
            app = std::move(physics);
        } else {
            graphical = std::make_shared<PoolApp>(data, settings, manager.get(), length, width, depth);
            graphical->Start();
        }
        timer = node->create_wall_timer(std::chrono::duration<double>(headless ? 0.1 : 1.0 / renderRate), [&] {
            if (stopRequested) {
                executor.cancel();
                return;
            }
            if (graphical) {
                graphical->Tick();
                if (graphical->getState() == sf::SimulationState::FINISHED) executor.cancel();
            }
        });
        executor.spin();
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        result = 1;
    }
    if (timer) timer->cancel();
    timer.reset();
    if (graphical) graphical->StopSimulation();
    if (app) app->StopSimulation();
    // Destroy physics and GL scene resources while the app's context is still alive.
    manager.reset();
    graphical.reset();
    app.reset();
    SDL_Quit();
    if (rclcpp::ok()) rclcpp::shutdown();
    return result;
}
