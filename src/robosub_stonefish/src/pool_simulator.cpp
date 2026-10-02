#include <stonefish_ros2/ROS2SimulationManager.h>
#include <Stonefish/core/ConsoleSimulationApp.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/graphics/OpenGLTrackball.h>
#include <Stonefish/sensors/Sensor.h>
#include <glm/gtx/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {
volatile std::sig_atomic_t stopRequested = 0;
void RequestStop(int) { stopRequested = 1; }
void EnableSensors(sf::SimulationManager* manager) {
    // The upstream parser starts sensors with enable services disabled.
    for (unsigned int index = 0; auto* sensor = manager->getSensor(index); ++index) sensor->setEnabled(true);
}
}

class PoolApp : public sf::GraphicalSimulationApp {
public:
    PoolApp(const std::string& data, sf::RenderSettings settings, sf::SimulationManager* manager,
            float length, float width, float depth, rclcpp::Node::SharedPtr node,
            bool keyboard, bool pulses, float linearSpeed, float angularSpeed)
        : GraphicalSimulationApp("Duke RoboSub | Pool", data, settings, sf::HelperSettings(), manager),
          length_(length), width_(width), depth_(depth), pulses_(pulses),
          linearSpeed_(linearSpeed), angularSpeed_(angularSpeed) {
        if (keyboard) commandPublisher_ = node->create_publisher<geometry_msgs::msg::Twist>(
            "/simulation/crush/cmd_vel", 1);
    }

    void Start() {
        setMaxPhysicsThreads(2);
        Init();
        EnableSensors(getSimulationManager());
        ResetView(false);
        if (getSimulationManager()->getRobot("crush")) FollowRobot();
        HideHUD();
        StartSimulation();
    }

    void Tick() {
        LoopInternal();
        PublishKeyboardCommand();
    }

    static bool IsMotionKey(SDL_Keycode key) {
        switch (key) {
            case SDLK_w: case SDLK_s: case SDLK_a: case SDLK_d: case SDLK_q: case SDLK_e:
            case SDLK_LEFT: case SDLK_RIGHT: case SDLK_UP: case SDLK_DOWN: case SDLK_z: case SDLK_x:
                return true;
            default: return false;
        }
    }

    void KeyDown(SDL_Event* event) override {
        const auto key = event->key.keysym.sym;
        if (IsMotionKey(key)) {
            if (commandPublisher_) keys_[key] = std::chrono::steady_clock::now();
        } else if (key == SDLK_SPACE) keys_.clear();
        else if (key == SDLK_f) FollowRobot();
        else if (key == SDLK_r) ResetView(false);
        else if (key == SDLK_u) ResetView(true);
        else if (key != SDLK_ESCAPE) GraphicalSimulationApp::KeyDown(event);
    }

    void KeyUp(SDL_Event* event) override {
        if (IsMotionKey(event->key.keysym.sym)) {
            if (!pulses_) keys_.erase(event->key.keysym.sym);
        } else GraphicalSimulationApp::KeyUp(event);
    }

    void PublishKeyboardCommand() {
        if (!commandPublisher_) return;
        const auto now = std::chrono::steady_clock::now();
        for (auto iterator = keys_.begin(); iterator != keys_.end();) {
            // Allow a software-rendered frame to finish without dropping a live key.
            if ((pulses_ && now - iterator->second > std::chrono::milliseconds(500)) ||
                SDL_GetKeyboardFocus() == nullptr) iterator = keys_.erase(iterator);
            else ++iterator;
        }
        const auto axis = [&](SDL_Keycode positive, SDL_Keycode negative) {
            return static_cast<float>(keys_.count(positive)) - static_cast<float>(keys_.count(negative));
        };
        geometry_msgs::msg::Twist command;
        command.linear.x = linearSpeed_ * axis(SDLK_w, SDLK_s);
        command.linear.y = linearSpeed_ * axis(SDLK_d, SDLK_a);
        command.linear.z = linearSpeed_ * axis(SDLK_e, SDLK_q);
        command.angular.x = angularSpeed_ * axis(SDLK_x, SDLK_z);
        command.angular.y = angularSpeed_ * axis(SDLK_UP, SDLK_DOWN);
        command.angular.z = angularSpeed_ * axis(SDLK_RIGHT, SDLK_LEFT);
        commandPublisher_->publish(command);
    }

    void FollowRobot() {
        auto* robot = getSimulationManager()->getRobot("crush");
        if (!robot) return;
        auto* link = robot->getLink(size_t(0));
        const auto centre = link->getCGTransform().getOrigin();
        SetView(glm::vec3(centre.x(), centre.y(), centre.z()),
                glm::normalize(glm::vec3(0.7f, 1.f, 0.45f)), 1.8f);
        getSimulationManager()->getTrackball()->GlueToMoving(link);
    }

    void ResetView(bool underwater) {
        const float radius = underwater ? std::min(length_, width_) * 0.25f : std::max(length_, width_) * 1.35f;
        const glm::vec3 direction = underwater ? glm::normalize(glm::vec3(1.f, 0.45f, 0.08f))
                                              : glm::normalize(glm::vec3(0.45f, 0.7f, 0.85f));
        SetView(glm::vec3(0, 0, underwater ? depth_ * 0.5f : depth_ * 0.25f), direction, radius);
    }

    void SetView(glm::vec3 centre, glm::vec3 direction, float radius) {
        auto* camera = getSimulationManager()->getTrackball();
        camera->GlueToMoving(nullptr);
        // Trackball's default up is -Z (NED). Rotate that basis toward the desired view direction.
        const glm::quat basis = glm::rotation(glm::vec3(0, 0, -1.f), glm::vec3(0, 0, 1.f));
        const glm::vec3 right = glm::normalize(glm::cross(direction, glm::vec3(0, 0, -1.f)));
        const glm::vec3 up = glm::cross(right, direction);
        const glm::quat target = glm::quat_cast(glm::mat3(
            glm::vec3(-right.x, -direction.x, up.x),
            glm::vec3(-right.y, -direction.y, up.y),
            glm::vec3(-right.z, -direction.z, up.z)));
        camera->Rotate(glm::inverse(basis) * target);
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
    bool pulses_;
    float linearSpeed_, angularSpeed_;
    std::unordered_map<SDL_Keycode, std::chrono::steady_clock::time_point> keys_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPublisher_;
};

class PhysicsApp : public sf::ConsoleSimulationApp {
public:
    PhysicsApp(const std::string& data, sf::SimulationManager* manager)
        : ConsoleSimulationApp("Duke RoboSub | Physics", data, manager) {}
    void Start() {
        setMaxPhysicsThreads(2);
        Init();
        EnableSensors(getSimulationManager());
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
    const auto robotEnabled = node->declare_parameter<bool>("robot_enabled", false);
    const auto keyboard = node->declare_parameter<bool>("keyboard_teleop", true) && robotEnabled;
    const auto pulses = node->declare_parameter<bool>("keyboard_pulse_mode", false);
    const auto linearSpeed = node->declare_parameter<double>("linear_speed", 0.45);
    const auto angularSpeed = node->declare_parameter<double>("angular_speed", 0.5);
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
            graphical = std::make_shared<PoolApp>(data, settings, manager.get(), length, width, depth,
                                                  node, keyboard, pulses, linearSpeed, angularSpeed);
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
