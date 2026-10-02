// Load the real Stonefish parser and raycast against the pool's collision boundaries.
#include <core/ConsoleSimulationApp.h>
#include <core/ScenarioParser.h>
#include <core/SimulationManager.h>
#include <entities/Entity.h>
#include <entities/forcefields/Ocean.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

class PoolManager : public sf::SimulationManager {
public:
    explicit PoolManager(const std::string& scenario)
        : SimulationManager(100.0, sf::Solver::SI, sf::CollisionFilter::EXCLUSIVE), scenario_(scenario) {}

    void BuildScenario() override {
        sf::ScenarioParser parser(this);
        if (!parser.Parse(scenario_)) {
            parser.SaveLog("/tmp/robosub_pool_parser.log");
            throw std::runtime_error("Scenario parsing failed; see /tmp/robosub_pool_parser.log");
        }
    }

private:
    std::string scenario_;
};

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: pool_smoke SCENARIO LENGTH WIDTH DEPTH\n";
        return 2;
    }
    try {
        const std::string scene = std::filesystem::absolute(argv[1]).string();
        const double length = std::stod(argv[2]), width = std::stod(argv[3]), depth = std::stod(argv[4]);
        PoolManager manager(scene);
        sf::ConsoleSimulationApp app("Pool smoke test", std::filesystem::path(scene).parent_path().string() + "/", &manager);
        manager.RestartScenario();
        if (!manager.isOceanEnabled()) throw std::runtime_error("Water physics missing");
        const sf::Vector3 start(0, 0, depth / 2);
        const std::vector<std::pair<sf::Vector3, sf::Vector3>> rays = {
            {{length, 0, depth / 2}, {length / 2, 0, depth / 2}},
            {{-length, 0, depth / 2}, {-length / 2, 0, depth / 2}},
            {{0, width, depth / 2}, {0, width / 2, depth / 2}},
            {{0, -width, depth / 2}, {0, -width / 2, depth / 2}},
            {{0, 0, depth * 2}, {0, 0, depth}},
        };
        for (const auto& [end, expected] : rays) {
            btCollisionWorld::ClosestRayResultCallback result(start, end);
            result.m_collisionFilterGroup = sf::MASK_DYNAMIC;
            result.m_collisionFilterMask = sf::MASK_STATIC;
            manager.getDynamicsWorld()->rayTest(start, end, result);
            if (!result.hasHit() || (result.m_hitPointWorld - expected).length() > 0.005) {
                throw std::runtime_error("Pool collision boundary missing or misplaced");
            }
        }
        if (!manager.StartSimulation()) throw std::runtime_error("Physics could not start");
        for (int i = 0; i < 100; ++i) manager.StepSimulation(0.01);
        manager.StopSimulation();
        std::cout << "PASS: parsed pool, water enabled, five collision boundaries, 100 physics steps\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
