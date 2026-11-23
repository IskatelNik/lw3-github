#include "physics.hpp"
#include <SFML/Graphics.hpp>

#include <bit>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>

constexpr unsigned width = 960, height = 580;
constexpr std::array modes{Mode::Discrete, Mode::CCD, Mode::MultiRate};
constexpr std::array scenes{Scene::Wall, Scene::Stack, Scene::Mixed};
const std::string normalRate = std::to_string(static_cast<int>(normalHz));
const std::string fastRate = std::to_string(static_cast<int>(fastHz));
const std::array modeNames{"Discrete " + normalRate + " Hz", "CCD " + normalRate + " Hz",
                           "Multi-rate " + normalRate + '/' + fastRate + " Hz"};
constexpr std::array sceneNames{"Thin wall", "Block stacks", "Fast vs normal"};

std::uint64_t recordState(std::uint64_t checksum, const World& world) {
    auto record = [&](double value) {
        checksum = (checksum ^ std::bit_cast<std::uint64_t>(value)) * 1099511628211ull;
    };
    for (const auto& group : world.groups) {
        for (const auto& body : group) {
            record(body.position.x);
            record(body.position.y);
            record(body.velocity.x);
            record(body.velocity.y);
        }
    }
    record(world.mixedHits);
    record(world.wallHits);
    record(world.ccdLimits);
    return checksum;
}

void measureScene(Scene scene, Mode mode) {
    using Clock = std::chrono::steady_clock;
    constexpr std::uint64_t initialChecksum = 14695981039346656037ull;
    constexpr std::array framesPerScene{1, 360, 60};
    const int sceneIndex = static_cast<int>(scene);
    const int frames = framesPerScene[sceneIndex];
    const bool wallScene = scene == Scene::Wall;
    const bool stackScene = scene == Scene::Stack;
    const bool mixedScene = scene == Scene::Mixed;
    const int runs = wallScene ? 2000 : 5;

    double elapsedMs = 0;
    double maxVerticalRange = 0;
    double peakStackSpeed = 0;
    double shotVx = 0;
    double targetVx = 0;
    double targetX = 0;
    int firstStepMixedHits = 0;

    bool deterministic = true;
    std::uint64_t firstChecksum = 0;
    World world;
    for (int run = 0; run < runs; ++run) {
        world = makeWorld(scene);
        std::uint64_t checksum = initialChecksum;
        const auto& blocks = world.groups[Normal];
        const double inf = std::numeric_limits<double>::infinity();
        std::vector<double> minY(blocks.size(), inf);
        std::vector<double> maxY(blocks.size(), -inf);

        for (int frame = 1; frame <= frames; ++frame) {
            const auto start = Clock::now();
            step(world, mode);
            elapsedMs += std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            checksum = recordState(checksum, world);

            if (frame == 1 && !stackScene) {
                shotVx = world.groups[Fast].front().velocity.x;
                if (mixedScene) {
                    const auto& target = blocks.front();
                    targetVx = target.velocity.x;
                    targetX = target.position.x;
                    firstStepMixedHits = world.mixedHits;
                }
            }

            // Считаем дрожание после трёх секунд, когда стеки уже осели.
            if (stackScene && frame >= static_cast<int>(3 * normalHz)) {
                for (std::size_t i = 0; i < blocks.size(); ++i) {
                    const auto& body = blocks[i];
                    minY[i] = std::min(minY[i], body.position.y);
                    maxY[i] = std::max(maxY[i], body.position.y);
                    maxVerticalRange = std::max(maxVerticalRange, maxY[i] - minY[i]);
                    peakStackSpeed = std::max(peakStackSpeed, std::hypot(body.velocity.x, body.velocity.y));
                }
            }
        }

        if (run == 0) {
            firstChecksum = checksum;
        }
        deterministic = deterministic && checksum == firstChecksum;
    }

    bool tunneled = false;
    if (wallScene && world.wallHits == 0) {
        const auto& shot = world.groups[Fast].front();
        tunneled = shot.position.x - shot.half.x > wallX + 5;
    }

    std::cout << sceneNames[sceneIndex] << ',' << modeNames[static_cast<int>(mode)] << ','
              << frames << ',' << runs << ',' << elapsedMs / (frames * runs) << ','
              << static_cast<double>(world.fastSteps) / frames << ',' << deterministic;

    // У каждой сцены свои измерения, остальные поля в CSV оставляем пустыми.
    auto field = [](bool include, auto value) {
        std::cout << ',';
        if (include) std::cout << value;
    };
    field(wallScene, world.wallHits);
    field(wallScene, tunneled);
    field(!stackScene, shotVx);
    field(mixedScene, firstStepMixedHits);
    field(mixedScene && world.firstMixedSubstep != 0, world.firstMixedSubstep);
    field(mixedScene, targetVx);
    field(mixedScene, targetX);
    field(stackScene, maxVerticalRange);
    field(stackScene, peakStackSpeed);
    std::cout << ',' << world.ccdLimits << '\n';
}

void measure() {
    std::cout << "scene,mode,frames_per_run,runs,ms_per_frame,fast_steps_per_frame,deterministic,"
                 "wall_hits,tunneled,shot_vx_after_frame_1,mixed_hits_after_frame_1,"
                 "first_mixed_substep,normal_vx_after_frame_1,normal_x_after_frame_1,"
                 "stack_range_px,stack_peak_speed_px_s,ccd_limits\n";
    std::cout << std::fixed << std::setprecision(6);
    for (Scene scene : scenes) {
        for (Mode mode : modes) {
            measureScene(scene, mode);
        }
    }
}

void box(sf::RenderTarget& target, float x, float y, float w, float h, sf::Color color) {
    sf::RectangleShape shape({w, h});
    shape.setPosition({x, y});
    shape.setFillColor(color);
    target.draw(shape);
}

void label(sf::RenderTarget& target, const sf::Font& font, const sf::String& text,
           float x, float y, unsigned size, sf::Color color = sf::Color(229, 238, 247)) {
    sf::Text shape(font, text, size);
    shape.setPosition({x, y});
    shape.setFillColor(color);
    target.draw(shape);
}

void draw(sf::RenderTarget& target, const sf::Font& font, const World& world, Scene scene, Mode mode, bool playing) {
    const sf::Color muted(155, 175, 193), line(37, 56, 75), accent(94, 223, 204), warning(255, 125, 121);
    const std::array colors{sf::Color(113, 137, 160), sf::Color(93, 170, 250), sf::Color(255, 185, 79)};
    target.clear(sf::Color(16, 27, 40));
    box(target, 0, 0, width, 80, sf::Color(22, 36, 52));
    box(target, 0, 79, width, 1, line);
    label(target, font, "1  Discrete     2  CCD     3  Multi-rate", 28, 13, 17, muted);
    label(target, font, "F1  Thin wall     F2  Block stacks     F3  Fast vs normal", 28, 42, 17, muted);
    label(target, font, modeNames[static_cast<int>(mode)], 704, 10, 21, accent);
    label(target, font, playing ? "RUNNING" : "PAUSED", 704, 42, 16, muted);

    constexpr double fieldLeft = 40, fieldRight = 920, fieldTop = 171, fieldBottom = 503;
    if (scene == Scene::Wall) {
        const auto& shot = world.groups[Fast].front();
        box(target, 80, 323, 800, 1, muted);
        label(target, font, "8 px projectile", 316, 222, 16, colors[Fast]);
        label(target, font, "10 px wall", 447, 195, 16, colors[Fixed]);
        if (world.frames > 0) {
            box(target, 320 - 32, 323 - 32, 64, 64, sf::Color(103, 86, 64, 165));
            label(target, font, "Start", 305, 361, 15, muted);
        }
        std::string status = "Initial velocity: " + std::to_string(static_cast<int>(shotSpeed)) + " px/s";
        sf::Color statusColor = muted;
        if (world.wallHits > 0) {
            status = "Contact detected  /  projectile bounced";
            statusColor = accent;
        } else if (shot.position.x - shot.half.x > wallX + 5) {
            status = "No contact detected  /  projectile tunneled";
            statusColor = warning;
        }
        label(target, font, status, 60, 454, 23, statusColor);
    }

    for (int group = Fixed; group <= Fast; ++group) {
        for (const auto& body : world.groups[group]) {
            Vec center = body.position;
            Vec half = body.half;
            if (scene == Scene::Wall) {
                center = {480 + (center.x - wallX) * 8, 323 + (center.y - 250) * 8};
                half = half * 8;
            }
            const Vec start = center - half;
            const Vec end = center + half;
            if (start.x < fieldLeft || end.x > fieldRight || start.y < fieldTop || end.y > fieldBottom) continue;
            const float x = static_cast<float>(start.x), y = static_cast<float>(start.y);
            const float w = static_cast<float>(end.x - start.x), hPixels = static_cast<float>(end.y - start.y);
            box(target, x, y, w, hPixels, sf::Color(39, 64, 88));
            box(target, x + 1, y + 1, w - 2, hPixels - 2, colors[group]);
        }
    }

    box(target, 0, 516, width, 64, sf::Color(22, 36, 52));
    box(target, 0, 516, width, 1, line);
    label(target, font, L"Space  Run / pause     \u2192  1 frame     R  Reset", 28, 531, 18);
    label(target, font, "Frame " + std::to_string(world.frames) + "    |    " + fastRate + " Hz steps " +
          std::to_string(world.fastSteps), 28, 554, 14, muted);
}

int main(int argc, char** argv) try {
    if (argc == 2 && std::string_view(argv[1]) == "--bench") {
        measure();
        return 0;
    }

    const sf::Font font("C:/Windows/Fonts/segoeui.ttf");
    sf::RenderWindow window(sf::VideoMode({width, height}), "LW3",
                            sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(60);
    window.setKeyRepeatEnabled(false);

    Scene scene = Scene::Wall;
    Mode mode = Mode::Discrete;
    World world = makeWorld(scene);
    sf::Clock clock;
    double accumulator = 0;
    bool playing = false;
    auto resetScene = [&] {
        world = makeWorld(scene);
        accumulator = 0;
        playing = false;
    };

    while (window.isOpen()) {
        const double elapsed = std::min(0.1, static_cast<double>(clock.restart().asSeconds()));
        while (const auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
            const auto* key = event->getIf<sf::Event::KeyPressed>();
            if (!key) continue;
            using Key = sf::Keyboard::Scancode;
            const auto code = key->scancode;
            switch (code) {
                case Key::Escape:
                    window.close();
                    break;
                case Key::Space:
                    playing = !playing;
                    accumulator = 0;
                    break;
                case Key::Right:
                    if (!playing) step(world, mode);
                    break;
                case Key::Num1:
                case Key::Num2:
                case Key::Num3:
                    mode = modes[static_cast<int>(code) - static_cast<int>(Key::Num1)];
                    resetScene();
                    break;
                case Key::F1:
                case Key::F2:
                case Key::F3:
                    scene = scenes[static_cast<int>(code) - static_cast<int>(Key::F1)];
                    resetScene();
                    break;
                case Key::R:
                    resetScene();
                    break;
                default:
                    break;
            }
        }
        if (!window.isOpen()) break;
        if (playing) accumulator += elapsed;
        while (accumulator >= frameDt) {
            step(world, mode);
            accumulator -= frameDt;
        }
        draw(window, font, world, scene, mode, playing);
        window.display();
    }
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
