#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

constexpr double normalHz = 60.0;
constexpr double fastHz = 240.0;
constexpr double frameDt = 1.0 / normalHz;
constexpr double fastDt = 1.0 / fastHz;
constexpr int substeps = static_cast<int>(fastHz / normalHz);
static_assert(normalHz * substeps == fastHz);

constexpr double shotSpeed = 2000.0;
constexpr double shotRestitution = 0.75;
constexpr double wallX = 400.0;
constexpr double tolerance = 1e-7;
constexpr double minBounceSpeed = 120.0;
constexpr int contactPasses = 32;
constexpr int maxCCDEvents = 16;

enum class Mode { Discrete, CCD, MultiRate };
enum class Scene { Wall, Stack, Mixed };
enum Group { Fixed, Normal, Fast };

struct Vec {
    double x = 0, y = 0;
    Vec operator+(Vec b) const { return {x + b.x, y + b.y}; }
    Vec operator-(Vec b) const { return {x - b.x, y - b.y}; }
    Vec operator*(double k) const { return {x * k, y * k}; }
    double dot(Vec b) const { return x * b.x + y * b.y; }
};

struct Body {
    Vec position;
    Vec half;
    Vec velocity;
    double inverseMass;
    double restitution;
    double positionTime = 0; // Момент внутри шага, к которому относится position.

    Vec positionAt(double time) const {
        return position + velocity * (time - positionTime);
    }

    void advanceTo(double time) {
        position = positionAt(time);
        positionTime = time;
    }
};

struct World {
    std::array<std::vector<Body>, 3> groups;
    double gravity = 0;
    int frames = 0, fastSteps = 0;
    int wallHits = 0, mixedHits = 0, firstMixedSubstep = 0, ccdLimits = 0;
};

inline World makeWorld(Scene scene) {
    World world;
    if (scene == Scene::Wall) {
        world.groups[Fixed].push_back({{wallX, 250}, {5, 13}, {}, 0, 0});
        world.groups[Fast].push_back({{wallX - 20, 250}, {4, 4}, {shotSpeed, 0}, 4, shotRestitution});
        return world;
    }

    world.gravity = 950;
    const bool mixed = scene == Scene::Mixed;
    const auto heights = mixed ? std::array{4, 3, 3} : std::array{5, 3, 2};
    const auto stackX = mixed ? std::array{480.0, 200.0, 760.0} : std::array{200.0, 480.0, 760.0};
    for (std::size_t stack = 0; stack < heights.size(); ++stack) {
        const double x = stackX[stack];
        world.groups[Fixed].push_back({{x, 460}, {34, 34}, {}, 0, 0});
        for (int i = 0; i < heights[stack]; ++i) {
            const double y = mixed ? 409.0 - 34 * i : 406.0 - 35 * i;
            world.groups[Normal].push_back({{x, y}, {17, 17}, {}, 1, 0});
        }
    }
    if (mixed) {
        const Vec target = world.groups[Normal].front().position;
        world.groups[Fast].push_back({{target.x - 34, target.y}, {4, 4}, {shotSpeed, 0}, 4, shotRestitution});
    }
    return world;
}

template<class Function>
void forPairs(World& world, bool fastOnly, Function visit) {
    const int startGroup = fastOnly ? Fast : Normal;
    for (int secondGroup = startGroup; secondGroup <= Fast; ++secondGroup) {
        auto& secondBodies = world.groups[secondGroup];
        for (int firstGroup = Fixed; firstGroup <= secondGroup; ++firstGroup) {
            auto& firstBodies = world.groups[firstGroup];
            for (std::size_t i = 0; i < firstBodies.size(); ++i) {
                // Внутри одной группы каждую пару проверяем один раз.
                const std::size_t startBody = firstGroup == secondGroup ? i + 1 : 0;
                for (std::size_t j = startBody; j < secondBodies.size(); ++j) {
                    visit(firstBodies[i], secondBodies[j], firstGroup, secondGroup);
                }
            }
        }
    }
}

inline void impulse(World& world, Body& a, Body& b, Vec normal,
                    int firstGroup, int secondGroup, int substep) {
    const double closingSpeed = (a.velocity - b.velocity).dot(normal);
    if (closingSpeed <= tolerance) return;

    const double restitution = closingSpeed > minBounceSpeed ? std::max(a.restitution, b.restitution) : 0;
    const double magnitude = (1 + restitution) * closingSpeed / (a.inverseMass + b.inverseMass);
    const Vec contactImpulse = normal * magnitude;
    a.velocity = a.velocity - contactImpulse * a.inverseMass;
    b.velocity = b.velocity + contactImpulse * b.inverseMass;

    if (secondGroup == Fast && firstGroup == Fixed) {
        ++world.wallHits;
    }
    if (secondGroup == Fast && firstGroup == Normal) {
        ++world.mixedHits;
        if (world.mixedHits == 1) {
            world.firstMixedSubstep = substep;
        }
    }
}

inline void solveContacts(World& world, double time, bool fastOnly, int substep = 0) {
    // Повторные проходы передают импульсы по всему стеку.
    for (int pass = 0; pass < contactPasses; ++pass) {
        forPairs(world, fastOnly, [&](Body& a, Body& b, int firstGroup, int secondGroup) {
            const Vec separation = b.positionAt(time) - a.positionAt(time);
            const double overlapX = a.half.x + b.half.x - std::abs(separation.x);
            const double overlapY = a.half.y + b.half.y - std::abs(separation.y);
            if (overlapX < -tolerance || overlapY < -tolerance) return;

            const Vec normal = overlapX < overlapY
                ? Vec{std::copysign(1.0, separation.x), 0}
                : Vec{0, std::copysign(1.0, separation.y)};

            // В смешанной паре обе координаты фиксируются до изменения скоростей.
            a.advanceTo(time);
            b.advanceTo(time);
            impulse(world, a, b, normal, firstGroup, secondGroup, substep);

            const double depth = std::max(0.0, std::min(overlapX, overlapY) - 0.001);
            const Vec correction = normal * (0.8 * depth / (a.inverseMass + b.inverseMass));
            a.position = a.position - correction * a.inverseMass;
            b.position = b.position + correction * b.inverseMass;
        });
    }
}

inline bool sweep(const Body& a, const Body& b, double limit, double& time, Vec& normal) {
    const Vec separation = b.position - a.position;
    const Vec relativeVelocity = b.velocity - a.velocity;
    const Vec halfSum = a.half + b.half;
    if (std::abs(separation.x) <= halfSum.x + tolerance &&
        std::abs(separation.y) <= halfSum.y + tolerance) {
        return false; // Уже соприкасающиеся тела обрабатывает solveContacts.
    }

    double enter = -std::numeric_limits<double>::infinity();
    double exit = limit;
    auto intersectAxis = [&](double gap, double speed, double half, Vec direction) {
        if (std::abs(speed) < tolerance) return std::abs(gap) <= half;

        const double t1 = (-half - gap) / speed;
        const double t2 = (half - gap) / speed;
        const double axisEnter = std::min(t1, t2);
        if (axisEnter > enter) {
            enter = axisEnter;
            normal = direction * (speed < 0 ? 1 : -1);
        }
        exit = std::min(exit, std::max(t1, t2));
        return enter <= exit;
    };
    if (!intersectAxis(separation.x, relativeVelocity.x, halfSum.x, {1, 0}) ||
        !intersectAxis(separation.y, relativeVelocity.y, halfSum.y, {0, 1}) || enter < 0) {
        return false;
    }
    time = enter;
    return true;
}

inline void advanceGroup(World& world, Group group, double time) {
    for (auto& body : world.groups[group]) {
        body.advanceTo(time);
    }
}

inline void addGravity(World& world, Group group, double dt) {
    for (auto& body : world.groups[group]) {
        body.velocity.y += world.gravity * dt;
    }
}

inline void stepMultiRate(World& world) {
    for (int substep = 1; substep <= substeps; ++substep) {
        const double time = substep * fastDt;
        addGravity(world, Fast, fastDt);
        advanceGroup(world, Fast, time);

        // Обычные тела продвигаются вместе только на последнем подшаге.
        const bool lastSubstep = substep == substeps;
        if (lastSubstep) {
            advanceGroup(world, Normal, time);
        }
        solveContacts(world, time, !lastSubstep, substep);
        ++world.fastSteps;
    }
}

struct Impact {
    double time;
    Body* first = nullptr;
    Body* second = nullptr;
    Vec normal{};
    int firstGroup = Fixed;
    int secondGroup = Fixed;
};

inline Impact firstImpact(World& world, double timeLeft) {
    Impact closest{timeLeft};
    forPairs(world, false, [&](Body& a, Body& b, int firstGroup, int secondGroup) {
        double time;
        Vec normal;
        if (sweep(a, b, closest.time, time, normal) && (!closest.first || time < closest.time)) {
            closest = {time, &a, &b, normal, firstGroup, secondGroup};
        }
    });
    return closest;
}

inline void processCCDImpacts(World& world) {
    double elapsed = 0;
    solveContacts(world, elapsed, false);

    for (int event = 1; event <= maxCCDEvents && elapsed < frameDt; ++event) {
        const Impact impact = firstImpact(world, frameDt - elapsed);
        if (!impact.first) break;

        elapsed += impact.time;
        advanceGroup(world, Normal, elapsed);
        advanceGroup(world, Fast, elapsed);
        impulse(world, *impact.first, *impact.second, impact.normal,
                impact.firstGroup, impact.secondGroup, 0);
        solveContacts(world, elapsed, false);
        if (event == maxCCDEvents && elapsed < frameDt) {
            ++world.ccdLimits;
        }
    }
}

inline void step(World& world, Mode mode) {
    ++world.frames;
    for (auto& group : world.groups) {
        for (auto& body : group) {
            body.positionTime = 0;
        }
    }
    addGravity(world, Normal, frameDt);

    if (mode == Mode::MultiRate) {
        stepMultiRate(world);
        return;
    }

    addGravity(world, Fast, frameDt);
    if (mode == Mode::CCD) {
        processCCDImpacts(world);
    }

    advanceGroup(world, Normal, frameDt);
    advanceGroup(world, Fast, frameDt);
    solveContacts(world, frameDt, false);
}
