#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <entt/entt.hpp>

struct Vec2 {
    double x, y;

    Vec2 operator+(const Vec2& other) const {
        return {x + other.x, y + other.y};
    }

    Vec2 operator-(const Vec2& other) const {
        return {x - other.x, y - other.y};
    }

    Vec2 operator*(double scalar) const {
        return {x * scalar, y * scalar};
    }

    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }
};

struct Sphere {
    Vec2 position;
    Vec2 velocity;
    double radius;
    std::string color;
};

const double g = 9.81;  // gravitační zrychlení

Vec2 computeDragSquerForce(const Vec2& velocity, double dragCoefficient) {
    double speed = velocity.magnitude();
    return velocity * (-dragCoefficient * speed);
}

Vec2 computeDragForce(const Vec2& velocity, double dragCoefficient) {
    return velocity * (-dragCoefficient);
}

void rungeKuttaStep(Sphere& sphere, double dt, double dragCoefficient, Vec2 (*computeDrag)(const Vec2&, double)) {
    Vec2 k1_v = computeDrag(sphere.velocity, dragCoefficient);
    Vec2 k1_r = sphere.velocity;

    Vec2 k2_v = computeDrag(sphere.velocity + k1_v * (0.5 * dt), dragCoefficient);
    Vec2 k2_r = sphere.velocity + k1_v * (0.5 * dt);

    Vec2 k3_v = computeDrag(sphere.velocity + k2_v * (0.5 * dt), dragCoefficient);
    Vec2 k3_r = sphere.velocity + k2_v * (0.5 * dt);

    Vec2 k4_v = computeDrag(sphere.velocity + k3_v * dt, dragCoefficient);
    Vec2 k4_r = sphere.velocity + k3_v * dt;

    sphere.velocity = sphere.velocity + (k1_v + k2_v * 2 + k3_v * 2 + k4_v) * (dt / 6.0);
    sphere.position = sphere.position + (k1_r + k2_r * 2 + k3_r * 2 + k4_r) * (dt / 6.0);

    sphere.velocity.y -= g * dt;
}

void eulerStep(Sphere& sphere, double dt, double dragCoefficient, Vec2 (*computeDrag)(const Vec2&, double)) {
    Vec2 dragForce = computeDrag(sphere.velocity, dragCoefficient);
    sphere.velocity = sphere.velocity + (dragForce - Vec2{0, g}) * dt;
    sphere.position = sphere.position + sphere.velocity * dt;
}

bool checkCollision(const Sphere& sphere1, const Sphere& sphere2) {
    double distance = (sphere1.position - sphere2.position).magnitude();
    return distance <= (sphere1.radius + sphere2.radius);
}

void writePOVRayFile(int frameNumber, const Sphere& targetSphere, const std::vector<Sphere>& spheres) {
    std::ofstream file("frame_" + std::to_string(frameNumber) + ".pov");
    file << "#include \"colors.inc\"\n";
    file << "camera { location <5, 4, -11> look_at <5, 4, 0> }\n";
    file << "light_source { <0, 10, -20> color rgb <1, 1, 1> }\n";
    file << "background { color White }\n";
    file << "union {\n";  // Start of the container object

    // Target Sphere
    file << "  sphere { <" << targetSphere.position.x << ", " << targetSphere.position.y << ", 0>, "
         << targetSphere.radius << " texture { pigment { color " << targetSphere.color << " } } }\n";

    // Other Spheres
    for (const auto& sphere : spheres) {
        file << "  sphere { <" << sphere.position.x << ", " << sphere.position.y << ", 0>, "
             << sphere.radius << " texture { pigment { color " << sphere.color << " } } }\n";
    }

    file << "}\n";  // End of the container object

    file.close();
}

void writeData(std::ofstream& file, const Sphere& sphere) {
    file << sphere.position.x << " " << sphere.position.y << "\n";
}

int main() {
    double dt = 0.01;      // časový krok
    double time = 0.0;     // počáteční čas
    double dragCoefficient = 0.1;  // Koeficient odporu
    int frameNumber = 0;

    // Otevření souborů pro ukládání trajektorie
    std::ofstream dataFileV2("sphere_trajectory_V2.txt");
    std::ofstream dataFileV1("sphere_trajectory_V1.txt");
    std::ofstream dataFileV0("sphere_trajectory_V0.txt");

    if (!dataFileV2.is_open() || !dataFileV1.is_open() || !dataFileV0.is_open()) {
        std::cerr << "Nepodařilo se otevřít soubor pro zápis!" << std::endl;
        return 1;
    }

    // EnTT registry
    entt::registry registry;

    // Vytvoření cílové koule (target)
    auto target = registry.create();
    registry.emplace<Sphere>(target, Vec2{10, 0}, Vec2{0, 0}, 0.5, "Red");

    // Vytvoření tří koulí (projectiles)
    auto sphere1 = registry.create();
    registry.emplace<Sphere>(sphere1, Vec2{0, 0}, Vec2{5, 10}, 0.5, "Blue");

    auto sphere2 = registry.create();
    registry.emplace<Sphere>(sphere2, Vec2{0, 0}, Vec2{5, 10}, 0.5, "Green");

    auto sphere3 = registry.create();
    registry.emplace<Sphere>(sphere3, Vec2{0, 0}, Vec2{5, 10}, 0.5, "Yellow");

    // Provádíme Euler krok PŘED začátkem smyčky
    eulerStep(registry.get<Sphere>(sphere1), dt, dragCoefficient, computeDragSquerForce);
    eulerStep(registry.get<Sphere>(sphere2), dt, dragCoefficient, computeDragForce);
    eulerStep(registry.get<Sphere>(sphere3), dt, 0.0f, computeDragForce);

    // Simulace pohybu
    while (time < 3.0) {  // Simulace na 3 sekundy
        auto& targetSphere = registry.get<Sphere>(target);
        auto& s1 = registry.get<Sphere>(sphere1);
        auto& s2 = registry.get<Sphere>(sphere2);
        auto& s3 = registry.get<Sphere>(sphere3);

        // Euler krok pro každou kouli během simulace
        if (s1.position.y > 0.0f) {
            eulerStep(s1, dt, dragCoefficient, computeDragSquerForce);
            writeData(dataFileV2, s1);
        }
        if (s2.position.y > 0.0f) {
            eulerStep(s2, dt, dragCoefficient, computeDragForce);
            writeData(dataFileV1, s2);
        }
        if (s3.position.y > 0.0f) {
            eulerStep(s3, dt, 0.0f, computeDragForce);
            writeData(dataFileV0, s3);
        }

        // POV-Ray soubor každých 0.05 sekundy
        if (std::fmod(time, 0.05) < dt) {
            std::vector<Sphere> spheres = { s1, s2, s3 };
            writePOVRayFile(frameNumber++, targetSphere, spheres);
        }

        std::cout << "Čas: " << time << std::endl;
        time += dt;
    }

    return 0;
}
