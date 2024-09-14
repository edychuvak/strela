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

void eulerStep(Sphere& sphere, double dt, double dragCoefficient, Vec2 (*computeDrag)(const Vec2&, double)) {
    Vec2 dragForce = computeDrag(sphere.velocity, dragCoefficient);
    sphere.velocity = sphere.velocity + (dragForce - Vec2{0, g}) * dt;
    sphere.position = sphere.position + sphere.velocity * dt;
}

void writePOVRayFile(int frameNumber, const Sphere& targetSphere, const Sphere& projectile) {
    std::ofstream file("frame_" + std::to_string(frameNumber) + ".pov");
    file << "#include \"colors.inc\"\n";
    file << "camera { location <5, 4, -11> look_at <5, 4, 0> }\n";
    file << "light_source { <0, 10, -20> color rgb <1, 1, 1> }\n";
    file << "background { color White }\n";
    file << "union {\n";  // Start of the container object

    // Target Sphere
    file << "  sphere { <" << targetSphere.position.x << ", " << targetSphere.position.y << ", 0>, "
         << targetSphere.radius << " texture { pigment { color " << targetSphere.color << " } } }\n";

    // Projectile Sphere
    file << "  sphere { <" << projectile.position.x << ", " << projectile.position.y << ", 0>, "
         << projectile.radius << " texture { pigment { color " << projectile.color << " } } }\n";

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
    double speed = 15.0;   // Počáteční rychlost 15 m/s

    // Otevření souboru pro ukládání trajektorie
    std::ofstream dataFile("sphere_trajectory.txt");

    if (!dataFile.is_open()) {
        std::cerr << "Nepodařilo se otevřít soubor pro zápis!" << std::endl;
        return 1;
    }

    // EnTT registry
    entt::registry registry;

    // Vytvoření cílové koule (target)
    auto target = registry.create();
    registry.emplace<Sphere>(target, Vec2{10, 0}, Vec2{0, 0}, 0.5, "Red");

    // Vytvoření jedné střelící koule (projectile)
    auto projectile = registry.create();
    registry.emplace<Sphere>(projectile, Vec2{0, 0}, Vec2{5, 10}, 0.5, "Blue");

    double maxDistance = 0.0;
    double bestAngle = 0.0;

    // Simulace pro různé úhly (od 30 do 60 stupňů)
    for (double angle = 30.0; angle <= 60.0; angle += 1.0) {
        // Převod úhlu na radiány
        double radians = angle * M_PI / 180.0;

        // Vypočítání složek rychlosti
        double vx = speed * std::cos(radians);
        double vy = speed * std::sin(radians);

        // Nastavení počátečních podmínek pro střelící kouli
        auto& projectileSphere = registry.get<Sphere>(projectile);
        projectileSphere.position = {0, 0};  // Výchozí pozice
        projectileSphere.velocity = {vx, vy};  // Počáteční rychlost z daného úhlu

        // Provádíme Euler krok před smyčkou while
        eulerStep(projectileSphere, dt, dragCoefficient, computeDragSquerForce);

        double currentDistance = 0.0;

        // Reset time pro každou simulaci
        time = 0.0;

        // Simulace letu
        while (projectileSphere.position.y > 0.0f && time < 10.0) {
            eulerStep(projectileSphere, dt, dragCoefficient, computeDragSquerForce);
            writeData(dataFile,projectileSphere);
            time += dt;
        }

        // Vypočítání vzdálenosti doletu
        currentDistance = projectileSphere.position.x;

        // Pokud je dolet pro tento úhel větší než dosud maximální, ulož ho
        if (currentDistance > maxDistance) {
            maxDistance = currentDistance;
            bestAngle = angle;
        }

        std::cout << "Úhel: " << angle << " stupňů, Dolet: " << currentDistance << " metrů" << std::endl;
    }

    std::cout << "Největší dolet byl dosažen při úhlu: " << bestAngle << " stupňů a dolet byl: " << maxDistance << " metrů." << std::endl;

    return 0;
}
