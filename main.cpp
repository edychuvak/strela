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

    std::ofstream dataFileV2("sphere_trajectory_V2.txt");
    if (!dataFileV2.is_open()) {
        std::cerr << "Nepodařilo se otevřít soubor pro zápis!" << std::endl;
        return 1;
    }

    std::ofstream dataFileV1("sphere_trajectory_V1.txt");
    if (!dataFileV2.is_open()) {
        std::cerr << "Nepodařilo se otevřít soubor p"
                     "ro zápis!" << std::endl;
        return 1;
    }

    std::ofstream dataFileV0("sphere_trajectory_V0.txt");
    if (!dataFileV2.is_open()) {
        std::cerr << "Nepodařilo se otevřít soubor pro zápis!" << std::endl;
        return 1;
    }

    Sphere targetSphere = {{10, 0}, {0, 0}, 0.5, "Red"};  // Target sphere
    std::vector<Sphere> spheres = {
        {{0, 0}, {5, 10}, 0.5, "Blue"},   // First moving sphere
        {{0, 0}, {5, 10}, 0.5, "Green"},  // Second moving sphere
        {{0, 0}, {5, 10}, 0.5, "Yellow"}    // Third moving sphere
    };
    eulerStep(spheres[0], dt, dragCoefficient, computeDragSquerForce);
    eulerStep(spheres[1], dt, dragCoefficient, computeDragForce);
    eulerStep(spheres[2], dt, 0.0f, computeDragForce);

    while (time < 3.0) {  // Simuluj 10 sekund
        //rungeKuttaStep(sphere1, dt, dragCoefficient, computeDragForce);
        //eulerStep(spheres[0], dt, dragCoefficient, computeDragSquerForce);
        //eulerStep(spheres[1], dt, dragCoefficient, computeDragForce);
        //eulerStep(spheres[2], dt, 0.0f, computeDragForce);
        // Každou sekundu vygeneruj POV-Ray soubor
        if (spheres[0].position.y > 0.0f)
        {
            eulerStep(spheres[0], dt, dragCoefficient, computeDragSquerForce);
            writeData(dataFileV2, spheres[0]);
        }
        if (spheres[1].position.y > 0.0f)
        {
            eulerStep(spheres[1], dt, dragCoefficient, computeDragForce);
            writeData(dataFileV1, spheres[1]);
        }
        if (spheres[2].position.y > 0.0f)
        {
            eulerStep(spheres[2], dt, 0.0f, computeDragForce);
            writeData(dataFileV0, spheres[2]);
        }

        if (std::fmod(time, 0.05) < dt) {
            writePOVRayFile(frameNumber++, targetSphere, spheres);
        }

        /*if (checkCollision(sphere1, sphere0)) {
            std::cout << "Koule se srazily!" << std::endl;
            break;
        }*/
        std::cout << time << std::endl;
        time += dt;
    }

    return 0;
}
