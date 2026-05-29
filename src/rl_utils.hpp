#pragma once
#include <filesystem>
#include "raylib.h"

struct RLModel {
    RLModel(std::filesystem::path model_path, const std::string &name);
    ~RLModel();

    // delete copy constructor and copy assignment operator
    RLModel(const RLModel &) = delete;
    RLModel &operator=(const RLModel &) = delete;

    void draw(Color color = WHITE);
    void draw_wires(Color color = WHITE);
    void draw_axes();

    std::string name = "";

    Model model = {0};
};

struct RLCamera3D {
    RLCamera3D();
    void update();
    Camera camera;
};
