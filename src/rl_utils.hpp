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

struct RLWindow {
    RLWindow(int width, int height, const char *title);

    ~RLWindow();

    // delete copy constructor and copy assignment operator
    RLWindow(const RLWindow &) = delete;
    RLWindow &operator=(const RLWindow &) = delete;
};

struct RLCamera3D {
    RLCamera3D();
    void update();
    Camera camera;
};
