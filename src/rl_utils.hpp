#pragma once
#include "raylib.h"
#include <filesystem>

struct RLModel {
    Model model;

    RLModel(const char *model_path);
    RLModel(std::filesystem::path model_path);
    RLModel(const char *model_path, const std::string &name);
    RLModel(std::filesystem::path model_path, const std::string &name);
    ~RLModel();

    void load();
    void unload();

    // delete copy constructor and copy assignment operator
    RLModel(const RLModel &) = delete;
    RLModel &operator=(const RLModel &) = delete;

    void draw(Color color = WHITE);
    void draw_wires(Color color = WHITE);
    void draw_axes();

    std::string name = "";
    std::string path = "";
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

void draw_axes_3d(float thickness, Matrix transform);
