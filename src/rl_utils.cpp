#include "rl_utils.hpp"
#include "rlImGui.h"
#include "raymath.h"
#include "rcamera.h"

// constexpr double AXES_THICKNESS = 0.0075;
constexpr double AXES_THICKNESS = 0.0030;

void draw_axes_3d(float thickness, Matrix transform);

RLModel::RLModel(std::filesystem::path model_path, const std::string &name)
    : name(name) {
    model = LoadModel(model_path.c_str());
}

RLModel::~RLModel() {
    if (IsModelValid(model)) {
        UnloadModel(model);
        model = {0};
    }
}

void RLModel::draw(Color color) {
    if (IsModelValid(model)) {
        DrawModel(model, Vector3Zeros, 1.0, color);
    }
}

void RLModel::draw_wires(Color color) {
    if (IsModelValid(model)) {
        DrawModelWires(model, Vector3Zeros, 1.0, color);
    }
}

void RLModel::draw_axes() {
    if (IsModelValid(model)) {
        draw_axes_3d(AXES_THICKNESS, model.transform);
    }
}


RLWindow::RLWindow(int width, int height, const char *title) {
    SetTraceLogLevel(LOG_FATAL);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    SetTargetFPS(60);
    InitWindow(width, height, title);
    SetExitKey(KEY_NULL);
    rlImGuiSetup(true);
};

RLWindow::~RLWindow() {
    rlImGuiShutdown();
    CloseWindow();
}


RLCamera3D::RLCamera3D() {
    camera = {0};
    camera.position = Vector3{-0.5f, 0.5f, -0.625f}; // Camera position
    camera.target = Vector3{0.0f, 0.25f, 0.0f};     // Camera looking at point
    camera.up = Vector3{0.0f, 1.0f, 0.0f};          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                            // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;         // Camera mode type
}

void RLCamera3D::update() {
    // adpated from raylib UpdateCamera function

    constexpr float CAMERA_MOVE_SPEED = 1.0;
    constexpr float CAMERA_ROTATION_SPEED = 0.03;
    constexpr float CAMERA_PAN_SPEED = 0.2;
    constexpr float CAMERA_MOUSE_MOVE_SENSITIVITY = 0.003;
    constexpr float CAMERA_ORBITAL_SPEED = 0.5; // rad/sec
    constexpr float CAMERA_ZOOM_FACTOR = 0.1;

    constexpr bool moveInWorldPlane = true;
    constexpr bool rotateAroundTarget = true;
    constexpr bool lockView = true;
    constexpr bool rotateUp = false;

    // Camera speeds based on frame time
    float cameraMoveSpeed = CAMERA_MOVE_SPEED * GetFrameTime();
    float cameraRotationSpeed = CAMERA_ROTATION_SPEED * GetFrameTime();
    float cameraPanSpeed = CAMERA_PAN_SPEED * GetFrameTime();
    float cameraOrbitalSpeed = CAMERA_ORBITAL_SPEED * GetFrameTime();

    // Zoom in on the target with the scroll wheel
    CameraMoveToTarget(&camera, -GetMouseWheelMove() * CAMERA_ZOOM_FACTOR);
    if (IsKeyPressed(KEY_KP_SUBTRACT))
        CameraMoveToTarget(&camera, 2.0f);
    if (IsKeyPressed(KEY_KP_ADD))
        CameraMoveToTarget(&camera, -2.0f);

    // rotate around the target when clicking the scroll wheel
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        Vector2 mousePositionDelta = GetMouseDelta();
        CameraYaw(&camera, -mousePositionDelta.x * CAMERA_MOUSE_MOVE_SENSITIVITY, rotateAroundTarget);
        CameraPitch(&camera, -mousePositionDelta.y * CAMERA_MOUSE_MOVE_SENSITIVITY, lockView,
                    rotateAroundTarget, rotateUp);
    }

    // move the camera with the W A S D keys
    if (IsKeyDown(KEY_W))
        CameraMoveForward(&camera, cameraMoveSpeed, moveInWorldPlane);
    if (IsKeyDown(KEY_A))
        CameraMoveRight(&camera, -cameraMoveSpeed, moveInWorldPlane);
    if (IsKeyDown(KEY_S))
        CameraMoveForward(&camera, -cameraMoveSpeed, moveInWorldPlane);
    if (IsKeyDown(KEY_D))
        CameraMoveRight(&camera, cameraMoveSpeed, moveInWorldPlane);
}

void draw_axes_3d(float thickness, Matrix transform) {
    constexpr int sides = 20;
    const Vector3 origin = Vector3Transform(Vector3Zeros, transform);
    const Vector3 x_end = Vector3Transform({0.1, 0.0, 0.0}, transform);
    const Vector3 y_end = Vector3Transform({0.0, 0.1, 0.0}, transform);
    const Vector3 z_end = Vector3Transform({0.0, 0.0, 0.1}, transform);
    DrawCylinderEx(origin, x_end, thickness, thickness, sides, RED);
    DrawCylinderEx(origin, y_end, thickness, thickness, sides, GREEN);
    DrawCylinderEx(origin, z_end, thickness, thickness, sides, BLUE);
}
