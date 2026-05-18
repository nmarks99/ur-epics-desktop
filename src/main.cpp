#include "raylib.h"
#include "raymath.h"

#include "imgui.h"
#include "rlImGui.h"
#include "ezec.hpp"

// DPI scaling functions
float ScaleToDPIF(float value) {
    return GetWindowScaleDPI().x * value;
}

int ScaleToDPII(int value) {
    return int(GetWindowScaleDPI().x * value);
}

int main(int argc, char* argv[]) {

    ezec::Context ctxt;
    ezec::ChannelGroup pvgroup;
    double rbv;
    pvgroup.add("nick:m1.TWF");
    pvgroup.add("nick:m1.TWR");
    pvgroup.add("nick:m1.RBV");
    pvgroup.bind(rbv, "nick:m1.RBV");

    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 1280;
    int screenHeight = 800;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "raylib-Extras [ImGui] example - simple ImGui Demo");
    SetTargetFPS(144);
    rlImGuiSetup(true);

    Texture image = LoadTexture("/home/nick/devel/epics-imgui/rlImGui/resources/parrots.png");

    // Main loop
    while (!WindowShouldClose()) {

        pvgroup.sync();

        BeginDrawing();
        ClearBackground(DARKGRAY);

        // start ImGui Conent
        rlImGuiBegin();

        // show ImGui Content
        bool open = true;
        // ImGui::ShowDemoWindow(&open);

        open = true;
        if (ImGui::Begin("Test Window", &open)) {
            // rlImGuiImage(&image);
            ImGui::Text("motor rbv = %.4f", rbv);
        }
        ImGui::End();

        // end ImGui Content
        rlImGuiEnd();

        // if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        // DrawText("Prssed", 0, 0, 20, RED);
        // if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        // DrawText("Down", 0, 20, 20, GREEN);
        // if (IsWindowFocused())
        // DrawText("Focused", 100, 20, 20, WHITE);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    rlImGuiShutdown();
    UnloadTexture(image);
    CloseWindow(); // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
