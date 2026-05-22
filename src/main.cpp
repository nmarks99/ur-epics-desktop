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
    double val;
    pvgroup.add("nick:m1.TWF");
    pvgroup.add("nick:m1.TWR");
    pvgroup.add("nick:m1.RBV").bind(rbv);
    pvgroup.add("nick:m1.VAL").bind(val);

    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 1280;
    int screenHeight = 800;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "raylib-Extras [ImGui] example - simple ImGui Demo");
    SetTargetFPS(144);
    rlImGuiSetup(true);
    ImGui::GetIO().FontGlobalScale = 2.0;

    // Main loop
    while (!WindowShouldClose()) {

        pvgroup.sync();

        BeginDrawing();
        ClearBackground(DARKGRAY);

        // start ImGui Conent
        rlImGuiBegin();

        // show ImGui Content
        // bool open = true;
        // ImGui::ShowDemoWindow(&open);

        {
            ImGui::Begin("Test Window");
            ImGui::Text("motor rbv = %.4f", rbv);
            ImGui::End();
        }

        {
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            ImGui::Begin("another window");
            static int counter = 0;
            float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            // ImGui::PushItemFlag(ImGuiItemFlags_ButtonRepeat, true);
            ImGui::TextColored(ImVec4(0.0f, 0.0f, 1.0f, 1.0f), "%.4f", rbv);
            ImGui::TextColored(ImVec4(0.0f, 0.0f, 0.0f, 1.0f), "%.4f", val);
            if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {
                pvgroup["nick:m1.TWR"].put(1);
            }
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::ArrowButton("##right", ImGuiDir_Right)) {
                pvgroup["nick:m1.TWF"].put(1);
            }
            ImGui::End();
            ImGui::PopStyleColor();
        }

        // end ImGui Content
        rlImGuiEnd();
        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    rlImGuiShutdown();
    CloseWindow();
    //--------------------------------------------------------------------------------------

    return 0;
}
