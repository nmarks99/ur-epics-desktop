#include "raylib.h"
#include "rcamera.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "rlImGui.h"

#include "rl_utils.hpp"
#include "ur.hpp"
#include "ezec.hpp"

static void BuildDefaultLayout(ImGuiID dockspace_id) {
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

    ImGuiID left, center_and_bottom;
    ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.20f, &left, &center_and_bottom);

    ImGuiID center, bottom;
    ImGui::DockBuilderSplitNode(center_and_bottom, ImGuiDir_Down, 0.30f, &bottom, &center);

    ImGui::DockBuilderDockWindow("Side Panel", left);
    ImGui::DockBuilderDockWindow("3D Viewport", center);
    ImGui::DockBuilderDockWindow("Controls", bottom);

    ImGui::DockBuilderFinish(dockspace_id);
}

int main(int argc, char* argv[]) {

    ezec::Context ctxt;
    ezec::ChannelGroup pvgroup;
    double rbv = 0.0;
    double val = 0.0;
    pvgroup.add("namSoft:m1.TWF");
    pvgroup.add("namSoft:m1.TWR");
    pvgroup.add("namSoft:m1.RBV").bind(rbv);
    pvgroup.add("namSoft:m1.VAL").bind(val);

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "UR EPICS Desktop");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.FontGlobalScale = 2.0f;

    UR robot_model(URVersion::UR3e);

    RLCamera3D cam;

    RenderTexture2D viewTexture = LoadRenderTexture(GetScreenWidth(), GetScreenHeight());

    bool layout_initialized = false;

    while (!WindowShouldClose()) {
        pvgroup.sync();

        if (IsWindowResized()) {
            UnloadRenderTexture(viewTexture);
            viewTexture = LoadRenderTexture(GetScreenWidth(), GetScreenHeight());
        }

        cam.update();

        BeginTextureMode(viewTexture);
        ClearBackground(RAYWHITE);
        BeginMode3D(cam.camera);
        robot_model.draw(0);
        DrawGrid(10, 1.0f);
        EndMode3D();
        EndTextureMode();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        rlImGuiBegin();

        ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
        ImGui::DockSpaceOverViewport(dockspace_id, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        if (!layout_initialized) {
            BuildDefaultLayout(dockspace_id);
            layout_initialized = true;
        }

        ImGui::Begin("Side Panel");
        ImGui::Text("Robot Arm Control");
        ImGui::Separator();
        ImGui::TextWrapped("Side panel placeholder for future controls and status displays.");
        ImGui::End();

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("3D Viewport", nullptr, ImGuiWindowFlags_NoScrollbar);
        rlImGuiImageRenderTextureFit(&viewTexture, true);
        ImGui::End();
        ImGui::PopStyleVar();

        {
            ImGui::Begin("Controls");
            ImGui::Text("RBV: %.4f", rbv);
            ImGui::Text("VAL: %.4f", val);
            float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {
                pvgroup["namSoft:m1.TWR"].put(1);
            }
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::ArrowButton("##right", ImGuiDir_Right)) {
                pvgroup["namSoft:m1.TWF"].put(1);
            }
            ImGui::End();
        }

        rlImGuiEnd();
        EndDrawing();
    }

    UnloadRenderTexture(viewTexture);
    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
