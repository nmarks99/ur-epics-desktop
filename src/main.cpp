#include "raylib.h"
#include "rcamera.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "rlImGui.h"

#include "rl_utils.hpp"
#include "ur.hpp"
#include "ezec.hpp"

constexpr int TARGET_FPS = 60;
constexpr double FRAME_TIME = 1.0 / TARGET_FPS;

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

void jog_button(const char* label, ImGuiDir dir, int& throttle) {
    ImGui::ArrowButton(label, dir);

    // Run this repeatedly (throttled) when pressed
    if (ImGui::IsItemActive()) {
        if ((throttle % (TARGET_FPS / 10)) == 0) {
            printf("%s pressed!\n", label);
        }
        throttle++;
    }

    // Run this on release
    if (ImGui::IsItemDeactivated() && throttle != 0) {
        printf("%s released\n", label);
        throttle = 0;
    }
}

void jog_buttons() {
    static bool pressed = false;
    static int pcount = 0;

    ImGuiStyle& style = ImGui::GetStyle();
    float btn_side = ImGui::GetFontSize() + (style.FramePadding.y * 2.0f);
    ImVec2 btn_size = ImVec2(btn_side, btn_side);

    static int throttle_up = TARGET_FPS / 10;
    static int throttle_left = TARGET_FPS / 10;
    static int throttle_right = TARGET_FPS / 10;
    static int throttle_down = TARGET_FPS / 10;

    ImGui::Dummy(btn_size);
    ImGui::SameLine();
    jog_button("##jog_up", ImGuiDir::ImGuiDir_Up, throttle_up);
    ImGui::SameLine();
    ImGui::Dummy(btn_size);

    jog_button("##jog_left", ImGuiDir::ImGuiDir_Left, throttle_left);
    ImGui::SameLine();
    ImGui::Dummy(btn_size);
    ImGui::SameLine();
    jog_button("##jog_right", ImGuiDir::ImGuiDir_Right, throttle_right);

    ImGui::Dummy(btn_size);
    ImGui::SameLine();
    jog_button("##jog_down", ImGuiDir::ImGuiDir_Down, throttle_down);
    ImGui::SameLine();
    ImGui::Dummy(btn_size);

}

int main(int argc, char* argv[]) {

    // SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "UR EPICS Desktop");
    SetTargetFPS(TARGET_FPS);
    rlImGuiSetup(true);

    // Setup docking and font
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.FontGlobalScale = 2.0f;
    auto font_ttf_path = get_resource_dir() / "fonts/JetBrainsMonoNerdFont-Regular.ttf";
    auto font = io.Fonts->AddFontFromFileTTF(font_ttf_path.c_str());
    io.FontDefault = font;

    ezec::Context ctxt;
    std::vector<double> joints(UR_NUM_AXES);
    ctxt.add("urExample:Receive:ActualJointPositions.VAL").bind(joints);

    auto robot_model = std::make_unique<UR>(URVersion::UR3e);

    RLCamera3D cam;

    int viewW = GetScreenWidth();
    int viewH = GetScreenHeight();
    RenderTexture2D viewTexture = LoadRenderTexture(viewW, viewH);
    SetTextureFilter(viewTexture.texture, TEXTURE_FILTER_BILINEAR);

    bool layout_initialized = false;

    while (!WindowShouldClose()) {
        double frame_start = GetTime();

        if (viewW > 0 && viewH > 0) {
            if (viewTexture.texture.width != viewW || viewTexture.texture.height != viewH) {
                UnloadRenderTexture(viewTexture);
                viewTexture = LoadRenderTexture(viewW, viewH);
                SetTextureFilter(viewTexture.texture, TEXTURE_FILTER_BILINEAR);
            }

            cam.update();
            if (ctxt.sync()) {
                for (auto& v : joints) {
                    v *= M_PI/180.0; // convert to rad
                }
                robot_model->update(joints);
            }

            BeginTextureMode(viewTexture);
            ClearBackground(RAYWHITE);
            // Draw ////////////////////////////
            BeginMode3D(cam.camera);
                robot_model->draw(0);
                DrawGrid(10, 0.25f);
            EndMode3D();
            ///////////////////////////////////
            EndTextureMode();
        }

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
        ImVec2 panelSize = ImGui::GetContentRegionAvail();
        viewW = (int)panelSize.x;
        viewH = (int)panelSize.y;
        rlImGuiImageRenderTextureFit(&viewTexture, true);
        ImGui::End();
        ImGui::PopStyleVar();

        {
            ImGui::Begin("Controls");
            jog_buttons();
            ImGui::End();
        }

        rlImGuiEnd();
        EndDrawing();

        double elapsed = GetTime() - frame_start;
        if (elapsed < FRAME_TIME) {
            WaitTime(FRAME_TIME - elapsed);
        }
    }

    robot_model.reset();
    UnloadRenderTexture(viewTexture);
    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
