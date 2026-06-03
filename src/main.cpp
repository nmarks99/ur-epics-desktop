#include "imgui.h"
#include "imgui_internal.h"
#include "raylib.h"
#include "rcamera.h"
#include "rlImGui.h"

#include "ezec.hpp"
#include "rl_utils.hpp"
#include "ur.hpp"

constexpr int TARGET_FPS = 60;
constexpr double FRAME_TIME = 1.0 / TARGET_FPS;

enum class ActiveWindow { Robot, Controls, Sidebar };

static ActiveWindow g_active_window = ActiveWindow::Robot;

static void build_default_layout(ImGuiID dockspace_id) {
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

    // ImGuiID left, center_and_bottom;
    // ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.20f, &left, &center_and_bottom);
//
    // ImGuiID center, bottom;
    // ImGui::DockBuilderSplitNode(center_and_bottom, ImGuiDir_Down, 0.30f, &bottom, &center);

    ImGuiID bottom, center;
    ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.30f, &bottom, &center);

    // ImGui::DockBuilderDockWindow("Side Panel", left);
    ImGui::DockBuilderDockWindow("Robot", center);
    ImGui::DockBuilderDockWindow("Controls", bottom);

    ImGui::DockBuilderFinish(dockspace_id);
}

namespace render {

void jog_button(const char* label, ImVec2 size, ImGuiDir dir, int& throttle) {
    ImGui::Button(label, size);
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right) && ImGui::IsItemHovered()) {
        ImGui::SetItemTooltip("%s",label);
    }

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

void controls() {
    ImGui::Begin("Controls");
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        ImGui::SetWindowFocus();
    }
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        g_active_window = ActiveWindow::Controls;
    }

    ImGuiStyle& style = ImGui::GetStyle();
    float border_height = ImGui::GetFontSize() + (style.FramePadding.y * 2.0f);

    ImVec2 button_size = ImVec2(50.0, 50.0);
    ImVec2 spacer = ImVec2(70.0, 50.0);
    auto y_center = ImGui::GetContentRegionAvail().y / 2 + ImGui::GetCursorPosY();
    ImGui::SetCursorPosY(y_center - (button_size.y / 2 + button_size.y + style.ItemSpacing.y));

    static int throttle_up = TARGET_FPS / 10;
    static int throttle_left = TARGET_FPS / 10;
    static int throttle_right = TARGET_FPS / 10;
    static int throttle_down = TARGET_FPS / 10;

    ImGui::Dummy(button_size);
    ImGui::SameLine();
    jog_button("##jog_fwd", button_size, ImGuiDir::ImGuiDir_Up, throttle_up);
    ImGui::SameLine();
    ImGui::Dummy(spacer);
    ImGui::SameLine();
    jog_button("##jog_up", button_size, ImGuiDir::ImGuiDir_Up, throttle_up);

    jog_button("##jog_left", button_size, ImGuiDir::ImGuiDir_Left, throttle_left);
    ImGui::SameLine();
    ImGui::Dummy(button_size);
    ImGui::SameLine();
    jog_button("##jog_right", button_size, ImGuiDir::ImGuiDir_Right, throttle_right);

    ImGui::Dummy(button_size);
    ImGui::SameLine();
    jog_button("##jog_bck", button_size, ImGuiDir::ImGuiDir_Down, throttle_down);
    ImGui::SameLine();
    ImGui::Dummy(spacer);
    ImGui::SameLine();
    jog_button("##jog_down", button_size, ImGuiDir::ImGuiDir_Up, throttle_up);

    ImGui::End();
}

void robot(RenderTexture2D& view_texture, int& view_width, int& view_height) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Robot", nullptr, ImGuiWindowFlags_NoScrollbar);
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        ImGui::SetWindowFocus();
    }
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        g_active_window = ActiveWindow::Robot;
    }
    ImVec2 panelSize = ImGui::GetContentRegionAvail();
    view_width = (int)panelSize.x;
    view_height = (int)panelSize.y;
    rlImGuiImageRenderTextureFit(&view_texture, true);
    ImGui::End();
    ImGui::PopStyleVar();
}

// void sidebar() {
    // ImGui::Begin("Side Panel");
    // if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        // ImGui::SetWindowFocus();
    // }
    // if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        // g_active_window = ActiveWindow::Sidebar;
    // }
    // ImGui::Text("Robot Arm Control");
    // ImGui::Separator();
    // ImGui::TextWrapped("Side panel placeholder for future controls and status displays.");
    // ImGui::End();
// }

} // namespace render


int main(int argc, char* argv[]) {

    // TODO: make configurable
    std::string ioc_prefix = "urExample:";

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

    // Set up EPICS connection with ezec
    ezec::Context ctxt;
    std::vector<double> joints(UR_NUM_AXES);
    ctxt.add(ioc_prefix + "Receive:ActualJointPositions.VAL").bind(joints);

    // Create the UR robot model
    auto robot_model = std::make_unique<UR>(URVersion::UR3e);

    RLCamera3D cam;

    int view_width = GetScreenWidth();
    int view_height = GetScreenHeight();
    RenderTexture2D view_texture = LoadRenderTexture(view_width, view_height);
    SetTextureFilter(view_texture.texture, TEXTURE_FILTER_BILINEAR);

    bool layout_initialized = false;

    while (!WindowShouldClose()) {
        double frame_start = GetTime();

        if (view_width > 0 && view_height > 0) {
            if (view_texture.texture.width != view_width || view_texture.texture.height != view_height) {
                UnloadRenderTexture(view_texture);
                view_texture = LoadRenderTexture(view_width, view_height);
                SetTextureFilter(view_texture.texture, TEXTURE_FILTER_BILINEAR);
            }

            // TODO: only update camera when 3D robot window is focused
            if (g_active_window == ActiveWindow::Robot) {
                cam.update();
            }

            // Update robot joint angles
            if (ctxt.sync()) {
                for (auto& v : joints) {
                    v *= M_PI / 180.0; // convert to rad
                }
                robot_model->update(joints);
            }

            BeginTextureMode(view_texture);
            ClearBackground(RAYWHITE);
            // Draw ------------------------
            BeginMode3D(cam.camera);
            robot_model->draw(0);
            DrawGrid(10, 0.25f);
            EndMode3D();
            EndTextureMode();
            // -----------------------------
        }

        // Draw //////////////////////////////////////////////////////
        BeginDrawing();
        ClearBackground(RAYWHITE);
        rlImGuiBegin();

        // Setup docking
        ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
        ImGui::DockSpaceOverViewport(dockspace_id, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);
        if (!layout_initialized) {
            build_default_layout(dockspace_id);
            layout_initialized = true;
        }

        // render::sidebar();
        render::robot(view_texture, view_width, view_height);
        render::controls();

        rlImGuiEnd();
        EndDrawing();
        //////////////////////////////////////////////////////////////

        double elapsed = GetTime() - frame_start;
        if (elapsed < FRAME_TIME) {
            WaitTime(FRAME_TIME - elapsed);
        }
    }

    robot_model.reset();
    UnloadRenderTexture(view_texture);
    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
