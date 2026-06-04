#include "imgui.h"
#include "imgui_internal.h"
#include "raylib.h"
#include "rcamera.h"
#include "rlImGui.h"

#include "ezec.hpp"
#include "rl_utils.hpp"
#include "ur.hpp"

enum class ActiveWindow { Robot, Controls, Sidebar };
enum class JogDir { Up, Down, Left, Right, Forward, Backward };

constexpr int TARGET_FPS = 60;
constexpr double FRAME_TIME = 1.0 / TARGET_FPS;

namespace global {
std::string ioc_prefix;
ezec::Context ctxt;
ActiveWindow active_window = ActiveWindow::Robot;
double jog_speed = 30; // mm/s
} // namespace global

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
    ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.35f, &bottom, &center);

    // ImGui::DockBuilderDockWindow("Side Panel", left);
    ImGui::DockBuilderDockWindow("Robot", center);
    ImGui::DockBuilderDockWindow("Controls", bottom);

    ImGui::DockBuilderFinish(dockspace_id);
}

void set_jog_speeds(std::vector<double> speeds) {
    static const std::array<std::string, UR_NUM_AXES> axis_names = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
    for (size_t i = 0; i < UR_NUM_AXES; i++) {
        const auto pv_name = global::ioc_prefix + "Control:JogSpeed" + axis_names[i] + ".VAL";
        global::ctxt.put(pv_name, speeds[i]);
    }
}
void jog_start() { global::ctxt.put(global::ioc_prefix + "Control:JogStart.PROC", 1); }
void jog_stop() { global::ctxt.put(global::ioc_prefix + "Control:JogStop.PROC", 1); }

namespace render {

void jog_button(const char* label, ImVec2 size, JogDir dir, int& throttle) {
    ImGui::Button(label, size);
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right) && ImGui::IsItemHovered()) {
        ImGui::SetItemTooltip("%s", label);
    }

    // Run this repeatedly (throttled) when pressed
    if (ImGui::IsItemActive()) {
        if (!global::ctxt[global::ioc_prefix + "Control:JogStart.PROC"].connected())
            return;
        if (!global::ctxt[global::ioc_prefix + "Control:JogStop.PROC"].connected())
            return;
        if ((throttle % (TARGET_FPS / 4)) == 0) {
            printf("%s pressed!\n", label);
            switch (dir) {
            case JogDir::Up:
                set_jog_speeds({0.0, 0.0, global::jog_speed, 0.0, 0.0, 0.0});
                break;
            case JogDir::Down:
                set_jog_speeds({0.0, 0.0, -global::jog_speed, 0.0, 0.0, 0.0});
                break;
            case JogDir::Forward:
                set_jog_speeds({0.0, global::jog_speed, 0.0, 0.0, 0.0, 0.0});
                break;
            case JogDir::Backward:
                set_jog_speeds({0.0, -global::jog_speed, 0.0, 0.0, 0.0, 0.0});
                break;
            case JogDir::Left:
                set_jog_speeds({-global::jog_speed, 0.0, 0.0, 0.0, 0.0, 0.0});
                break;
            case JogDir::Right:
                set_jog_speeds({global::jog_speed, 0.0, 0.00, 0.0, 0.0, 0.0});
                break;
            }
            jog_start();
        }
        throttle++;
    }

    // Run this on release
    if (ImGui::IsItemDeactivated() && throttle != 0) {
        if (!global::ctxt[global::ioc_prefix + "Control:JogStart.PROC"].connected())
            return;
        if (!global::ctxt[global::ioc_prefix + "Control:JogStop.PROC"].connected())
            return;
        printf("stopping jog\n");
        jog_stop();
        throttle = 0;
    }
}

void controls() {
    ImGui::Begin("Controls");
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        ImGui::SetWindowFocus();
    }
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        global::active_window = ActiveWindow::Controls;
    }

    ImGuiStyle& style = ImGui::GetStyle();
    float border_height = ImGui::GetFontSize() + (style.FramePadding.y * 2.0f);

    ImVec2 button_size = ImVec2(50.0, 50.0);
    ImVec2 spacer = ImVec2(70.0, 50.0);
    auto y_center = ImGui::GetContentRegionAvail().y / 2 + ImGui::GetCursorPosY();
    ImGui::SetCursorPosY(y_center - (button_size.y + button_size.y/4 + button_size.y + style.ItemSpacing.y + style.ItemSpacing.y/2));

    static int throttle_fwd = TARGET_FPS / 4;
    static int throttle_back = TARGET_FPS / 4;
    static int throttle_up = TARGET_FPS / 4;
    static int throttle_left = TARGET_FPS / 4;
    static int throttle_right = TARGET_FPS / 4;
    static int throttle_down = TARGET_FPS / 4;

    ImGui::Dummy(button_size);
    ImGui::SameLine();
    jog_button("##jog_fwd", button_size, JogDir::Forward, throttle_fwd);
    ImGui::SameLine();
    ImGui::Dummy(spacer);
    ImGui::SameLine();
    jog_button("##jog_up", button_size, JogDir::Up, throttle_up);

    jog_button("##jog_left", button_size, JogDir::Left, throttle_left);
    ImGui::SameLine();
    ImGui::Dummy(button_size);
    ImGui::SameLine();
    jog_button("##jog_right", button_size, JogDir::Right, throttle_right);

    ImGui::Dummy(button_size);
    ImGui::SameLine();
    jog_button("##jog_bck", button_size, JogDir::Backward, throttle_back);
    ImGui::SameLine();
    ImGui::Dummy(spacer);
    ImGui::SameLine();
    jog_button("##jog_down", button_size, JogDir::Down, throttle_down);

    ImGui::Dummy({0, button_size.y/2});
    ImGui::Text("Speed:");
    ImGui::SameLine();
    ImGui::PushItemWidth(150);
    ImGui::InputDouble("##jog_speed", &global::jog_speed, 0, 0, "%.2f");
    ImGui::SameLine();
    ImGui::Text("mm/s");

    ImGui::End();
}

void robot(RenderTexture2D& view_texture, int& view_width, int& view_height) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Robot", nullptr, ImGuiWindowFlags_NoScrollbar);
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        ImGui::SetWindowFocus();
    }
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        global::active_window = ActiveWindow::Robot;
    }
    ImVec2 panelSize = ImGui::GetContentRegionAvail();
    view_width = (int)panelSize.x;
    view_height = (int)panelSize.y;
    rlImGuiImageRenderTextureFit(&view_texture, true);
    ImGui::End();
    ImGui::PopStyleVar();
}

} // namespace render

int main(int argc, char* argv[]) {

    // TODO: make configurable
    global::ioc_prefix = "urExample:";
    if (!global::ioc_prefix.size()) {
        printf("IOC prefix empty\n");
        return EXIT_FAILURE;
    }

    // SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1000, 900, "UR EPICS Desktop");
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
    std::vector<double> joints(UR_NUM_AXES);
    global::ctxt.bind(joints, global::ioc_prefix + "Receive:ActualJointPositions.VAL");

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

        bool new_epics_data = global::ctxt.sync();

        if (view_width > 0 && view_height > 0) {
            if (view_texture.texture.width != view_width || view_texture.texture.height != view_height) {
                UnloadRenderTexture(view_texture);
                view_texture = LoadRenderTexture(view_width, view_height);
                SetTextureFilter(view_texture.texture, TEXTURE_FILTER_BILINEAR);
            }

            // TODO: only update camera when 3D robot window is focused
            if (global::active_window == ActiveWindow::Robot) {
                cam.update();
            }

            // Update robot joint angles
            if (new_epics_data) {
                std::vector<double> joints_rad(joints.size());
                for (size_t i = 0; i < joints.size(); i++) {
                    joints_rad[i] = joints[i] * M_PI / 180.0;
                }
                robot_model->update(joints_rad);
            }

            BeginTextureMode(view_texture);
            ClearBackground(RAYWHITE);
            // Draw ------------------------
            BeginMode3D(cam.camera);
            robot_model->draw(0, !global::ctxt[global::ioc_prefix + "Receive:ActualJointPositions.VAL"].connected());
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
