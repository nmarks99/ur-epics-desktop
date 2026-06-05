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

class RobotRenderer {
  public:
    RobotRenderer() :
        robot_model_(UR(URVersion::UR3e)),
        view_width_(GetScreenWidth()),
        view_height_(GetScreenHeight()),
        view_texture_(LoadRenderTexture(view_width_, view_height_))
    {
        SetTextureFilter(view_texture_.texture, TEXTURE_FILTER_BILINEAR);
    }

    ~RobotRenderer() {
        UnloadRenderTexture(view_texture_);
    }

    void update(const std::vector<double>& joints, bool needs_update, bool window_active, bool pv_connected) {
        if (view_width_ > 0 && view_height_ > 0) {
            if (view_texture_.texture.width != view_width_ || view_texture_.texture.height != view_height_) {
                UnloadRenderTexture(view_texture_);
                view_texture_ = LoadRenderTexture(view_width_, view_height_);
                SetTextureFilter(view_texture_.texture, TEXTURE_FILTER_BILINEAR);
            }

            // only update camera when 3D robot window is focused
            if (window_active) {
                cam_.update();
            }

            // Update robot joint angles
            if (needs_update) {
                std::vector<double> joints_rad(joints.size());
                for (size_t i = 0; i < joints.size(); i++) {
                    joints_rad[i] = joints[i] * M_PI / 180.0;
                }
                robot_model_.update(joints_rad);
            }

            BeginTextureMode(view_texture_);
            ClearBackground(RAYWHITE);
            // Draw ------------------------
            BeginMode3D(cam_.camera);
            robot_model_.draw(0, !pv_connected);
            DrawGrid(10, 0.25f);
            EndMode3D();
            EndTextureMode();
            // -----------------------------
        }
    }

    void set_width(int width) {
        view_width_ = width;
    }

    void set_height(int height) {
        view_height_ = height;
    }

    RenderTexture2D& texture() {
        return view_texture_;
    }

  private:
    int view_width_ = 0;
    int view_height_ = 0;
    RenderTexture2D view_texture_;
    UR robot_model_;
    RLCamera3D cam_;
};

class Application {
  public:
    Application(const std::string& ioc_prefix) :
        P_(ioc_prefix),
        rl_window_(1000, 900, "UR EPICS Desktop"),
        joint_angles_(UR_NUM_AXES, 0.0)
    {
        // RLWindow constructor calls InitWindow, rlImGuiSetup, etc.

        // Setup ImGui docking and font
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.FontGlobalScale = 2.0f;
        auto font_ttf_path = get_resource_dir() / "fonts/JetBrainsMonoNerdFont-Regular.ttf";
        auto font = io.Fonts->AddFontFromFileTTF(font_ttf_path.c_str());
        io.FontDefault = font;

        // Set up EPICS connection
        ctxt_.ensure(P_, {
            "Receive:ActualJointPositions.VAL",
            "Control:JogStart.PROC",
            "Control:JogStop.PROC",
            "Control:JogSpeedX.VAL",
            "Control:JogSpeedY.VAL",
            "Control:JogSpeedZ.VAL",
            "Control:JogSpeedRoll.VAL",
            "Control:JogSpeedPitch.VAL",
            "Control:JogSpeedYaw.VAL",
            "RobotiqGripper:Open.PROC",
            "RobotiqGripper:Close.PROC",
        });
        ctxt_.bind(joint_angles_, P_ + "Receive:ActualJointPositions.VAL");

    }

    void run() {
        while(!WindowShouldClose()) {
            double frame_start = GetTime();

            update();
            draw();

            double elapsed = GetTime() - frame_start;
            if (elapsed < FRAME_TIME) {
                WaitTime(FRAME_TIME - elapsed);
            }
        }
    };

  private:
    const std::string P_;
    ezec::Context ctxt_;
    RLWindow rl_window_;
    RobotRenderer robot_renderer_;
    ActiveWindow active_window_ = ActiveWindow::Robot;
    std::vector<double> joint_angles_;
    bool layout_initialized_ = false;
    float jog_speed_ = 50.0;
    static constexpr double JOG_INTERVAL = 0.25;
    std::array<double, 6> jog_last_time_{};

    // sync EPICS, update robot model
    void update() {
        bool new_epics_data = ctxt_.sync();
        auto connected = ctxt_[P_ + "Receive:ActualJointPositions.VAL"].connected();
        robot_renderer_.update(joint_angles_, new_epics_data, active_window_==ActiveWindow::Robot, connected);
    }

    // render 3D robot model to 2D texture, draw ImGui
    void draw() {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        rlImGuiBegin();

        // Setup docking
        ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
        ImGui::DockSpaceOverViewport(dockspace_id, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);
        if (!layout_initialized_) {
            build_default_layout(dockspace_id);
            layout_initialized_ = true;
        }

        draw_robot_window();
        draw_controls_window();

        rlImGuiEnd();
        EndDrawing();
        //////////////////////////////////////////////////////////////
    }

    static void build_default_layout(ImGuiID dockspace_id) {
        ImGui::DockBuilderRemoveNode(dockspace_id);
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

        ImGuiID bottom, center;
        ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.35f, &bottom, &center);

        ImGui::DockBuilderDockWindow("Robot", center);
        ImGui::DockBuilderDockWindow("Controls", bottom);

        ImGui::DockBuilderFinish(dockspace_id);
    }

    void set_jog_speeds(std::vector<double> speeds) {
        static const std::array<std::string, UR_NUM_AXES> axis_names = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
        for (size_t i = 0; i < UR_NUM_AXES; i++) {
            const auto pv_name = P_ + "Control:JogSpeed" + axis_names[i] + ".VAL";
            ctxt_.put(pv_name, speeds[i]);
        }
    }

    void jog_start() { ctxt_.put(P_ + "Control:JogStart.PROC", 1); }

    void jog_stop() { ctxt_.put(P_ + "Control:JogStop.PROC", 1); }

    void jog_button(const char* label, ImVec2 size, JogDir dir) {
        ImGui::Button(label, size);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right) && ImGui::IsItemHovered()) {
            ImGui::SetItemTooltip("%s", label);
        }

        if (ImGui::IsItemActive()) {
            if (!ctxt_[P_ + "Control:JogStart.PROC"].connected() ||
                !ctxt_[P_ + "Control:JogStop.PROC"].connected()) {
                return;
            }
            double now = GetTime();
            double& last = jog_last_time_[static_cast<int>(dir)];
            if (now - last >= JOG_INTERVAL) {
                switch (dir) {
                case JogDir::Up:
                    set_jog_speeds({0.0, 0.0, jog_speed_, 0.0, 0.0, 0.0});
                    break;
                case JogDir::Down:
                    set_jog_speeds({0.0, 0.0, -jog_speed_, 0.0, 0.0, 0.0});
                    break;
                case JogDir::Forward:
                    set_jog_speeds({0.0, jog_speed_, 0.0, 0.0, 0.0, 0.0});
                    break;
                case JogDir::Backward:
                    set_jog_speeds({0.0, -jog_speed_, 0.0, 0.0, 0.0, 0.0});
                    break;
                case JogDir::Left:
                    set_jog_speeds({-jog_speed_, 0.0, 0.0, 0.0, 0.0, 0.0});
                    break;
                case JogDir::Right:
                    set_jog_speeds({jog_speed_, 0.0, 0.0, 0.0, 0.0, 0.0});
                    break;
                }
                jog_start();
                last = now;
            }
        }

        if (ImGui::IsItemDeactivated()) {
            if (!ctxt_[P_ + "Control:JogStart.PROC"].connected() ||
                !ctxt_[P_ + "Control:JogStop.PROC"].connected()) {
                return;
            }
            jog_stop();
        }
    }

    void draw_controls_window() {
        ImGui::Begin("Controls");
        if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
            ImGui::SetWindowFocus();
        }
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
            active_window_ = ActiveWindow::Controls;
        }

        ImGuiStyle& style = ImGui::GetStyle();
        ImVec2 button_size = ImVec2(50.0, 50.0);
        ImVec2 spacer = ImVec2(70.0, 50.0);
        auto y_center = ImGui::GetContentRegionAvail().y / 2 + ImGui::GetCursorPosY();
        ImGui::SetCursorPosY(y_center - (button_size.y + button_size.y/4 + button_size.y + style.ItemSpacing.y + style.ItemSpacing.y/2));

        ImGui::Dummy(button_size);
        ImGui::SameLine();
        jog_button("##jog_fwd", button_size, JogDir::Forward);
        ImGui::SameLine();
        ImGui::Dummy(spacer);
        ImGui::SameLine();
        jog_button("##jog_up", button_size, JogDir::Up);

        jog_button("##jog_left", button_size, JogDir::Left);
        ImGui::SameLine();
        ImGui::Dummy(button_size);
        ImGui::SameLine();
        jog_button("##jog_right", button_size, JogDir::Right);

        ImGui::Dummy(button_size);
        ImGui::SameLine();
        jog_button("##jog_bck", button_size, JogDir::Backward);
        ImGui::SameLine();
        ImGui::Dummy(spacer);
        ImGui::SameLine();
        jog_button("##jog_down", button_size, JogDir::Down);

        ImGui::Dummy({0, button_size.y/2});
        ImGui::Text("Speed:");
        ImGui::SameLine();
        ImGui::PushItemWidth(200);
        ImGui::SliderFloat("%##jog_speed_slider", &jog_speed_, 1.0f, 100.0f, "%.0f");

        ImGui::End();
    }

    void draw_robot_window() {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Robot", nullptr, ImGuiWindowFlags_NoScrollbar);
        if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
            ImGui::SetWindowFocus();
        }
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
            active_window_ = ActiveWindow::Robot;
        }
        ImVec2 panelSize = ImGui::GetContentRegionAvail();
        robot_renderer_.set_width((int)panelSize.x);
        robot_renderer_.set_height((int)panelSize.y);
        rlImGuiImageRenderTextureFit(&robot_renderer_.texture(), true);
        ImGui::End();
        ImGui::PopStyleVar();
    }
};


int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: ./ur-epics-desktop <ioc_prefix>\n");
        return EXIT_FAILURE;
    }

    std::string ioc_prefix = argv[1];
    if (!ioc_prefix.size()) {
        printf("IOC prefix empty\n");
        return EXIT_FAILURE;
    }

    Application app(ioc_prefix);
    app.run();

    return EXIT_SUCCESS;
}
