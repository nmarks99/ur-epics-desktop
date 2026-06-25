#include "imgui.h"
#include "imgui_internal.h"
#include "raylib.h"
#include "rcamera.h"
#include "rlImGui.h"

#include "pace.hpp"
#include "rl_utils.hpp"
#include "ur.hpp"

const std::string IMAGE_DATA_DIR="/home/nmarks/open-house/oh26/iocBoot/iocoh26/data";

enum class ActiveWindow { Robot, Controls, Sidebar };
enum class JogDir { Up, Down, Left, Right, Forward, Backward };

constexpr int TARGET_FPS = 60;
constexpr double FRAME_TIME = 1.0 / TARGET_FPS;

constexpr std::array<const char*, 11> safety_mode_labels = {
    "Normal            ",
    "Reduced           ",
    "Protective Stopped",
    "Recovery Mode     ",
    "Safeguard Stopped ",
    "System E-Stopped  ",
    "Robot E-Stopped   ",
    "Emergency Stopped ",
    "Violation         ",
    "Fault             ",
    "Stopped for Safety",
};

class RobotRenderer {
  public:
    RobotRenderer() :
        robot_model_(UR(URVersion::UR3e)),
        view_width_(GetScreenWidth()),
        view_height_(GetScreenHeight()),
        view_texture_(LoadRenderTexture(view_width_, view_height_))
    {
        SetTextureFilter(view_texture_.texture, TEXTURE_FILTER_BILINEAR);


        // // 1. Load image into CPU RAM
        // Image img = LoadImage("robotiq_handE_screenshot-crop.png");
        // gripper_img_.width = img.width;
        // gripper_img_.height = img.height;
        // gripper_img_.texture = LoadTextureFromImage(img);
        // UnloadImage(img);
    }

    ~RobotRenderer() {
        UnloadRenderTexture(view_texture_);
        UnloadTexture(gripper_img_.texture);
    }

    void update(const std::vector<double>& joints, double pose_z, double gripper_pos, bool needs_update, bool window_active, bool pv_connected) {
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
                robot_model_.update(joints_rad, gripper_pos);
            }


            BeginTextureMode(view_texture_);
            ClearBackground(RAYWHITE);
            // Draw ------------------------
            BeginMode3D(cam_.camera);
            robot_model_.draw(0, !pv_connected);
            // robot_model_.draw(0, false);
            // robot_model_.draw_axes(0b1100000000);
            DrawGrid(10, 0.25f);
            EndMode3D();

            // // close-up pick view
            // DrawRectangle(0, pose_z-105+0, 210, 70, ColorAlpha(RED, 0.9));
            // DrawRectangle(0, pose_z-105+70, 210, 70, ColorAlpha(GREEN, 0.9));
            // DrawRectangle(0, pose_z-105+140, 210, 70, ColorAlpha(RED, 0.9));
            // // DrawRectangle(0, 0, 210, 210, LIGHTGRAY);
            // DrawTextureEx(gripper_img_.texture, {105 - float(0.30*(gripper_img_.width)/2.0), 0}, 0.0, 0.30, WHITE);
            // DrawCircle(105, 92, 5, BLACK);
            // // DrawText("<-", 105, pose_z, 24, BLACK);

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
    struct GripperImageTexture {
        Texture2D texture;
        int width;
        int height;
    } gripper_img_;
    UR robot_model_;
    RLCamera3D cam_;
};

class Application {
  public:
    Application(const std::string& ioc_prefix) :
        P_(ioc_prefix),
        rl_window_(1200, 900, "UR EPICS Desktop")
    {
        // RLWindow constructor calls InitWindow, rlImGuiSetup, etc.

        // Setup ImGui docking and font
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.FontGlobalScale = 1.5f;
        auto font_ttf_path = get_resource_dir() / "fonts/JetBrainsMonoNerdFont-Regular.ttf";
        auto font = io.Fonts->AddFontFromFileTTF(font_ttf_path.c_str());
        io.FontDefault = font;

        // Connect to all the EPICS PVs we will need
        ctxt_.connect(P_ + "UR:Control:JogStart.PROC");
        ctxt_.connect(P_ + "UR:Control:JogStop.PROC");
        ctxt_.connect(P_ + "UR:Control:JogSpeedX.VAL");
        ctxt_.connect(P_ + "UR:Control:JogSpeedY.VAL");
        ctxt_.connect(P_ + "UR:Control:JogSpeedZ.VAL");
        ctxt_.connect(P_ + "UR:Control:JogSpeedRoll.VAL");
        ctxt_.connect(P_ + "UR:Control:JogSpeedPitch.VAL");
        ctxt_.connect(P_ + "UR:Control:JogSpeedYaw.VAL");
        ctxt_.connect(P_ + "UR:Control:Connected").bind(epics_.control_connected);
        ctxt_.connect(P_ + "UR:RobotiqGripper:Open.PROC");
        ctxt_.connect(P_ + "UR:RobotiqGripper:Close.PROC");
        ctxt_.connect(P_ + "UR:ClearFault.PROC");
        ctxt_.connect(P_ + "UR:Receive:PoseZ.VAL");
        ctxt_.connect(P_ + "m1.TWR");
        ctxt_.connect(P_ + "m1.TWF");
        ctxt_.connect(P_ + "m1.STOP");
        ctxt_.connect(P_ + "m1.VAL").bind(epics_.m1_val);
        ctxt_.connect(P_ + "m1.TWV").bind(epics_.m1_twv);
        ctxt_.connect(P_ + "m1.RBV").bind(epics_.m1_rbv);
        ctxt_.connect(P_ + "m1.DESC").bind(epics_.m1_desc);
        ctxt_.connect(P_ + "scan2.EXSC");
        ctxt_.connect(P_ + "AbortScans.PROC");
        ctxt_.connect(P_ + "scan2.BUSY").bind(epics_.scan2_busy);
        ctxt_.connect(P_ + "UR:Receive:ActualJointPositions.VAL").bind(epics_.joint_angles);
        ctxt_.connect(P_ + "UR:Receive:PoseZ.VAL").bind(epics_.pose_z);
        ctxt_.connect(P_ + "UR:Receive:SafetyStatusBits.VAL").bind(epics_.safety_status_bits);
        ctxt_.connect(P_ + "UR:RobotiqGripper:CurrentPosition.VAL").bind(epics_.gripper_pos);
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
    RLWindow rl_window_;
    RobotRenderer robot_renderer_;
    ActiveWindow active_window_ = ActiveWindow::Robot;

    bool layout_initialized_ = false;
    float jog_speed_ = 50.0;
    static constexpr double JOG_INTERVAL = 0.25;
    std::array<double, 6> jog_last_time_{};

    const std::string P_;
    pace::Context ctxt_;

    // Variables updated by EPICS monitor via pace::Context::sync()
    struct EPICSMonitorValues {
        std::vector<double> joint_angles = std::vector<double>(UR_NUM_AXES, 0.0);
        double pose_z = 0.0;
        // int gripper_open = 1;
        // int gripper_closed = 0;
        int safety_status_bits = 1;
        double m1_val = 0.0;
        double m1_twv = 0.0;
        double m1_rbv = 0.0;
        double gripper_pos = 0.0;
        int scan2_busy = 0;
        int control_connected = 0;
        std::string m1_desc;
    } epics_;

    // sync EPICS, update robot model
    void update() {
        bool new_epics_data = ctxt_.sync();
        auto connected = ctxt_[P_ + "UR:Receive:ActualJointPositions.VAL"].connected();
        robot_renderer_.update(epics_.joint_angles, epics_.pose_z, epics_.gripper_pos, new_epics_data, active_window_==ActiveWindow::Robot, connected);
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

        // // Popup if any PVs are disconnected
        // if (!ctxt_.all_connected()) {
            // ImGui::OpenPopup("PV(s) Disconnected");
        // }
        // ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        // ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        // if (ImGui::BeginPopupModal("PV(s) Disconnected", nullptr,
                // ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove))
        // {
            // ImGui::Text("One or more PVs are not connected.");
            // ImGui::Separator();
//
            // bool connected = ctxt_.all_connected();
            // if (connected) {
                // ImGui::CloseCurrentPopup();
            // }
            // ImGui::EndPopup();
        // }

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
        ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.40f, &bottom, &center);

        ImGui::DockBuilderDockWindow("Robot", center);
        ImGui::DockBuilderDockWindow("Controls", bottom);

        ImGui::DockBuilderFinish(dockspace_id);
    }

    void set_jog_speeds(std::vector<double> speeds) {
        static const std::array<std::string, UR_NUM_AXES> axis_names = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
        for (size_t i = 0; i < UR_NUM_AXES; i++) {
            const auto pv_name = P_ + "UR:Control:JogSpeed" + axis_names[i] + ".VAL";
            ctxt_.put(pv_name, speeds[i]);
        }
    }

    void jog_start() { ctxt_.put(P_ + "UR:Control:JogStart.PROC", 1); }

    void jog_stop() { ctxt_.put(P_ + "UR:Control:JogStop.PROC", 1); }

    void jog_button(const char* label, ImVec2 size, JogDir dir) {
        ImGui::Button(label, size);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right) && ImGui::IsItemHovered()) {
            ImGui::SetItemTooltip("%s", label);
        }

        if (ImGui::IsItemActive()) {
            if (!ctxt_[P_ + "UR:Control:JogStart.PROC"].connected() ||
                !ctxt_[P_ + "UR:Control:JogStop.PROC"].connected()) {
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
            if (!ctxt_[P_ + "UR:Control:JogStart.PROC"].connected() ||
                !ctxt_[P_ + "UR:Control:JogStop.PROC"].connected()) {
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

        ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX;

        if (ImGui::BeginTable("controls_layout", 5)) {
            ImGui::TableSetupColumn("left_col", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("spacer1", ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableSetupColumn("mid_col", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("spacer2", ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableSetupColumn("right_col", ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableNextRow();

            // Column 1: jog pad + gripper /////////////////////////
            static bool need_to_clear_disabled = false;
            if (!epics_.control_connected) {
                ImGui::BeginDisabled();
                need_to_clear_disabled = true;
            }
            ImGui::TableNextColumn();

            if (ImGui::BeginTable("jog_table", 7, table_flags)) {
                const ImVec2 btn_size = ImVec2(50.0f, 50.0f);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn(); jog_button("##jog_fwd", btn_size, JogDir::Forward);
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn(); jog_button("##jog_up", btn_size, JogDir::Up);
                ImGui::TableNextColumn();

                ImGui::TableNextRow();
                ImGui::TableNextColumn(); jog_button("##jog_left", btn_size, JogDir::Left);
                ImGui::TableNextColumn();
                ImGui::TableNextColumn(); jog_button("##jog_right", btn_size, JogDir::Right);
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn(); jog_button("##jog_bck", btn_size, JogDir::Backward);
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn(); jog_button("##jog_down", btn_size, JogDir::Down);
                ImGui::TableNextColumn();

                ImGui::EndTable();
            }

            ImGui::Spacing();
            ImGui::Spacing();
            ImGui::Text("Gripper:");
            ImGui::SameLine();
            if (ImGui::Button("Open", ImVec2(100, 0))) {
                ctxt_.put(P_ + "UR:RobotiqGripper:Open.PROC", 1);
            }
            ImGui::SameLine();
            if (ImGui::Button("Close", ImVec2(100, 0))) {
                ctxt_.put(P_ + "UR:RobotiqGripper:Close.PROC", 1);
            }
            // ImGui::SameLine();
            // ImGui::Spacing();
            // ImGui::SameLine();
            // if (epics_.gripper_open) {
                // ImGui::TextColored({0.0, 1.0, 0.0, 1.0}, "Open  ");
            // } else if (epics_.gripper_closed) {
                // ImGui::TextColored({1.0, 0.0, 0.0, 1.0}, "Closed");
            // } else {
                // ImGui::TextColored({1.0, 1.0, 1.0, 1.0}, "      ");
            // }
            if (need_to_clear_disabled) {
                need_to_clear_disabled = false;
                ImGui::EndDisabled();
            }

            // Column 2: spacer ////////////////////////////////////
            ImGui::TableNextColumn();

            // Column 3: safety status + clear fault ////////////////
            ImGui::TableNextColumn();

            if (ImGui::Button("SCAN##start_scan", {100.0, 0.0})) {
                ctxt_.put(P_ + "scan2.EXSC", 1);
            }
            if (epics_.scan2_busy) {
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
                ImGui::ProgressBar(-1.0f * (float)ImGui::GetTime(), ImVec2(0.0f, 0.0f), "Scanning..");
                ImGui::PopStyleColor();
            }

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            if (ImGui::Button("ABORT##abort_scan", {100.0, 0.0})) {
                ctxt_.put(P_ + "AbortScans.PROC", 1);
            }
            ImGui::PopStyleColor(2);

            ImGui::SameLine();
            if (ImGui::Button("  ##open_video", {50.0, 0.0})) {
                std::string cmd = "/bin/bash " + IMAGE_DATA_DIR + "/play_latest.sh";
                std::system(cmd.c_str());
            }

            ImGui::NewLine();

            for (size_t i = 0; i < safety_mode_labels.size()-1; i++) {
                if (epics_.safety_status_bits & (1 << i)) {
                    const char* label = safety_mode_labels[i];
                    ImGui::Text("Safety:");
                    ImGui::SameLine();
                    if (i == 0) {
                        ImGui::TextColored({0.0, 1.0, 0.0, 1.0}, "%s", label);
                    } else {
                        ImGui::TextColored({1.0, 1.0, 0.0, 1.0}, "%s", label);
                    }
                }
            }

            ImGui::PushFont(nullptr, ImGui::GetFontSize() * 0.6f);
            if (ImGui::Button("Clear Fault##clear_fault", {180.0, 30.0})) {
                ctxt_.put(P_ + "UR:ClearFault.PROC", 1);
            }
            ImGui::PopFont();

            // Column 4: spacer ////////////////////////////////////
            ImGui::TableNextColumn();

            // Column 5: motor controls ////////////////////////////
            ImGui::TableNextColumn();

            const ImVec2 bsize = ImVec2{40.0, 40.0};
            const float input_width = 120.0f;
            float pad_y = (bsize.y - ImGui::GetFontSize()) * 0.5f;
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(ImGui::GetStyle().FramePadding.x, pad_y));
            if (ImGui::BeginTable("rot_motor_table", 3, table_flags | ImGuiTableFlags_BordersOuter)) {
                ImGui::TableSetupColumn("col1", ImGuiTableColumnFlags_WidthFixed, bsize.x);
                ImGui::TableSetupColumn("col2", ImGuiTableColumnFlags_WidthFixed, input_width);
                ImGui::TableSetupColumn("col3", ImGuiTableColumnFlags_WidthFixed, bsize.x);

                // row 1: header + RBV
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::Text("%s", epics_.m1_desc.c_str());
                ImGui::TextColored({0.0, 0.75, 1.0, 1.0},"%.2f", epics_.m1_rbv);
                ImGui::TableNextColumn();

                // row 2
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                static double m1_val_out;
                m1_val_out = epics_.m1_val;
                ImGui::SetNextItemWidth(input_width);
                if (ImGui::InputDouble("##rot_val", &m1_val_out, 0.0, 0.0, "%.2f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                    ctxt_.put(P_ + "m1.VAL", m1_val_out);
                }
                ImGui::TableNextColumn();

                // row 3
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::Button("##rot_twr", bsize)) {
                    ctxt_.put(P_ + "m1.TWR", 1);
                }
                ImGui::TableNextColumn();
                static double m1_twv_out;
                m1_twv_out = epics_.m1_twv;
                ImGui::SetNextItemWidth(input_width);
                if (ImGui::InputDouble("##rot_twv", &m1_twv_out, 0.0, 0.0, "%.2f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                    ctxt_.put(P_ + "m1.TWV", m1_twv_out);
                }
                ImGui::TableNextColumn();
                if (ImGui::Button("##rot_twf", bsize)) {
                    ctxt_.put(P_ + "m1.TWF", 1);
                }

                // row 4
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                ImGui::Dummy({15.0, 40.0});
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                if (ImGui::Button("STOP##rot_stop", {70, 40.0})) {
                    ctxt_.put(P_ + "m1.STOP", 1);
                }
                ImGui::PopStyleColor(2);

                ImGui::EndTable();
            }
            ImGui::PopStyleVar();

            ImGui::EndTable();
        }

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
