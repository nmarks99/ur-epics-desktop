#pragma once
#include <array>
#include <filesystem>
#include <vector>

#include "rl_utils.hpp"
#include "raymath.h"

inline std::filesystem::path get_resource_dir() {
#ifndef UR_RESOURCE_DIR
    if (const char* env = std::getenv("UR_RESOURCE_DIR")) {
        return std::filesystem::path(env);
    } else {
        throw std::runtime_error("UR_RESOURCE_DIR not set");
    }
#endif
    return std::filesystem::path(UR_RESOURCE_DIR);
}

constexpr int UR_NUM_AXES = 6;
// constexpr int UR_NUM_MODELS = UR_NUM_AXES + 2; // 6 axes/links plus base and tool
constexpr int UR_NUM_MODELS = UR_NUM_AXES + 4; // 6 axes/links plus base, tool, 2 fingers

constexpr std::array<std::string_view, UR_NUM_MODELS> UR_MODEL_LABELS = {
    "Base", "Shoulder", "Upperarm", "Forearm", "Wrist1", "Wrist2", "Wrist3", "Tool", "LeftFinger", "RightFinger"
};

enum class URVersion : int {
    UR3e,
    UR5e,
};

class UR {
  public:
    UR(URVersion version);
    void draw();
    void draw(int mask, bool opaque = false);
    void draw_axes();
    void draw_axes(int mask);
    void update(const std::vector<double> &joint_angles, double gripper_pos);

  private:
    RLModel &at(int index);

    template <typename Func> void for_each_model(Func func) {
        for (int i = 0; i < UR_NUM_MODELS; i++) {
            func(at(i));
        }
    }

    std::filesystem::path model_dir_;
    URVersion version_;
    std::array<Matrix, UR_NUM_MODELS> tfs_;

    RLModel base_;
    RLModel shoulder_;
    RLModel upperarm_;
    RLModel forearm_;
    RLModel wrist1_;
    RLModel wrist2_;
    RLModel wrist3_;
    RLModel tool_;
    RLModel finger_left_;
    RLModel finger_right_;
};

namespace UR3e {
const Matrix TSBASE = MatrixRotateXYZ({-PI / 2, 0.0, PI});

const Matrix TB1 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.15), MatrixRotateZ(PI));

const Matrix T12 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.12), MatrixRotateXYZ({-PI / 2, 0.0, -PI / 2}));

const Matrix T23 = MatrixTranslate(0.0, 0.245, -0.09);

const Matrix T34 = MatrixMultiply(MatrixTranslate(-0.212, 0.0, 0.1), MatrixRotateXYZ({0.0, 0.0, -PI / 2}));

const Matrix T45 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.085), MatrixRotateXYZ({PI / 2, 0.0, PI}));

const Matrix T56 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.072), MatrixRotateXYZ({PI / 2, 0.0, 0.0}));

// const Matrix T6TOOL = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.10), MatrixRotateXYZ({0.0, 0.0, PI / 2}));
const Matrix T6TOOL = MatrixMultiply(MatrixTranslate(0.0, 0.02, 0.0), MatrixRotateXYZ({PI/2, PI/2, 0.0}));

// const Matrix TOOLFL = MatrixMultiply(MatrixTranslate(0.0, 0.02, 0.093), MatrixRotateXYZ({-PI/2, 0.0, 0.0}));
const Matrix TOOLFL = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.093), MatrixRotateXYZ({-PI/2, 0.0, 0.0}));

// const Matrix TOOLFR = MatrixMultiply(MatrixTranslate(0.0, -0.02, 0.093), MatrixRotateXYZ({-PI/2, 0.0, 0.0}));
const Matrix TOOLFR = MatrixMultiply(MatrixTranslate(0.0, -0.0, 0.093), MatrixRotateXYZ({-PI/2, 0.0, 0.0}));

} // namespace UR3e

namespace UR5e {
const Matrix TSBASE = MatrixRotateXYZ({-PI / 2, 0.0, PI});

const Matrix TB1 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.086), MatrixRotateZ(PI));

const Matrix T12 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.135), MatrixRotateXYZ({-PI / 2, 0.0, -PI / 2}));

const Matrix T23 = MatrixTranslate(0.0, 0.425, -0.12);

const Matrix T34 =
    MatrixMultiply(MatrixTranslate(-0.3925, 0.0, 0.0925), MatrixRotateXYZ({0.0, 0.0, -PI / 2}));

const Matrix T45 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.09), MatrixRotateXYZ({PI / 2, 0.0, PI}));

const Matrix T56 = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.062), MatrixRotateXYZ({PI / 2, 0.0, 0.0}));

const Matrix T6TOOL = MatrixMultiply(MatrixTranslate(0.0, 0.0, 0.095), MatrixRotateXYZ({0.0, 0.0, PI / 2}));
} // namespace UR5e
