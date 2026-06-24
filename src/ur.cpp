#include "ur.hpp"

inline std::filesystem::path get_model_dir(URVersion version) {
    auto model_dir = get_resource_dir();
    switch (version) {
    case URVersion::UR3e:
        model_dir /= "UR3e";
        break;
    case URVersion::UR5e:
        model_dir /= "UR5e";
        break;
    }
    return model_dir;
}

std::array<Matrix, UR_NUM_MODELS> get_tfs(URVersion version) {
    std::array<Matrix, UR_NUM_MODELS> tfs;
    switch (version) {
    case URVersion::UR3e:
        tfs = {
            UR3e::TSBASE, UR3e::TB1, UR3e::T12, UR3e::T23, UR3e::T34, UR3e::T45, UR3e::T56, UR3e::T6TOOL, UR3e::TOOLFL, UR3e::TOOLFR,
        };
        break;
    case URVersion::UR5e:
        tfs = {
            UR5e::TSBASE, UR5e::TB1, UR5e::T12, UR5e::T23, UR5e::T34, UR5e::T45, UR5e::T56, UR5e::T6TOOL, UR3e::TOOLFL, UR3e::TOOLFR
        };
        break;
    }
    return tfs;
}

UR::UR(URVersion version)
    : model_dir_(get_model_dir(version)), version_(version),
      base_(model_dir_ / "base.obj", UR_MODEL_LABELS.at(0).data()),
      shoulder_(model_dir_ / "shoulder.obj", UR_MODEL_LABELS.at(1).data()),
      upperarm_(model_dir_ / "upperarm.obj", UR_MODEL_LABELS.at(2).data()),
      forearm_(model_dir_ / "forearm.obj", UR_MODEL_LABELS.at(3).data()),
      wrist1_(model_dir_ / "wrist1.obj", UR_MODEL_LABELS.at(4).data()),
      wrist2_(model_dir_ / "wrist2.obj", UR_MODEL_LABELS.at(5).data()),
      wrist3_(model_dir_ / "wrist3.obj", UR_MODEL_LABELS.at(6).data()),
      tool_(model_dir_ / "../robotiq/robotiq_hand-e_body.obj", UR_MODEL_LABELS.at(7).data()),
      finger_left_(model_dir_ / "../robotiq/robotiq_hand-e_finger-left.obj", UR_MODEL_LABELS.at(8).data()),
      finger_right_(model_dir_ / "../robotiq/robotiq_hand-e_finger-right.obj", UR_MODEL_LABELS.at(9).data()) {

    version_ = version;
    model_dir_ = get_model_dir(version);

    tfs_ = get_tfs(version);

    // apply initial static transform
    this->at(0).model.transform = tfs_.at(0);
    for (int i = 1; i < UR_NUM_MODELS; i++) {
        this->at(i).model.transform = MatrixMultiply(tfs_.at(i), this->at(i - 1).model.transform);
    }
}

void UR::update(const std::vector<double>& joint_angles, double gripper_pos) {
    shoulder_.model.transform =
        MatrixMultiply(MatrixMultiply(MatrixRotateZ(joint_angles.at(0)), tfs_.at(1)), base_.model.transform);
    upperarm_.model.transform = MatrixMultiply(MatrixMultiply(MatrixRotateZ(joint_angles.at(1)), tfs_.at(2)),
                                               shoulder_.model.transform);
    forearm_.model.transform = MatrixMultiply(MatrixMultiply(MatrixRotateZ(joint_angles.at(2)), tfs_.at(3)),
                                              upperarm_.model.transform);
    wrist1_.model.transform = MatrixMultiply(MatrixMultiply(MatrixRotateZ(joint_angles.at(3)), tfs_.at(4)),
                                             forearm_.model.transform);
    wrist2_.model.transform = MatrixMultiply(MatrixMultiply(MatrixRotateZ(joint_angles.at(4)), tfs_.at(5)),
                                             wrist1_.model.transform);
    wrist3_.model.transform = MatrixMultiply(MatrixMultiply(MatrixRotateZ(joint_angles.at(5)), tfs_.at(6)),
                                             wrist2_.model.transform);
    tool_.model.transform = MatrixMultiply(tfs_.at(7), wrist3_.model.transform);
    gripper_pos = gripper_pos * 0.025;
    finger_left_.model.transform = MatrixMultiply(MatrixTranslate(0.0, gripper_pos, 0.0), MatrixMultiply(tfs_.at(8), tool_.model.transform));
    finger_right_.model.transform = MatrixMultiply(MatrixTranslate(0.0, -gripper_pos, 0.0), MatrixMultiply(tfs_.at(9), tool_.model.transform));
}

void UR::draw() {
    for_each_model([](RLModel& model) { model.draw(); });
}

void UR::draw(int mask, bool opaque) {
    int i = 0;
    for_each_model([&](RLModel& model) {
        if (mask & (1 << i)) {
            if (opaque) {
                model.draw_wires(ColorAlpha(WHITE, 0.5));
            } else {
                model.draw_wires();
            }
        } else {
            if (opaque) {
                model.draw(ColorAlpha(WHITE, 0.5));
            } else {
                model.draw();
            }
        }
        i++;
    });
}

void UR::draw_axes() {
    for_each_model([](RLModel& model) { model.draw_axes(); });
}

void UR::draw_axes(int mask) {
    int i = 0;
    for_each_model([&](RLModel& model) {
        if (mask & (1 << i)) {
            model.draw_axes();
        }
        i++;
    });
}

RLModel& UR::at(int i) {
    switch (i) {
    case 0:
        return base_;
    case 1:
        return shoulder_;
    case 2:
        return upperarm_;
    case 3:
        return forearm_;
    case 4:
        return wrist1_;
    case 5:
        return wrist2_;
    case 6:
        return wrist3_;
    case 7:
        return tool_;
    case 8:
        return finger_left_;
    case 9:
        return finger_right_;
    default:
        throw std::out_of_range("Index must be 0 to 9");
    }
}
