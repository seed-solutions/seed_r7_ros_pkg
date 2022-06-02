#include <pluginlib/class_list_macros.h>
#include "stroke_converter.h"

void seed_converter::TypeG::makeTables() {
    ROS_INFO("upper's csv_dir is %s", upper_csv_dir.c_str());
    ROS_INFO("lifter's csv_dir is %s", lifter_csv_dir.c_str());

    if (makeTable(shoulder_p.table, upper_csv_dir + "/shoulder_p.csv"))
        makeInvTable(shoulder_p.inv_table, shoulder_p.table);
    if (makeTable(shoulder_r.table, upper_csv_dir + "/shoulder_r.csv"))
        makeInvTable(shoulder_r.inv_table, shoulder_r.table);
    if (makeTable(elbow_p.table, upper_csv_dir + "/elbow_p.csv"))
        makeInvTable(elbow_p.inv_table, elbow_p.table);
    if (makeTable(wrist_p.table, upper_csv_dir + "/wrist_p.csv"))
        makeInvTable(wrist_p.inv_table, wrist_p.table);
    if (makeTable(wrist_r.table, upper_csv_dir + "/wrist_r.csv"))
        makeInvTable(wrist_r.inv_table, wrist_r.table);
    if (makeTable(neck_p.table, upper_csv_dir + "/neck_p.csv"))
        makeInvTable(neck_p.inv_table, neck_p.table);
    if (makeTable(neck_r.table, upper_csv_dir + "/neck_r.csv"))
        makeInvTable(neck_r.inv_table, neck_r.table);
    if (makeTable(waist_p.table, upper_csv_dir + "/waist_p.csv"))
        makeInvTable(waist_p.inv_table, waist_p.table);
    if (makeTable(waist_r.table, upper_csv_dir + "/waist_r.csv"))
        makeInvTable(waist_r.inv_table, waist_r.table);
    if (makeTable(leg.table, lifter_csv_dir + "/leg.csv"))
        makeInvTable(leg.inv_table, leg.table);
}

void seed_converter::TypeG::Angle2Stroke(std::vector<int16_t> &_strokes, const std::vector<double> &_angles) {
    static const float rad2Deg = 180.0 / M_PI;
    static const float scale = 100.0;

    seed_converter::DiffJoint r_wrist = setDualAngleToStroke(rad2Deg * _angles[20], -rad2Deg * _angles[19], wrist_r.table, wrist_p.table, true);
    seed_converter::DiffJoint l_wrist = setDualAngleToStroke(-rad2Deg * _angles[9], rad2Deg * _angles[8], wrist_r.table, wrist_p.table, true);
    seed_converter::DiffJoint waist = setDualAngleToStroke(-rad2Deg * _angles[2], rad2Deg * _angles[1], waist_r.table, waist_p.table, false);
    seed_converter::DiffJoint neck = setDualAngleToStroke(-rad2Deg * _angles[13], rad2Deg * _angles[12], neck_r.table, neck_p.table, false);

    _strokes[0] = static_cast<int16_t>(scale * rad2Deg * _angles[0]);
    _strokes[1] = static_cast<int16_t>(scale * waist.one);
    _strokes[2] = static_cast<int16_t>(scale * waist.two);

    _strokes[3] = static_cast<int16_t>(scale * setAngleToStroke(-rad2Deg * _angles[3], shoulder_p.table));
    _strokes[4] = static_cast<int16_t>(scale * setAngleToStroke(rad2Deg * _angles[4], shoulder_r.table));
    _strokes[5] = static_cast<int16_t>(-scale * rad2Deg * _angles[5]);
    _strokes[6] = static_cast<int16_t>(scale * setAngleToStroke(90 + rad2Deg * _angles[6], elbow_p.table));
    _strokes[7] = static_cast<int16_t>(-scale * rad2Deg * _angles[7]);
    _strokes[8] = static_cast<int16_t>(scale * l_wrist.one);
    _strokes[9] = static_cast<int16_t>(scale * l_wrist.two);
    _strokes[10] = static_cast<int16_t>(scale * (rad2Deg * _angles[10] + 50.0) * 0.18);

    _strokes[11] = static_cast<int16_t>(scale * rad2Deg * _angles[11]);
    _strokes[12] = static_cast<int16_t>(scale * neck.two);
    _strokes[13] = static_cast<int16_t>(scale * neck.one);

    _strokes[14] = static_cast<int16_t>(scale * setAngleToStroke(-rad2Deg * _angles[14], shoulder_p.table));
    _strokes[15] = static_cast<int16_t>(scale * setAngleToStroke(-rad2Deg * _angles[15], shoulder_r.table));
    _strokes[16] = static_cast<int16_t>(-scale * rad2Deg * _angles[16]);
    _strokes[17] = static_cast<int16_t>(scale * setAngleToStroke(90 + rad2Deg * _angles[17], elbow_p.table));
    _strokes[18] = static_cast<int16_t>(-scale * rad2Deg * _angles[18]);
    _strokes[19] = static_cast<int16_t>(scale * r_wrist.two);
    _strokes[20] = static_cast<int16_t>(scale * r_wrist.one);
    _strokes[21] = static_cast<int16_t>(-scale * (rad2Deg * _angles[21] - 50.0) * 0.18);

    _strokes[22] = static_cast<int16_t>(scale * setAngleToStroke(-rad2Deg * _angles[22], leg.table));  //knee
    _strokes[23] = static_cast<int16_t>(scale * setAngleToStroke(rad2Deg * _angles[23], leg.table));  //ankle
}

void seed_converter::TypeG::Stroke2Angle(std::vector<double> &_angles, const std::vector<int16_t> &_strokes) {
    static const float deg2Rad = M_PI / 180.0;
    static const float scale_inv = 0.01;

    _angles[0] = deg2Rad * scale_inv * _strokes[0];
    _angles[1] = deg2Rad * setStrokeToAngle(scale_inv * (_strokes[2] + _strokes[1]) * 0.5, waist_p.inv_table);
    _angles[2] = deg2Rad * setStrokeToAngle(scale_inv * (_strokes[2] - _strokes[1]) * 0.5, waist_r.inv_table);
    _angles[3] = -deg2Rad * setStrokeToAngle(scale_inv * _strokes[3], shoulder_p.inv_table);
    _angles[4] = deg2Rad * setStrokeToAngle(scale_inv * _strokes[4], shoulder_r.inv_table);
    _angles[5] = -deg2Rad * scale_inv * _strokes[5];
    _angles[6] = -(M_PI / 2) + deg2Rad * setStrokeToAngle(scale_inv * _strokes[6], elbow_p.inv_table);
    _angles[7] = -deg2Rad * scale_inv * _strokes[7];
    _angles[8] = -deg2Rad * setStrokeToAngle(scale_inv * (_strokes[9] - _strokes[8]) * 0.5, wrist_p.inv_table);
    _angles[9] = -deg2Rad * setStrokeToAngle(scale_inv * (_strokes[9] + _strokes[8]) * 0.5, wrist_r.inv_table);
    _angles[10] = deg2Rad * (scale_inv * _strokes[10] * 5.556 - 50.0);

    _angles[11] = deg2Rad * scale_inv * _strokes[11];
    _angles[12] = deg2Rad * setStrokeToAngle(scale_inv * (_strokes[13] + _strokes[12]) * 0.5, neck_p.inv_table);
    _angles[13] = -deg2Rad * setStrokeToAngle(scale_inv * (_strokes[13] - _strokes[12]) * 0.5, neck_r.inv_table);

    _angles[14] = -deg2Rad * setStrokeToAngle(scale_inv * _strokes[14], shoulder_p.inv_table);
    _angles[15] = -deg2Rad * setStrokeToAngle(scale_inv * _strokes[15], shoulder_r.inv_table);
    _angles[16] = -deg2Rad * scale_inv * _strokes[16];
    _angles[17] = -(M_PI / 2) + deg2Rad * setStrokeToAngle(scale_inv * _strokes[17], elbow_p.inv_table);
    _angles[18] = -deg2Rad * scale_inv * _strokes[18];
    _angles[19] = -deg2Rad * setStrokeToAngle(scale_inv * (_strokes[20] - _strokes[19]) * 0.5, wrist_p.inv_table);
    _angles[20] = deg2Rad * setStrokeToAngle(scale_inv * (_strokes[20] + _strokes[19]) * 0.5, wrist_r.inv_table);
    _angles[21] = -deg2Rad * (scale_inv * _strokes[21] * 5.556 - 50.0);

    _angles[22] = -deg2Rad * setStrokeToAngle(scale_inv * _strokes[22], leg.inv_table);  // knee
    _angles[23] = deg2Rad * setStrokeToAngle(scale_inv * _strokes[23], leg.inv_table);  // ankle
}

PLUGINLIB_EXPORT_CLASS(seed_converter::TypeG, seed_converter::StrokeConverter)
