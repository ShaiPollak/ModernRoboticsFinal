#include "robot_logger.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cassert>
#include <cstdlib>

// ─────────────────────────── CSVLogger ───────────────────────────

RobotLogger::CSVLogger::CSVLogger(const std::string& filepath, const bool write_headers) {
    file_.open(filepath);
    if (!file_.is_open()) {
        std::cerr << "CSVLogger: could not open " << filepath << "\n";
        return;
    }
    file_ << std::fixed << std::setprecision(6);
    if (write_headers)
        file_ << "# chassis_phi,chassis_x,chassis_y,j1,j2,j3,j4,j5,w1,w2,w3,w4,gripper\n";
}

RobotLogger::CSVLogger::~CSVLogger() {
    if (file_.is_open()) file_.close();
}

void RobotLogger::CSVLogger::logState(const Eigen::VectorXd& q, const int gripper_state) {
    if (!file_.is_open()) return;
    assert(q.size() == 12 && "Logger expected a 12-element state vector!");
    for (int i = 0; i < q.size(); ++i) file_ << q[i] << ", ";
    file_ << gripper_state << "\n";
}

void RobotLogger::CSVLogger::logError(const std::string& msg) {
    if (file_.is_open()) file_ << "# ERROR: " << msg << "\n";
}

void RobotLogger::CSVLogger::writeConfigurationToCSVFile(const Eigen::VectorXd& q, const int gripper_state) {
    logState(q, gripper_state);
}

// ─────────────────────────── GraphLogger ───────────────────────────

RobotLogger::GraphLogger::GraphLogger(const std::string& output_dir)
    : output_dir_(output_dir) {}

void RobotLogger::GraphLogger::logStep(
    double t,
    const std::string& seg_name,
    const Eigen::VectorXd& q,
    const Eigen::Vector<double, 6>& X_err,
    const Eigen::VectorXd& controls,
    const Eigen::Matrix4d& T_desired,
    const Eigen::Matrix4d& T_actual)
{
    if (seg_name != current_segment_) {
        segment_starts_.emplace_back(t, seg_name);
        current_segment_ = seg_name;
    }
    times_.push_back(t);
    q_history_.push_back(q);
    error_history_.push_back(X_err);
    controls_history_.push_back(controls);
    ee_actual_positions_.push_back(T_actual.block<3, 1>(0, 3));
    ee_desired_positions_.push_back(T_desired.block<3, 1>(0, 3));
}

void RobotLogger::GraphLogger::generatePlots() const {
    if (times_.empty()) {
        std::cerr << "GraphLogger: no data recorded — skipping plot generation.\n";
        return;
    }
    std::string script_path = output_dir_ + "/generate_plots.py";
    std::ofstream f(script_path);
    if (!f.is_open()) {
        std::cerr << "GraphLogger: could not write script to " << script_path << "\n";
        return;
    }
    f << buildPythonScript();
    f.close();
    std::string cmd = "python3 \"" + script_path + "\"";
    int rc = std::system(cmd.c_str());
    if (rc != 0)
        std::cerr << "GraphLogger: python3 exited with code " << rc << "\n";
}

// ─────────────────── serialisation helpers ───────────────────────

std::string RobotLogger::GraphLogger::serialize(const std::vector<double>& v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) ss << ",";
        ss << v[i];
    }
    ss << "]";
    return ss.str();
}

std::string RobotLogger::GraphLogger::extractChannel(
    const std::vector<Eigen::VectorXd>& h, int idx)
{
    std::vector<double> ch;
    ch.reserve(h.size());
    for (const auto& v : h) ch.push_back(v[idx]);
    return serialize(ch);
}

std::string RobotLogger::GraphLogger::extractErrChannel(
    const std::vector<Eigen::Vector<double, 6>>& h, int idx)
{
    std::vector<double> ch;
    ch.reserve(h.size());
    for (const auto& v : h) ch.push_back(v[idx]);
    return serialize(ch);
}

std::string RobotLogger::GraphLogger::extractVec3Channel(
    const std::vector<Eigen::Vector3d>& h, int idx)
{
    std::vector<double> ch;
    ch.reserve(h.size());
    for (const auto& v : h) ch.push_back(v[idx]);
    return serialize(ch);
}

// ─────────────────── Python script builder ───────────────────────

std::string RobotLogger::GraphLogger::buildPythonScript() const {
    std::ostringstream py;

    // ── Preamble ──────────────────────────────────────────────────
    py << "import matplotlib\n"
       << "matplotlib.use('Agg')\n"
       << "import matplotlib.pyplot as plt\n"
       << "import numpy as np\n"
       << "import os\n\n";

    py << "output_dir = '" << output_dir_ << "'\n\n";

    // ── Palette constants (defined once, reused in all figures) ──
    py << "C3 = ['#e74c3c','#e67e22','#f1c40f','#2ecc71','#3498db']\n";
    py << "C4 = ['#e74c3c','#3498db','#2ecc71','#9b59b6']\n\n";

    // ── Data arrays ──────────────────────────────────────────────
    py << "t           = " << serialize(times_)                        << "\n";
    py << "chassis_phi = " << extractChannel(q_history_, 0)           << "\n";
    py << "chassis_x   = " << extractChannel(q_history_, 1)           << "\n";
    py << "chassis_y   = " << extractChannel(q_history_, 2)           << "\n\n";

    for (int i = 0; i < 5; ++i)
        py << "j" << (i+1) << " = " << extractChannel(q_history_, 3+i) << "\n";
    py << "\n";

    for (int i = 0; i < 4; ++i)
        py << "w" << (i+1) << " = " << extractChannel(q_history_, 8+i) << "\n";
    py << "\n";

    // Error: [wx, wy, wz, vx, vy, vz]
    const char* err_names[6] = {"wx","wy","wz","vx","vy","vz"};
    for (int i = 0; i < 6; ++i)
        py << "e_" << err_names[i] << " = " << extractErrChannel(error_history_, i) << "\n";
    py << "\n";

    // Controls: [w1..w4, j1..j5]  (9 total)
    int nc = controls_history_.empty() ? 0 : (int)controls_history_[0].size();
    for (int i = 0; i < nc; ++i)
        py << "u" << i << " = " << extractChannel(controls_history_, i) << "\n";
    py << "\n";

    // End-effector positions
    py << "ee_ax = " << extractVec3Channel(ee_actual_positions_,  0) << "\n";
    py << "ee_ay = " << extractVec3Channel(ee_actual_positions_,  1) << "\n";
    py << "ee_az = " << extractVec3Channel(ee_actual_positions_,  2) << "\n";
    py << "ee_dx = " << extractVec3Channel(ee_desired_positions_, 0) << "\n";
    py << "ee_dy = " << extractVec3Channel(ee_desired_positions_, 1) << "\n";
    py << "ee_dz = " << extractVec3Channel(ee_desired_positions_, 2) << "\n\n";

    // Segment start times & names
    py << "seg_t = [";
    for (const auto& [st, sn] : segment_starts_) py << st << ",";
    py << "]\n";
    py << "seg_n = [";
    for (const auto& [st, sn] : segment_starts_) py << "'" << sn << "',";
    py << "]\n\n";

    // ── Helper functions ─────────────────────────────────────────
    py << "def add_seg(ax):\n"
       << "    for st, sn in zip(seg_t, seg_n):\n"
       << "        ax.axvline(x=st, color='#bbbbbb', ls='--', lw=0.8)\n"
       << "        ax.text(st, 1.01, sn, rotation=40, fontsize=6.5, color='#666666',\n"
       << "                ha='left', va='bottom', transform=ax.get_xaxis_transform())\n\n";

    py << "def save(fig, name):\n"
       << "    fig.tight_layout()\n"
       << "    fig.savefig(os.path.join(output_dir, name), dpi=150, bbox_inches='tight')\n"
       << "    plt.close(fig)\n"
       << "    print('  Saved:', name)\n\n";

    py << "print('Generating robot performance graphs...')\n\n";
    py << "try:\n\n";

    // ── Figure 1: 2D Trajectory ───────────────────────────────────
    py << "    # 1 – 2D Trajectory\n"
       << "    fig, ax = plt.subplots(figsize=(8, 8))\n"
       << "    ax.plot(chassis_x, chassis_y, 'b-', lw=1.5, label='Chassis')\n"
       << "    ax.plot(ee_ax, ee_ay, color='#e74c3c', lw=1.2, label='EE actual')\n"
       << "    ax.plot(ee_dx, ee_dy, color='#2ecc71', ls='--', lw=1.2, label='EE desired')\n"
       << "    ax.scatter([chassis_x[0]], [chassis_y[0]], color='blue', s=100, zorder=5, label='Start')\n"
       << "    ax.scatter([chassis_x[-1]], [chassis_y[-1]], color='blue', marker='s', s=100, zorder=5, label='End')\n"
       << "    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')\n"
       << "    ax.set_title('2D Trajectory (X-Y Plane)')\n"
       << "    ax.legend(loc='best'); ax.grid(True, alpha=0.4); ax.set_aspect('equal')\n"
       << "    save(fig, 'trajectory_xy.png')\n\n";

    // ── Figure 2: Chassis states vs time ─────────────────────────
    py << "    # 2 – Chassis States\n"
       << "    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)\n"
       << "    for ax, data, label, color in zip(axes,\n"
       << "            [chassis_phi, chassis_x, chassis_y],\n"
       << "            ['phi (rad)', 'x (m)', 'y (m)'],\n"
       << "            ['#9b59b6', '#3498db', '#2ecc71']):\n"
       << "        ax.plot(t, data, color=color, lw=1.2)\n"
       << "        ax.set_ylabel(label); ax.grid(True, alpha=0.4); add_seg(ax)\n"
       << "    axes[0].set_title('Chassis States vs Time')\n"
       << "    axes[-1].set_xlabel('Time (s)')\n"
       << "    save(fig, 'chassis_states.png')\n\n";

    // ── Figure 3: Joint angles vs time ───────────────────────────
    py << "    # 3 – Joint Angles\n"
       << "    fig, ax = plt.subplots(figsize=(12, 5))\n"
       << "    for data, label, c in zip([j1,j2,j3,j4,j5], ['J1','J2','J3','J4','J5'], C3):\n"
       << "        ax.plot(t, data, color=c, lw=1.2, label=label)\n"
       << "    add_seg(ax)\n"
       << "    ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (rad)')\n"
       << "    ax.set_title('Joint Angles vs Time')\n"
       << "    ax.legend(loc='upper right'); ax.grid(True, alpha=0.4)\n"
       << "    save(fig, 'joint_angles.png')\n\n";

    // ── Figure 4: Wheel angles vs time ───────────────────────────
    py << "    # 4 – Wheel Angles\n"
       << "    fig, ax = plt.subplots(figsize=(12, 5))\n"
       << "    for data, label, c in zip([w1,w2,w3,w4], ['W1','W2','W3','W4'], C4):\n"
       << "        ax.plot(t, data, color=c, lw=1.2, label=label)\n"
       << "    add_seg(ax)\n"
       << "    ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (rad)')\n"
       << "    ax.set_title('Wheel Angles vs Time')\n"
       << "    ax.legend(loc='upper right'); ax.grid(True, alpha=0.4)\n"
       << "    save(fig, 'wheel_angles.png')\n\n";

    // ── Figure 5: Error components vs time ───────────────────────
    py << "    # 5 – Error Components\n"
       << "    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)\n"
       << "    for data, label, c in zip([e_wx,e_wy,e_wz], ['omega_x','omega_y','omega_z'], C3):\n"
       << "        axes[0].plot(t, data, color=c, lw=1.2, label=label)\n"
       << "    for data, label, c in zip([e_vx,e_vy,e_vz], ['v_x','v_y','v_z'], C4):\n"
       << "        axes[1].plot(t, data, color=c, lw=1.2, label=label)\n"
       << "    for ax in axes:\n"
       << "        add_seg(ax); ax.legend(loc='upper right'); ax.grid(True, alpha=0.4)\n"
       << "        ax.axhline(0, color='k', lw=0.5, ls=':')\n"
       << "    axes[0].set_ylabel('Angular error (rad)'); axes[0].set_title('Task-Space Error Components vs Time')\n"
       << "    axes[1].set_ylabel('Linear error (m)'); axes[1].set_xlabel('Time (s)')\n"
       << "    save(fig, 'error_components.png')\n\n";

    // ── Figure 6: Error norm vs time ─────────────────────────────
    py << "    # 6 – Error Norm\n"
       << "    err_mat  = np.array([e_wx,e_wy,e_wz,e_vx,e_vy,e_vz]).T\n"
       << "    err_norm = np.linalg.norm(err_mat, axis=1)\n"
       << "    fig, ax = plt.subplots(figsize=(12, 4))\n"
       << "    ax.plot(t, err_norm, color='#e74c3c', lw=1.5, label='||X_err||')\n"
       << "    add_seg(ax)\n"
       << "    ax.set_xlabel('Time (s)'); ax.set_ylabel('Error norm')\n"
       << "    ax.set_title('Task-Space Error Norm vs Time')\n"
       << "    ax.legend(loc='upper right'); ax.grid(True, alpha=0.4)\n"
       << "    save(fig, 'error_norm.png')\n\n";

    // ── Figure 7: Controls vs time ───────────────────────────────
    // Controls layout: [u0..u3] = wheels W1-W4, [u4..u8] = joints J1-J5
    py << "    # 7 – Control Inputs\n"
       << "    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)\n";
    if (nc >= 4) {
        py << "    for data, label, c in zip([u0,u1,u2,u3], ['W1','W2','W3','W4'], C4):\n"
           << "        axes[0].plot(t, data, color=c, lw=1.2, label=label)\n";
    }
    if (nc >= 9) {
        py << "    for data, label, c in zip([u4,u5,u6,u7,u8], ['J1','J2','J3','J4','J5'], C3):\n"
           << "        axes[1].plot(t, data, color=c, lw=1.2, label=label)\n";
    }
    py << "    for ax in axes:\n"
       << "        add_seg(ax); ax.legend(loc='upper right'); ax.grid(True, alpha=0.4)\n"
       << "    axes[0].set_ylabel('Wheel vel (rad/s)'); axes[0].set_title('Control Inputs vs Time')\n"
       << "    axes[1].set_ylabel('Joint vel (rad/s)'); axes[1].set_xlabel('Time (s)')\n"
       << "    save(fig, 'controls.png')\n\n";

    // ── Figure 8: End-effector position vs time ──────────────────
    py << "    # 8 – End-Effector Position\n"
       << "    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)\n"
       << "    for ax, act, des, label in zip(axes,\n"
       << "            [ee_ax, ee_ay, ee_az], [ee_dx, ee_dy, ee_dz],\n"
       << "            ['X (m)', 'Y (m)', 'Z (m)']):\n"
       << "        ax.plot(t, act, color='#3498db', lw=1.2, label='Actual')\n"
       << "        ax.plot(t, des, color='#e74c3c', ls='--', lw=1.2, label='Desired')\n"
       << "        ax.set_ylabel(label); ax.legend(loc='upper right')\n"
       << "        ax.grid(True, alpha=0.4); add_seg(ax)\n"
       << "    axes[0].set_title('End-Effector Position vs Time')\n"
       << "    axes[-1].set_xlabel('Time (s)')\n"
       << "    save(fig, 'ee_position.png')\n\n";

    py << "    print('All graphs saved to:', output_dir)\n\n";
    py << "except Exception as exc:\n"
       << "    import traceback\n"
       << "    print('ERROR generating graphs:', exc)\n"
       << "    traceback.print_exc()\n";

    return py.str();
}
