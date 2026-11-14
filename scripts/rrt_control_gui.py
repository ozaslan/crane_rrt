#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


ARENA_PRESETS = {
    "Arena1_ImageIndustrial": (-3.8, 3.5, 3.8, -3.5, False, 1.25),
    "Arena2_SlalomCorridor": (-3.8, -3.5, 3.8, 3.5, False, 1.25),
    "Arena3_DensePillarYard": (-3.8, 3.5, 3.8, -3.5, False, 1.25),
    "Arena4_PayloadClutterCorridor": (-3.8, -3.5, 3.8, 3.5, True, 1.25),
    "none": (-3.8, 3.5, 3.8, -3.5, False, 1.25),
}


METHOD_PRESETS = {
    "M7_ProposedFull": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.35, w_pos=0.85, w_vel=0.15,
        gbias=0.15, smin=1, smax=15, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=20000,
    ),
    "M7_RandSub": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.35, w_pos=0.85, w_vel=0.15,
        gbias=0.15, smin=1, smax=15, randsub=True, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=20000,
    ),
    "M7_ForceHist": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.35, w_pos=0.85, w_vel=0.15,
        gbias=0.15, smin=1, smax=15, randsub=False, w_u=0.05, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=20000,
    ),
    "M1_Vanilla": dict(
        min_dur=0.10, max_dur=0.12, smooth=False, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.0, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=False, max_iter=20000,
    ),
    "M2_SustainedTf": dict(
        min_dur=0.20, max_dur=0.50, smooth=False, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.0, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=False, max_iter=20000,
    ),
    "M3_ForceSmooth": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.0, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=False, max_iter=20000,
    ),
    "M4_GoalBiasBrake": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.15, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=20000,
    ),
    "M5_4DPhaseOnly": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.35, w_pos=0.85, w_vel=0.15,
        gbias=0.15, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=20000,
    ),
    "M6_StrideOnly": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.15, smin=1, smax=15, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=20000,
    ),
    "B1_LaValleBestK": dict(
        min_dur=0.20, max_dur=0.50, smooth=False, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.15, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=5,
        cell=False, pd=False, brake=False, max_iter=20000,
    ),
    "B2_ZhouCellRRT": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.15, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=True, pd=False, brake=True, max_iter=20000,
    ),
    "B3_PDSteerRRT": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.15, smin=1, smax=1, randsub=False, w_u=0.0, sst=False, k_ctrl=1,
        cell=False, pd=True, brake=True, max_iter=20000,
    ),
    "B4_SST": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.0, w_pos=1.0, w_vel=0.0,
        gbias=0.15, smin=1, smax=1, randsub=False, w_u=0.0, sst=True, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=25000,
    ),
    "B4_SST_4D": dict(
        min_dur=0.20, max_dur=0.50, smooth=True, fbias=0.35, w_pos=0.85, w_vel=0.15,
        gbias=0.15, smin=1, smax=1, randsub=False, w_u=0.0, sst=True, k_ctrl=1,
        cell=False, pd=False, brake=True, max_iter=25000,
    ),
}


class RRTControlROSNode(Node):
    def __init__(self):
        super().__init__("rrt_control_gui_node")
        self.declare_parameter("arena_file", "Arena1_ImageIndustrial")
        self.initial_arena = (
            self.get_parameter("arena_file").get_parameter_value().string_value
        )
        self.cmd_pub = self.create_publisher(String, "/gantry_crane/rrt_command", 10)
        self.status_sub = self.create_subscription(
            String, "/gantry_crane/rrt_status", self._status_cb, 10
        )
        self.latest_status = {}

    def _status_cb(self, msg: String):
        data = {}
        for line in msg.data.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                data[k.strip()] = v.strip()
        self.latest_status = data

    def send_command(self, payload: str):
        msg = String()
        msg.data = payload
        self.cmd_pub.publish(msg)


class RRTControlWindow(QMainWindow):
    def __init__(self, ros_node: RRTControlROSNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("3D Gantry Crane — Kinodynamic RRT Control Panel (ECC 2027)")
        self.resize(650, 900)

        self.setStyleSheet("""
            QMainWindow { background-color: #f5f6fa; }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #dcdde1;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #2f3640;
            }
            QPushButton {
                border-radius: 5px;
                padding: 8px 12px;
                font-weight: bold;
                font-size: 12px;
            }
            QTabWidget::pane {
                border: 1px solid #dcdde1;
                border-radius: 6px;
                background: #ffffff;
            }
            QTabBar::tab {
                padding: 8px 14px;
                font-weight: bold;
            }
            QTabBar::tab:selected {
                background: #273c75;
                color: #ffffff;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        main_layout.addWidget(self._build_status_group())
        main_layout.addWidget(self._build_action_buttons_group())

        self.tabs = QTabWidget()
        self.tabs.addTab(self._wrap_scroll(self._build_rrt_tab()), "1. Planner & Presets")
        self.tabs.addTab(self._wrap_scroll(self._build_model_solver_tab()), "2. Crane & Solver")
        self.tabs.addTab(self._wrap_scroll(self._build_arena_vis_tab()), "3. Arena & RViz")
        main_layout.addWidget(self.tabs, stretch=1)

        init_arena = self.ros_node.initial_arena
        if init_arena:
            if self.cmb_arena.findText(init_arena) < 0:
                self.cmb_arena.addItem(init_arena)
            self.cmb_arena.setCurrentText(init_arena)
            self._on_arena_preset_changed(init_arena)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(50)

    def _wrap_scroll(self, widget: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(widget)
        return scroll

    def _build_status_group(self) -> QGroupBox:
        box = QGroupBox("Live Telemetry")
        grid = QGridLayout(box)

        self.lbl_mode = QLabel("INITIALIZING...")
        self.lbl_mode.setFont(QFont("Sans", 11, QFont.Bold))
        self.lbl_mode.setStyleSheet("color: #192a56; padding: 4px;")

        self.lbl_iter = QLabel("Iterations: 0 / 20000")
        self.lbl_nodes = QLabel("Active Nodes: 1 / 6000")
        self.lbl_coll = QLabel("Rejected Rollouts: 0")
        self.lbl_dist = QLabel("Best Goal Dist: — m")
        self.lbl_play = QLabel("Playback Step: 0 / 0")
        self.lbl_state = QLabel("Trolley (x, y): (0.00, 0.00) m | Swing (α, β): (0.0°, 0.0°)")
        self.lbl_force = QLabel("Force (Fx, Fy): (0.00, 0.00) N")

        grid.addWidget(QLabel("Status:"), 0, 0)
        grid.addWidget(self.lbl_mode, 0, 1, 1, 3)
        grid.addWidget(self.lbl_iter, 1, 0, 1, 2)
        grid.addWidget(self.lbl_nodes, 1, 2, 1, 2)
        grid.addWidget(self.lbl_coll, 2, 0, 1, 2)
        grid.addWidget(self.lbl_dist, 2, 2, 1, 2)
        grid.addWidget(self.lbl_play, 3, 0, 1, 4)
        grid.addWidget(self.lbl_state, 4, 0, 1, 4)
        grid.addWidget(self.lbl_force, 5, 0, 1, 4)
        return box

    def _build_action_buttons_group(self) -> QGroupBox:
        box = QGroupBox("Execution Controls")
        layout = QGridLayout(box)

        btn_reset = QPushButton("▶ Apply & Reset Exploration")
        btn_reset.setStyleSheet("background-color: #2ecc71; color: white;")
        btn_reset.clicked.connect(self.on_apply_and_reset)

        btn_new_seed = QPushButton("🎲 New Random Seed & Reset")
        btn_new_seed.setStyleSheet("background-color: #2980b9; color: white;")
        btn_new_seed.clicked.connect(self.on_new_seed_and_reset)

        self.btn_pause = QPushButton("⏸ Pause / Resume")
        self.btn_pause.setStyleSheet("background-color: #f39c12; color: white;")
        self.btn_pause.clicked.connect(lambda: self.send_action("toggle_pause"))

        btn_step = QPushButton("⏭ Step Batch")
        btn_step.setStyleSheet("background-color: #8e44ad; color: white;")
        btn_step.clicked.connect(lambda: self.send_action("step_once"))

        btn_replay = QPushButton("🔄 Restart Playback")
        btn_replay.setStyleSheet("background-color: #16a085; color: white;")
        btn_replay.clicked.connect(lambda: self.send_action("restart_playback"))

        btn_clear_hist = QPushButton("🧹 Clear RViz Trail")
        btn_clear_hist.setStyleSheet("background-color: #7f8c8d; color: white;")
        btn_clear_hist.clicked.connect(lambda: self.send_action("clear_history"))

        btn_apply_only = QPushButton("⚙ Apply Parameters Live (Without Reset)")
        btn_apply_only.setStyleSheet("background-color: #34495e; color: white;")
        btn_apply_only.clicked.connect(lambda: self.send_action("apply_only"))

        layout.addWidget(btn_reset, 0, 0, 1, 2)
        layout.addWidget(btn_new_seed, 1, 0, 1, 2)
        layout.addWidget(self.btn_pause, 2, 0)
        layout.addWidget(btn_step, 2, 1)
        layout.addWidget(btn_replay, 3, 0)
        layout.addWidget(btn_clear_hist, 3, 1)
        layout.addWidget(btn_apply_only, 4, 0, 1, 2)
        return box

    def _make_dspin(self, val, low, high, step, decimals=3):
        sp = QDoubleSpinBox()
        sp.setDecimals(decimals)
        sp.setRange(low, high)
        sp.setSingleStep(step)
        sp.setValue(val)
        return sp

    def _make_ispin(self, val, low, high, step=1):
        sp = QSpinBox()
        sp.setRange(low, high)
        sp.setSingleStep(step)
        sp.setValue(val)
        return sp

    def _build_rrt_tab(self) -> QWidget:
        w = QWidget()
        vbox = QVBoxLayout(w)

        g_pre = QGroupBox("ECC 2027 Benchmark Presets")
        grid_p = QGridLayout(g_pre)
        self.cmb_preset = QComboBox()
        self.cmb_preset.addItems(list(METHOD_PRESETS.keys()))
        self.cmb_preset.currentTextChanged.connect(self._on_method_preset_changed)

        self.cmb_arena = QComboBox()
        self.cmb_arena.addItems(list(ARENA_PRESETS.keys()))
        self.cmb_arena.currentTextChanged.connect(self._on_arena_preset_changed)

        grid_p.addWidget(QLabel("Method Preset (Table II):"), 0, 0)
        grid_p.addWidget(self.cmb_preset, 0, 1)
        grid_p.addWidget(QLabel("Obstacle Arena:"), 1, 0)
        grid_p.addWidget(self.cmb_arena, 1, 1)
        vbox.addWidget(g_pre)

        g_sg = QGroupBox("Start, Goal & Workspace Bounds [m]")
        grid_sg = QGridLayout(g_sg)
        self.sp_start_x = self._make_dspin(-3.8, -10.0, 10.0, 0.1, 2)
        self.sp_start_y = self._make_dspin(3.5, -10.0, 10.0, 0.1, 2)
        self.sp_goal_x = self._make_dspin(3.8, -10.0, 10.0, 0.1, 2)
        self.sp_goal_y = self._make_dspin(-3.5, -10.0, 10.0, 0.1, 2)
        self.sp_x_min = self._make_dspin(-4.5, -15.0, 0.0, 0.5, 2)
        self.sp_x_max = self._make_dspin(4.5, 0.0, 15.0, 0.5, 2)
        self.sp_y_min = self._make_dspin(-4.5, -15.0, 0.0, 0.5, 2)
        self.sp_y_max = self._make_dspin(4.5, 0.0, 15.0, 0.5, 2)

        grid_sg.addWidget(QLabel("Start X [m]:"), 0, 0)
        grid_sg.addWidget(self.sp_start_x, 0, 1)
        grid_sg.addWidget(QLabel("Start Y [m]:"), 0, 2)
        grid_sg.addWidget(self.sp_start_y, 0, 3)
        grid_sg.addWidget(QLabel("Goal X [m]:"), 1, 0)
        grid_sg.addWidget(self.sp_goal_x, 1, 1)
        grid_sg.addWidget(QLabel("Goal Y [m]:"), 1, 2)
        grid_sg.addWidget(self.sp_goal_y, 1, 3)
        grid_sg.addWidget(QLabel("X Bounds [min, max]:"), 2, 0)
        grid_sg.addWidget(self.sp_x_min, 2, 1)
        grid_sg.addWidget(self.sp_x_max, 2, 2)
        grid_sg.addWidget(QLabel("Y Bounds [min, max]:"), 3, 0)
        grid_sg.addWidget(self.sp_y_min, 3, 1)
        grid_sg.addWidget(self.sp_y_max, 3, 2)
        vbox.addWidget(g_sg)

        g_heur = QGroupBox("Kinodynamic Steering & Nearest-Neighbor Parameters")
        grid_h = QGridLayout(g_heur)
        self.sp_w_pos = self._make_dspin(0.85, 0.0, 5.0, 0.05, 2)
        self.sp_w_vel = self._make_dspin(0.15, 0.0, 5.0, 0.05, 2)
        self.sp_w_u = self._make_dspin(0.00, 0.0, 2.0, 0.01, 2)
        self.sp_goal_bias = self._make_dspin(0.15, 0.0, 1.0, 0.05, 2)
        self.sp_force_bias = self._make_dspin(0.35, 0.0, 1.0, 0.05, 2)
        self.sp_max_force = self._make_dspin(0.50, 0.05, 10.0, 0.05, 2)
        self.sp_min_dur = self._make_dspin(0.20, 0.02, 5.0, 0.05, 2)
        self.sp_max_dur = self._make_dspin(0.50, 0.02, 5.0, 0.05, 2)
        self.chk_smoothing = QCheckBox("Piecewise-Constant ZOH Force Smoothing")
        self.chk_smoothing.setChecked(True)
        self.sp_min_step = self._make_ispin(1, 1, 200, 1)
        self.sp_max_step = self._make_ispin(15, 1, 500, 1)
        self.chk_randsub = QCheckBox("Equal-Budget Uniform Random Subset (M7-RandSub)")
        self.chk_randsub.setChecked(False)
        self.chk_sst = QCheckBox("Stable Sparse RRT (SST) BestFirst + Witness Pruning")
        self.chk_sst.setChecked(False)
        self.sp_sst_bn = self._make_dspin(0.45, 0.05, 3.0, 0.05, 2)
        self.sp_sst_s = self._make_dspin(0.18, 0.02, 2.0, 0.02, 2)
        self.sp_k_ctrl = self._make_ispin(1, 1, 20, 1)
        self.chk_cell = QCheckBox("3×3 Cell-Partitioned NNS (B2)")
        self.chk_pd = QCheckBox("PD + Anti-Sway Steering (B3)")
        self.chk_braking = QCheckBox("Near-Goal PD Braking & Sway Damping")
        self.chk_braking.setChecked(True)
        self.sp_brake_rad = self._make_dspin(0.85, 0.05, 5.0, 0.05, 2)
        self.sp_brake_prob = self._make_dspin(0.60, 0.0, 1.0, 0.05, 2)
        self.chk_soft_corridor = QCheckBox("Soft Crane Corridor Guidance (Arena 4)")
        self.chk_soft_corridor.setChecked(False)
        self.sp_corr_width = self._make_dspin(1.25, 0.25, 4.5, 0.05, 2)

        grid_h.addWidget(QLabel("w_pos / w_vel:"), 0, 0)
        hl_w = QHBoxLayout()
        hl_w.addWidget(self.sp_w_pos)
        hl_w.addWidget(self.sp_w_vel)
        grid_h.addLayout(hl_w, 0, 1)
        grid_h.addWidget(QLabel("Force-Hist w_u:"), 0, 2)
        grid_h.addWidget(self.sp_w_u, 0, 3)
        grid_h.addWidget(QLabel("Goal Bias:"), 1, 0)
        grid_h.addWidget(self.sp_goal_bias, 1, 1)
        grid_h.addWidget(QLabel("Force Bias p_steer:"), 1, 2)
        grid_h.addWidget(self.sp_force_bias, 1, 3)
        grid_h.addWidget(QLabel("Max Force [N]:"), 2, 0)
        grid_h.addWidget(self.sp_max_force, 2, 1)
        grid_h.addWidget(QLabel("Duration [min, max] s:"), 2, 2)
        hl_dur = QHBoxLayout()
        hl_dur.addWidget(self.sp_min_dur)
        hl_dur.addWidget(self.sp_max_dur)
        grid_h.addLayout(hl_dur, 2, 3)
        grid_h.addWidget(self.chk_smoothing, 3, 0, 1, 4)
        grid_h.addWidget(QLabel("Stride [s_min, s_max]:"), 4, 0)
        hl_step = QHBoxLayout()
        hl_step.addWidget(self.sp_min_step)
        hl_step.addWidget(self.sp_max_step)
        grid_h.addLayout(hl_step, 4, 1)
        grid_h.addWidget(QLabel("Best-of-K Controls:"), 4, 2)
        grid_h.addWidget(self.sp_k_ctrl, 4, 3)
        grid_h.addWidget(self.chk_randsub, 5, 0, 1, 4)
        grid_h.addWidget(self.chk_sst, 6, 0, 1, 2)
        grid_h.addWidget(QLabel("SST δ_bn / δ_s [m]:"), 6, 2)
        hl_sst = QHBoxLayout()
        hl_sst.addWidget(self.sp_sst_bn)
        hl_sst.addWidget(self.sp_sst_s)
        grid_h.addLayout(hl_sst, 6, 3)
        grid_h.addWidget(self.chk_cell, 7, 0, 1, 2)
        grid_h.addWidget(self.chk_pd, 7, 2, 1, 2)
        grid_h.addWidget(self.chk_braking, 8, 0, 1, 2)
        grid_h.addWidget(QLabel("Brake r [m] / p:"), 8, 2)
        hl_brk = QHBoxLayout()
        hl_brk.addWidget(self.sp_brake_rad)
        hl_brk.addWidget(self.sp_brake_prob)
        grid_h.addLayout(hl_brk, 8, 3)
        grid_h.addWidget(self.chk_soft_corridor, 9, 0, 1, 2)
        grid_h.addWidget(QLabel("Corridor W_corr [m]:"), 9, 2)
        grid_h.addWidget(self.sp_corr_width, 9, 3)
        vbox.addWidget(g_heur)

        g_lim = QGroupBox("Terminal Safe Set, Path Constraints & Seed")
        grid_l = QGridLayout(g_lim)
        self.sp_goal_tol = self._make_dspin(0.35, 0.05, 2.0, 0.05, 2)
        self.sp_goal_vel_tol = self._make_dspin(0.55, 0.05, 5.0, 0.05, 2)
        self.sp_goal_sw_tol = self._make_dspin(0.22, 0.02, 1.5, 0.02, 3)
        self.sp_goal_om_tol = self._make_dspin(0.55, 0.05, 5.0, 0.05, 2)
        self.chk_req_vel = QCheckBox("Enforce Terminal Velocity Limit")
        self.chk_req_vel.setChecked(True)
        self.chk_req_sw = QCheckBox("Enforce Terminal Swing Limit")
        self.chk_req_sw.setChecked(True)
        self.sp_max_vel = self._make_dspin(2.00, 0.2, 10.0, 0.1, 2)
        self.sp_max_swing = self._make_dspin(0.4363, 0.1, 1.57, 0.05, 4)
        self.sp_dt = self._make_dspin(0.020, 0.005, 0.1, 0.005, 3)
        self.sp_batch = self._make_ispin(20, 1, 2000, 5)
        self.sp_max_iter = self._make_ispin(20000, 100, 500000, 5000)
        self.sp_max_nodes = self._make_ispin(6000, 100, 200000, 1000)
        self.sp_seed = self._make_ispin(1001, 0, 999999, 1)
        self.chk_rope_col = QCheckBox("10-Sphere Cable Collision Check")
        self.chk_rope_col.setChecked(True)
        self.chk_loop = QCheckBox("Loop Trajectory Playback")
        self.chk_loop.setChecked(True)

        grid_l.addWidget(QLabel("Goal r_goal [m]:"), 0, 0)
        grid_l.addWidget(self.sp_goal_tol, 0, 1)
        grid_l.addWidget(QLabel("Goal v_max [m/s]:"), 0, 2)
        grid_l.addWidget(self.sp_goal_vel_tol, 0, 3)
        grid_l.addWidget(QLabel("Goal θ_max [rad]:"), 1, 0)
        grid_l.addWidget(self.sp_goal_sw_tol, 1, 1)
        grid_l.addWidget(QLabel("Goal ω_max [rad/s]:"), 1, 2)
        grid_l.addWidget(self.sp_goal_om_tol, 1, 3)
        grid_l.addWidget(self.chk_req_vel, 2, 0, 1, 2)
        grid_l.addWidget(self.chk_req_sw, 2, 2, 1, 2)
        grid_l.addWidget(QLabel("Max Trolley v [m/s]:"), 3, 0)
        grid_l.addWidget(self.sp_max_vel, 3, 1)
        grid_l.addWidget(QLabel("Max Swing θ [rad]:"), 3, 2)
        grid_l.addWidget(self.sp_max_swing, 3, 3)
        grid_l.addWidget(QLabel("RK4 Step h [s]:"), 4, 0)
        grid_l.addWidget(self.sp_dt, 4, 1)
        grid_l.addWidget(QLabel("Batch Iters/Tick:"), 4, 2)
        grid_l.addWidget(self.sp_batch, 4, 3)
        grid_l.addWidget(QLabel("Max Iterations:"), 5, 0)
        grid_l.addWidget(self.sp_max_iter, 5, 1)
        grid_l.addWidget(QLabel("Max Active Nodes:"), 5, 2)
        grid_l.addWidget(self.sp_max_nodes, 5, 3)
        grid_l.addWidget(QLabel("Random Seed:"), 6, 0)
        grid_l.addWidget(self.sp_seed, 6, 1)
        grid_l.addWidget(self.chk_rope_col, 7, 0, 1, 2)
        grid_l.addWidget(self.chk_loop, 7, 2, 1, 2)
        vbox.addWidget(g_lim)

        vbox.addStretch(1)
        return w

    def _build_model_solver_tab(self) -> QWidget:
        w = QWidget()
        vbox = QVBoxLayout(w)

        g_mod = QGroupBox("4-DOF Gantry Crane Dynamic Parameters")
        grid_m = QGridLayout(g_mod)
        self.sp_mx = self._make_dspin(0.10, 0.01, 50.0, 0.05, 3)
        self.sp_my = self._make_dspin(0.10, 0.01, 50.0, 0.05, 3)
        self.sp_m = self._make_dspin(0.10, 0.01, 50.0, 0.05, 3)
        self.sp_ell = self._make_dspin(1.00, 0.10, 10.0, 0.10, 2)
        self.sp_g = self._make_dspin(9.81, 0.10, 25.0, 0.10, 2)
        self.sp_dx = self._make_dspin(0.05, 0.00, 10.0, 0.01, 3)
        self.sp_dy = self._make_dspin(0.05, 0.00, 10.0, 0.01, 3)
        self.sp_ball_rad = self._make_dspin(0.12, 0.02, 1.0, 0.02, 2)

        grid_m.addWidget(QLabel("Trolley Mass m_x [kg]:"), 0, 0)
        grid_m.addWidget(self.sp_mx, 0, 1)
        grid_m.addWidget(QLabel("Bridge Mass m_y [kg]:"), 0, 2)
        grid_m.addWidget(self.sp_my, 0, 3)
        grid_m.addWidget(QLabel("Payload Mass m [kg]:"), 1, 0)
        grid_m.addWidget(self.sp_m, 1, 1)
        grid_m.addWidget(QLabel("Cable Length ℓ [m]:"), 1, 2)
        grid_m.addWidget(self.sp_ell, 1, 3)
        grid_m.addWidget(QLabel("Damping D_x [N·s/m]:"), 2, 0)
        grid_m.addWidget(self.sp_dx, 2, 1)
        grid_m.addWidget(QLabel("Damping D_y [N·s/m]:"), 2, 2)
        grid_m.addWidget(self.sp_dy, 2, 3)
        grid_m.addWidget(QLabel("Gravity g [m/s²]:"), 3, 0)
        grid_m.addWidget(self.sp_g, 3, 1)
        grid_m.addWidget(QLabel("Payload Radius r_p [m]:"), 3, 2)
        grid_m.addWidget(self.sp_ball_rad, 3, 3)
        vbox.addWidget(g_mod)

        g_sol = QGroupBox("Numerical ODE Solver")
        grid_s = QGridLayout(g_sol)
        self.chk_boost = QCheckBox("Use Boost.Odeint Stepper (unchecked = lightweight fixed RK)")
        self.chk_boost.setChecked(False)
        self.cmb_method = QComboBox()
        self.cmb_method.addItems([
            "ClassicalRK4",
            "Euler",
            "ModifiedMidpoint",
            "CashKarp54",
            "DormandPrince5",
            "Fehlberg78",
            "BulirschStoer",
        ])
        grid_s.addWidget(self.chk_boost, 0, 0, 1, 2)
        grid_s.addWidget(QLabel("Integration Method:"), 1, 0)
        grid_s.addWidget(self.cmb_method, 1, 1)
        vbox.addWidget(g_sol)

        vbox.addStretch(1)
        return w

    def _build_arena_vis_tab(self) -> QWidget:
        w = QWidget()
        vbox = QVBoxLayout(w)

        g_ar = QGroupBox("Arena Bounding-Box Display")
        grid_a = QGridLayout(g_ar)
        self.chk_arena_aabb = QCheckBox("Show Arena AABB Bounding Boxes")
        self.chk_arena_aabb.setChecked(False)
        grid_a.addWidget(self.chk_arena_aabb, 0, 0, 1, 2)
        vbox.addWidget(g_ar)

        g_vis = QGroupBox("RViz2 Visualization Options")
        grid_v = QGridLayout(g_vis)
        self.chk_payload_tree = QCheckBox("Draw Payload-Space RRT Tree (Orange)")
        self.chk_payload_tree.setChecked(True)
        self.chk_trolley_vel = QCheckBox("Draw Trolley Velocity Vector")
        self.chk_trolley_vel.setChecked(True)
        self.chk_ball_vel = QCheckBox("Draw Payload Velocity Vector")
        self.chk_ball_vel.setChecked(True)
        self.chk_trolley_hist = QCheckBox("Draw Trolley Trail")
        self.chk_trolley_hist.setChecked(True)
        self.chk_ball_hist = QCheckBox("Draw Payload Trail")
        self.chk_ball_hist.setChecked(True)

        self.sp_trolley_size = self._make_dspin(0.20, 0.05, 1.0, 0.02, 2)
        self.sp_rope_thick = self._make_dspin(0.02, 0.005, 0.2, 0.005, 3)
        self.sp_arrow_thick = self._make_dspin(0.04, 0.005, 0.2, 0.005, 3)
        self.sp_hist_limit = self._make_ispin(600, 10, 5000, 50)

        grid_v.addWidget(self.chk_payload_tree, 0, 0, 1, 2)
        grid_v.addWidget(self.chk_trolley_vel, 1, 0)
        grid_v.addWidget(self.chk_ball_vel, 1, 1)
        grid_v.addWidget(self.chk_trolley_hist, 2, 0)
        grid_v.addWidget(self.chk_ball_hist, 2, 1)
        grid_v.addWidget(QLabel("Trolley Size [m]:"), 3, 0)
        grid_v.addWidget(self.sp_trolley_size, 3, 1)
        grid_v.addWidget(QLabel("Cable Thickness [m]:"), 4, 0)
        grid_v.addWidget(self.sp_rope_thick, 4, 1)
        grid_v.addWidget(QLabel("Arrow Thickness [m]:"), 5, 0)
        grid_v.addWidget(self.sp_arrow_thick, 5, 1)
        grid_v.addWidget(QLabel("Trail Point Limit:"), 6, 0)
        grid_v.addWidget(self.sp_hist_limit, 6, 1)
        vbox.addWidget(g_vis)

        vbox.addStretch(1)
        return w

    def _on_arena_preset_changed(self, arena_name: str):
        if arena_name not in ARENA_PRESETS:
            return
        sx, sy, gx, gy, soft_corr, w_corr = ARENA_PRESETS[arena_name]
        self.sp_start_x.setValue(sx)
        self.sp_start_y.setValue(sy)
        self.sp_goal_x.setValue(gx)
        self.sp_goal_y.setValue(gy)
        self.chk_soft_corridor.setChecked(soft_corr)
        self.sp_corr_width.setValue(w_corr)

    def _on_method_preset_changed(self, method_name: str):
        if method_name not in METHOD_PRESETS:
            return
        cfg = METHOD_PRESETS[method_name]
        self.sp_min_dur.setValue(cfg["min_dur"])
        self.sp_max_dur.setValue(cfg["max_dur"])
        self.chk_smoothing.setChecked(cfg["smooth"])
        self.sp_force_bias.setValue(cfg["fbias"])
        self.sp_w_pos.setValue(cfg["w_pos"])
        self.sp_w_vel.setValue(cfg["w_vel"])
        self.sp_goal_bias.setValue(cfg["gbias"])
        self.sp_min_step.setValue(cfg["smin"])
        self.sp_max_step.setValue(cfg["smax"])
        self.chk_randsub.setChecked(cfg["randsub"])
        self.sp_w_u.setValue(cfg["w_u"])
        self.chk_sst.setChecked(cfg["sst"])
        self.sp_k_ctrl.setValue(cfg["k_ctrl"])
        self.chk_cell.setChecked(cfg["cell"])
        self.chk_pd.setChecked(cfg["pd"])
        self.chk_braking.setChecked(cfg["brake"])
        self.sp_max_iter.setValue(cfg["max_iter"])

    def _collect_payload(self, command: str) -> str:
        lines = [
            f"command={command}",
            f"rrt.start_x={self.sp_start_x.value()}",
            f"rrt.start_y={self.sp_start_y.value()}",
            f"rrt.goal_x={self.sp_goal_x.value()}",
            f"rrt.goal_y={self.sp_goal_y.value()}",
            f"rrt.x_min={self.sp_x_min.value()}",
            f"rrt.x_max={self.sp_x_max.value()}",
            f"rrt.y_min={self.sp_y_min.value()}",
            f"rrt.y_max={self.sp_y_max.value()}",
            f"rrt.w_pos={self.sp_w_pos.value()}",
            f"rrt.w_vel={self.sp_w_vel.value()}",
            f"rrt.force_history_nns_weight={self.sp_w_u.value()}",
            f"rrt.goal_bias={self.sp_goal_bias.value()}",
            f"rrt.force_direction_bias={self.sp_force_bias.value()}",
            f"rrt.max_force={self.sp_max_force.value()}",
            f"rrt.min_duration={self.sp_min_dur.value()}",
            f"rrt.max_duration={self.sp_max_dur.value()}",
            f"rrt.enable_force_smoothing={1 if self.chk_smoothing.isChecked() else 0}",
            f"rrt.min_search_step={self.sp_min_step.value()}",
            f"rrt.max_search_step={self.sp_max_step.value()}",
            f"rrt.use_equal_budget_random_subset={1 if self.chk_randsub.isChecked() else 0}",
            f"rrt.use_sst_planner={1 if self.chk_sst.isChecked() else 0}",
            f"rrt.sst_delta_bn={self.sp_sst_bn.value()}",
            f"rrt.sst_delta_s={self.sp_sst_s.value()}",
            f"rrt.num_control_samples={self.sp_k_ctrl.value()}",
            f"rrt.use_cell_partition_nns={1 if self.chk_cell.isChecked() else 0}",
            f"rrt.use_pd_local_steering={1 if self.chk_pd.isChecked() else 0}",
            f"rrt.enable_near_goal_braking={1 if self.chk_braking.isChecked() else 0}",
            f"rrt.near_goal_radius={self.sp_brake_rad.value()}",
            f"rrt.near_goal_zero_force_prob={self.sp_brake_prob.value()}",
            f"rrt.enable_soft_corridor={1 if self.chk_soft_corridor.isChecked() else 0}",
            f"rrt.corridor_half_width={self.sp_corr_width.value()}",
            f"rrt.goal_pos_tolerance={self.sp_goal_tol.value()}",
            f"rrt.goal_vel_tolerance={self.sp_goal_vel_tol.value()}",
            f"rrt.goal_swing_tolerance={self.sp_goal_sw_tol.value()}",
            f"rrt.goal_omega_tolerance={self.sp_goal_om_tol.value()}",
            f"rrt.require_goal_velocity_check={1 if self.chk_req_vel.isChecked() else 0}",
            f"rrt.require_goal_swing_check={1 if self.chk_req_sw.isChecked() else 0}",
            f"rrt.max_velocity={self.sp_max_vel.value()}",
            f"rrt.max_swing_angle={self.sp_max_swing.value()}",
            f"rrt.integration_dt={self.sp_dt.value()}",
            f"rrt.batch_iterations={self.sp_batch.value()}",
            f"rrt.max_iterations={self.sp_max_iter.value()}",
            f"rrt.max_nodes={self.sp_max_nodes.value()}",
            f"rrt.random_seed={self.sp_seed.value()}",
            f"rrt.check_rope_collision={1 if self.chk_rope_col.isChecked() else 0}",
            f"rrt.loop_playback={1 if self.chk_loop.isChecked() else 0}",
            f"rrt.draw_payload_tree={1 if self.chk_payload_tree.isChecked() else 0}",
            f"model.m_x={self.sp_mx.value()}",
            f"model.m_y={self.sp_my.value()}",
            f"model.m={self.sp_m.value()}",
            f"model.ell={self.sp_ell.value()}",
            f"model.g={self.sp_g.value()}",
            f"model.D_x={self.sp_dx.value()}",
            f"model.D_y={self.sp_dy.value()}",
            f"ball_radius={self.sp_ball_rad.value()}",
            f"solver.use_boost={1 if self.chk_boost.isChecked() else 0}",
            f"solver.integration_method={self.cmb_method.currentText()}",
            f"arena_file={self.cmb_arena.currentText()}",
            f"arena.show_aabb={1 if self.chk_arena_aabb.isChecked() else 0}",
            f"visualizer.trolley_size={self.sp_trolley_size.value()}",
            f"visualizer.rope_thickness={self.sp_rope_thick.value()}",
            f"visualizer.arrow_thickness={self.sp_arrow_thick.value()}",
            f"visualizer.history_count_limit={self.sp_hist_limit.value()}",
            f"visualizer.draw_trolley_velocity={1 if self.chk_trolley_vel.isChecked() else 0}",
            f"visualizer.draw_ball_velocity={1 if self.chk_ball_vel.isChecked() else 0}",
            f"visualizer.draw_trolley_history={1 if self.chk_trolley_hist.isChecked() else 0}",
            f"visualizer.draw_ball_history={1 if self.chk_ball_hist.isChecked() else 0}",
        ]
        return "\n".join(lines)

    def send_action(self, command: str):
        if command in ("toggle_pause", "step_once", "restart_playback", "clear_history"):
            self.ros_node.send_command(f"command={command}")
            return
        payload = self._collect_payload(command)
        self.ros_node.send_command(payload)

    def on_apply_and_reset(self):
        self.send_action("apply_and_reset")

    def on_new_seed_and_reset(self):
        new_seed = random.randint(1, 999999)
        self.sp_seed.setValue(new_seed)
        self.send_action("apply_and_reset")

    def _on_timer(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        st = self.ros_node.latest_status
        if not st:
            return

        mode = st.get("mode", "EXPLORING")
        if mode == "EXPLORING":
            self.lbl_mode.setText("EXPLORING...")
            self.lbl_mode.setStyleSheet("color: #2980b9; font-weight: bold;")
        elif mode == "GOAL_REACHED":
            self.lbl_mode.setText("GOAL REACHED — REPLAYING TRAJECTORY")
            self.lbl_mode.setStyleSheet("color: #27ae60; font-weight: bold;")
        elif mode == "PAUSED":
            self.lbl_mode.setText("PAUSED")
            self.lbl_mode.setStyleSheet("color: #d35400; font-weight: bold;")
        else:
            self.lbl_mode.setText("BUDGET REACHED — REPLAYING CLOSEST PATH")
            self.lbl_mode.setStyleSheet("color: #c0392b; font-weight: bold;")

        self.lbl_iter.setText(
            f"Iterations: {st.get('iterations', '0')} / {st.get('max_iterations', '0')}"
        )
        self.lbl_nodes.setText(
            f"Active Nodes: {st.get('nodes', '0')} / {st.get('max_nodes', '0')}"
        )
        self.lbl_coll.setText(f"Rejected Rollouts: {st.get('collisions', '0')}")
        self.lbl_dist.setText(f"Best Goal Dist: {st.get('best_dist', '—')} m")
        self.lbl_play.setText(
            f"Playback Step: {st.get('playback_idx', '0')} / {st.get('playback_len', '0')}"
        )
        self.lbl_state.setText(
            f"Trolley (x, y): ({st.get('x', '0')}, {st.get('y', '0')}) m | "
            f"Swing (α, β): ({st.get('alpha_deg', '0')}°, {st.get('beta_deg', '0')}°)"
        )
        self.lbl_force.setText(
            f"Force (Fx, Fy): ({st.get('fx', '0')}, {st.get('fy', '0')}) N"
        )


def main():
    rclpy.init()
    node = RRTControlROSNode()
    app = QApplication(sys.argv)
    win = RRTControlWindow(node)
    win.show()
    exit_code = app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

