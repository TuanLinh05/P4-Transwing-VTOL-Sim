#!/usr/bin/env python3
"""Focused tests for the local P4 Transwing control station."""

from __future__ import annotations

import importlib.util
import collections
import json
import os
import sys
import tempfile
import threading
import time
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from pymavlink import mavutil


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "p4_transwing_control_station_server", HERE / "server.py"
)
assert SPEC and SPEC.loader
SERVER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SERVER
SPEC.loader.exec_module(SERVER)


class MissionTestCase(unittest.TestCase):
    def manager(self):
        manager = object.__new__(SERVER.MavlinkManager)
        manager.snapshot = lambda: {
            "gps": {"lat": 10.772100, "lon": 106.657900}
        }
        return manager

    def test_normalises_vtol_mission_and_home(self) -> None:
        manager = self.manager()
        items = manager._normalise_mission(
            [
                {
                    "command": "VTOL_TAKEOFF",
                    "lat": 10.772100,
                    "lon": 106.657900,
                    "alt": 20,
                },
                {"command": "TRANSITION_FW"},
                {
                    "command": "WAYPOINT",
                    "lat": 10.772649,
                    "lon": 106.659050,
                    "alt": 50,
                    "acceptance": 25,
                },
                {"command": "TRANSITION_VTOL"},
                {
                    "command": "VTOL_LAND",
                    "lat": 10.772100,
                    "lon": 106.657900,
                    "alt": 0,
                },
            ]
        )

        self.assertEqual(len(items), 6)
        self.assertEqual(items[0]["commandName"], "HOME")
        self.assertEqual(items[0]["frame"], mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
        self.assertEqual(
            items[2]["params"][0],
            float(mavutil.mavlink.MAV_VTOL_STATE_FW),
        )
        self.assertEqual(
            items[4]["params"][0],
            float(mavutil.mavlink.MAV_VTOL_STATE_MC),
        )
        self.assertEqual(
            items[3]["frame"],
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )

    def test_rejects_invalid_coordinates_and_empty_mission(self) -> None:
        manager = self.manager()
        with self.assertRaisesRegex(ValueError, "At least one"):
            manager._normalise_mission([])
        with self.assertRaisesRegex(ValueError, "Invalid coordinates"):
            manager._normalise_mission(
                [{"command": "WAYPOINT", "lat": 120, "lon": 106, "alt": 30}]
            )

    def test_writes_qgc_waypoint_file(self) -> None:
        manager = self.manager()
        items = manager._normalise_mission(
            [
                {
                    "command": "WAYPOINT",
                    "lat": 10.772649,
                    "lon": 106.659050,
                    "alt": 40,
                }
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            manager.mission_output = Path(directory) / "mission.waypoints"
            manager._write_waypoint_file(items)
            text = manager.mission_output.read_text(encoding="utf-8")
        self.assertTrue(text.startswith("QGC WPL 110\n"))
        self.assertEqual(len(text.splitlines()), 3)


class PackagingTestCase(unittest.TestCase):
    def test_wsl_path_converts_to_windows_path(self) -> None:
        self.assertEqual(
            SERVER.ProcessManager._windows_path_from_wsl(
                "/mnt/e/Mission Planner/MissionPlanner.exe"
            ),
            r"E:\Mission Planner\MissionPlanner.exe",
        )

    def test_required_dashboard_controls_exist(self) -> None:
        html = (HERE / "static/index.html").read_text(encoding="utf-8")
        for element_id in (
            "start-button",
            "stop-button",
            "arm-button",
            "disarm-button",
            "takeoff-button",
            "transition-fw-button",
            "flight-action-status",
            "throttle-slider",
            "map-control",       # tab-based UI: formerly "map"
            "map-mission",       # tab-based UI: mission map
            "waypoint-body",
            "upload-mission",
            "start-mission",
            "latitude-value",
            "status-messages",
            "system-log",
        ):
            self.assertIn(f'id="{element_id}"', html)


    def test_p4_quadplane_mapping_and_safety_override(self) -> None:
        parameters = (
            HERE.parent / "ardupilot/p4_transwing_sitl.param"
        ).read_text(encoding="utf-8")
        for expected in (
            "Q_FRAME_CLASS 1",
            "Q_FRAME_TYPE 1",
            "Q_TILT_MASK 15",
            "Q_TILT_TYPE 0",
            "SERVO5_FUNCTION 41",
            "SERVO6_FUNCTION 4",
            "SERVO7_FUNCTION 79",
            "SERVO8_FUNCTION 80",
            "MIXING_GAIN 1.0",
            "ARSPD_USE 1",
        ):
            self.assertIn(expected, parameters)
        self.assertIn("ARMING_SKIPCHK 16", parameters)
        self.assertIn("MAV_GCS_SYSID 255", parameters)
        self.assertIn("Q_M_THST_HOVER 0.60", parameters)
        self.assertIn("Q_PILOT_SPD_UP 0.7", parameters)
        self.assertIn("Q_LOIT_ANG_MAX 6", parameters)
        self.assertIn("Q_P_ANGLE_MAX 6", parameters)
        self.assertIn("Q_A_ANGLE_MAX 15", parameters)
        self.assertIn("Q_LOIT_ACC_MAX_M 1.0", parameters)
        self.assertIn("Q_LOIT_BRK_ACC_M 0.3", parameters)
        self.assertIn("Q_LOIT_BRK_JRK_M 1.0", parameters)
        self.assertIn("Q_P_JERK_NE 1.0", parameters)
        self.assertIn("Q_A_ANG_PIT_P 2.5", parameters)
        self.assertIn("Q_A_RAT_PIT_I 0.12", parameters)

    def test_fixed_wing_transition_safety_gate(self) -> None:
        safe = {
            "connected": True,
            "armed": True,
            "mode": "QLOITER",
            "position": {"relativeAlt": 30.0},
            "attitude": {"roll": 2.0, "pitch": -3.0},
        }
        self.assertIsNone(
            SERVER.fixed_wing_transition_precondition(safe)
        )
        self.assertIn(
            "remain level and low-rate",
            SERVER.fixed_wing_transition_precondition(safe, stable_duration_s=1.49),
        )
        self.assertIsNone(
            SERVER.fixed_wing_transition_precondition(safe, stable_duration_s=1.5)
        )
        unstable_rate = json.loads(json.dumps(safe))
        unstable_rate["attitude"]["pitchSpeed"] = 12.1
        self.assertFalse(SERVER.hover_attitude_is_stable(unstable_rate))
        low = json.loads(json.dumps(safe))
        low["position"]["relativeAlt"] = 24.9
        self.assertIn(
            "at least",
            SERVER.fixed_wing_transition_precondition(low),
        )
        tilted = json.loads(json.dumps(safe))
        tilted["attitude"]["pitch"] = 10.1
        self.assertIn(
            "level",
            SERVER.fixed_wing_transition_precondition(tilted),
        )
        wrong_mode = json.loads(json.dumps(safe))
        wrong_mode["mode"] = "QHOVER"
        self.assertIn(
            "QLOITER",
            SERVER.fixed_wing_transition_precondition(wrong_mode),
        )

    def test_external_hover_safety_envelope_holds_neutral_until_calm(self) -> None:
        manager = object.__new__(SERVER.MavlinkManager)
        manager._lock = threading.RLock()
        manager._external_rc = SERVER.ExternalRcInput()
        manager._hover_stable_since = None
        manager._external_hover_recovery_active = False
        manager._external_hover_recovery_stable_since = None
        manager._external_hover_axes = [1500, 1500]
        manager._external_hover_axes_updated_at = time.monotonic() - 1.0
        manager.logs = types.SimpleNamespace(add=lambda *args, **kwargs: None)
        calm_hover = {
            "armed": True,
            "mode": "QLOITER",
            "attitude": {
                "roll": 0.0,
                "pitch": 0.0,
                "rollSpeed": 0.0,
                "pitchSpeed": 0.0,
            },
        }
        shaped = manager._shape_external_hover_channels(
            [2000, 1000, 1500, 1500], calm_hover
        )
        self.assertLessEqual(shaped[0], 1500 + SERVER.EXTERNAL_HOVER_AXIS_LIMIT_PWM)
        self.assertGreaterEqual(shaped[1], 1500 - SERVER.EXTERNAL_HOVER_AXIS_LIMIT_PWM)

        unsafe_hover = json.loads(json.dumps(calm_hover))
        unsafe_hover["attitude"]["pitch"] = 11.1
        self.assertEqual(
            manager._shape_external_hover_channels(
                [1500, 1500, 1300, 1500], unsafe_hover
            ),
            [1500, 1500, 1500, 1500],
        )
        self.assertTrue(manager._external_hover_recovery_is_active())

        with manager._lock:
            manager._external_hover_recovery_stable_since = (
                time.monotonic() - SERVER.EXTERNAL_HOVER_RECOVERY_HOLD_S
            )
        manager._shape_external_hover_channels([1500, 1500, 1500, 1500], calm_hover)
        self.assertFalse(manager._external_hover_recovery_is_active())


    def test_dashboard_routes_fbwb_through_bounded_transition(self) -> None:
        javascript = (HERE / "static/app.js").read_text(encoding="utf-8")
        server = (HERE / "server.py").read_text(encoding="utf-8")
        self.assertIn('endpoint: "/api/vehicle/transition-fw"', javascript)
        self.assertIn('button.dataset.mode === "FBWB"', javascript)
        self.assertIn('path == "/api/vehicle/transition-fw"', server)
        self.assertIn("FIXED_WING_CRUISE_RC3 = 1470", server)
        self.assertIn("throttle=FIXED_WING_CRUISE_RC3", server)

    def test_dashboard_contains_external_mkr_controls(self) -> None:
        html = (HERE / "static/index.html").read_text(encoding="utf-8")
        javascript = (HERE / "static/app.js").read_text(encoding="utf-8")
        server = (HERE / "server.py").read_text(encoding="utf-8")
        for element_id in (
            "external-rc-indicator",
            "external-rc-message",
            "external-rc-enable-button",
            "external-rc-disable-button",
            "external-ch1",
            "external-ch6",
            "external-ch10",
        ):
            self.assertIn(f'id="{element_id}"', html)
        self.assertIn("updateExternalRcState", javascript)
        self.assertNotIn("updateMkrTelemetry", javascript)
        self.assertIn('"/api/controller/external/enable"', javascript)
        self.assertIn('path == "/api/controller/external/frame"', server)
        self.assertIn("HOVER RECOVERY", server)
        self.assertIn("EXTERNAL_HOVER_AXIS_LIMIT_PWM", server)
        self.assertIn("hoverRecoveryActive", server)
        self.assertIn("hoverRecovery", javascript)
        self.assertIn('self._set_mode_and_wait("QLOITER", 4.0)', server)
        self.assertIn('Cannot ARM while QRTL is airborne', server)

    def test_external_radio_is_rc_only(self) -> None:
        server = (HERE / "server.py").read_text(encoding="utf-8")
        bridge = (HERE.parent / "tools/bridge_mkr_usb.py").read_text(encoding="utf-8")
        firmware = (HERE.parent.parent / "MCU/Controller/Controller.ino").read_text(
            encoding="utf-8"
        )
        self.assertIn("rc_forward_worker", bridge)
        self.assertIn("P4RC USB ingest rate", bridge)
        self.assertIn("P4RC,1", firmware)
        self.assertNotIn("P4TEL", bridge)
        self.assertNotIn("P4TEL", firmware)
        self.assertNotIn("SERCOM3", firmware)
        self.assertNotIn("/api/controller/external/telemetry", server)

    def test_ground_takeoff_calibration_has_four_complete_propellers(self) -> None:
        root = ET.parse(
            HERE.parent / "gazebo/models/p4_transwing_ardupilot/model.sdf"
        ).getroot()
        base_inertial_pose = root.find(
            ".//link[@name='base_link']/inertial/pose"
        )
        self.assertIsNotNone(base_inertial_pose)
        self.assertAlmostEqual(
            float(base_inertial_pose.text.split()[0]), 0.10, places=9
        )
        rotor_sections = [
            plugin
            for plugin in root.findall(
                ".//plugin[@name='gz::sim::systems::LiftDrag']"
            )
            if (plugin.findtext("link_name") or "").startswith("rotor_")
        ]
        self.assertEqual(len(rotor_sections), 8)
        self.assertTrue(
            all(
                abs(float(plugin.findtext("area")) - 0.00621) < 1e-9
                for plugin in rotor_sections
            )
        )

    def test_optional_remote_scenery_world_is_visual_only(self) -> None:
        world_path = HERE.parent / "gazebo/worlds/p4_ardupilot_scenery.sdf"
        scenery_path = HERE.parent / "gazebo/models/p4_scenery/model.sdf"
        world_root = ET.parse(world_path).getroot()
        world = world_root.find("world")
        self.assertIsNotNone(world)
        assert world is not None
        self.assertEqual(world.attrib.get("name"), "p4_ardupilot_scenery")
        uris = [item.findtext("uri") for item in world.findall("include")]
        self.assertIn("model://p4_transwing_ardupilot", uris)
        self.assertIn("model://p4_scenery", uris)

        scenery_root = ET.parse(scenery_path).getroot()
        model = scenery_root.find("model")
        self.assertIsNotNone(model)
        assert model is not None
        self.assertEqual(model.findtext("static"), "true")
        self.assertGreaterEqual(len(model.findall(".//visual")), 300)
        self.assertEqual(model.findall(".//collision"), [])
        self.assertTrue(
            (
                HERE.parent
                / "gazebo/models/p4_scenery/materials/textures/urban_aerial_diffuse.png"
            ).is_file()
        )

        server = (HERE / "server.py").read_text(encoding="utf-8")
        runner = (
            HERE.parent / "gazebo/run_p4_ardupilot_scenery_server_wsl.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("P4_TRANSWING_GAZEBO_WORLD", server)
        self.assertIn("p4_ardupilot_scenery.sdf", runner)

    def test_optional_xl_scenery_world_is_large_and_visual_only(self) -> None:
        world_path = HERE.parent / "gazebo/worlds/p4_ardupilot_scenery_xl.sdf"
        scenery_path = HERE.parent / "gazebo/models/p4_scenery_xl/model.sdf"
        world_root = ET.parse(world_path).getroot()
        world = world_root.find("world")
        self.assertIsNotNone(world)
        assert world is not None
        self.assertEqual(world.attrib.get("name"), "p4_ardupilot_scenery_xl")
        uris = [item.findtext("uri") for item in world.findall("include")]
        self.assertIn("model://p4_transwing_ardupilot", uris)
        self.assertIn("model://p4_scenery_xl", uris)

        scenery_root = ET.parse(scenery_path).getroot()
        model = scenery_root.find("model")
        self.assertIsNotNone(model)
        assert model is not None
        self.assertEqual(model.attrib.get("name"), "p4_scenery_xl")
        self.assertEqual(model.findtext("static"), "true")
        self.assertGreaterEqual(len(model.findall(".//visual")), 700)
        self.assertEqual(model.findall(".//collision"), [])
        model_text = scenery_path.read_text(encoding="utf-8")
        self.assertNotIn("urban_xl_aerial_diffuse.png", model_text)
        self.assertIn("tower_", model_text)
        self.assertIn("house_", model_text)

        server = (HERE / "server.py").read_text(encoding="utf-8")
        launcher = (
            HERE.parent / "control_station/start_control_station.ps1"
        ).read_text(encoding="utf-8")
        runner = (
            HERE.parent / "gazebo/run_p4_ardupilot_scenery_xl_server_wsl.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("scenery_xl", server)
        self.assertIn("run_p4_ardupilot_scenery_xl_server_wsl.sh", server)
        self.assertIn("SceneryXL", launcher)
        self.assertIn("p4_ardupilot_scenery_xl.sdf", runner)

    def test_vtail_has_transition_pitch_authority(self) -> None:
        root = ET.parse(
            HERE.parent / "gazebo/models/p4_transwing_ardupilot/model.sdf"
        ).getroot()
        vtail_areas = [
            float(plugin.findtext("area", "0"))
            for plugin in root.findall(
                ".//plugin[@name='gz::sim::systems::LiftDrag']"
            )
            if plugin.findtext("link_name", "")
            in {"left_ruddervator_link", "right_ruddervator_link"}
        ]
        self.assertEqual(sorted(vtail_areas), [0.2448, 0.2448])

    def test_main_wing_cp_is_relative_to_inertial_cog(self) -> None:
        root = ET.parse(
            HERE.parent / "gazebo/models/p4_transwing_ardupilot/model.sdf"
        ).getroot()
        wing_cp = {
            plugin.findtext("link_name"): [
                float(value) for value in plugin.findtext("cp", "").split()
            ]
            for plugin in root.findall(
                ".//plugin[@name='gz::sim::systems::LiftDrag']"
            )
            if plugin.findtext("link_name", "")
            in {"left_wing_link", "right_wing_link"}
        }
        self.assertEqual(
            wing_cp,
            {
                "left_wing_link": [-0.13, 0.0, 0.03],
                "right_wing_link": [-0.13, 0.0, 0.03],
            },
        )

    def test_lifting_surfaces_have_positive_zero_geometry_lift(self) -> None:
        root = ET.parse(
            HERE.parent / "gazebo/models/p4_transwing_ardupilot/model.sdf"
        ).getroot()
        lifting_links = {
            "left_wing_link",
            "right_wing_link",
            "left_inboard_aileron_link",
            "left_outboard_aileron_link",
            "right_inboard_aileron_link",
            "right_outboard_aileron_link",
        }
        alpha_offsets = {
            plugin.findtext("link_name"): float(plugin.findtext("a0", "0"))
            for plugin in root.findall(
                ".//plugin[@name='gz::sim::systems::LiftDrag']"
            )
            if plugin.findtext("link_name", "") in lifting_links
        }
        self.assertEqual(set(alpha_offsets), lifting_links)
        self.assertTrue(
            all(abs(value - 0.14) < 1e-9 for value in alpha_offsets.values())
        )

    def test_takeoff_throttle_tapers_before_target(self) -> None:
        self.assertEqual(SERVER.TAKEOFF_ALTITUDE_TOLERANCE_M, 0.4)
        self.assertEqual(SERVER.takeoff_altitude_tolerance_m(5.0), 0.4)
        self.assertEqual(SERVER.takeoff_altitude_tolerance_m(20.0), 0.8)
        self.assertEqual(SERVER.takeoff_throttle_for_remaining(5.0), 1900)
        self.assertEqual(SERVER.takeoff_throttle_for_remaining(2.0), 1700)
        self.assertEqual(SERVER.takeoff_throttle_for_remaining(0.8), 1600)

    def test_p4_reference_matches_transition_surrogate(self) -> None:
        reference = json.loads(
            (HERE.parent / "config/p4_reference.json").read_text(
                encoding="utf-8"
            )
        )
        parameters = (
            HERE.parent / "ardupilot/p4_transwing_sitl.param"
        ).read_text(encoding="utf-8")
        model = ET.parse(
            HERE.parent / "gazebo/models/p4_transwing_ardupilot/model.sdf"
        ).getroot()

        self.assertEqual(
            reference["presentation_transition_50_percent"]["airspeed_mps"],
            [10, 20],
        )
        self.assertAlmostEqual(
            reference["published_x_p4"]["uafm_rev_d"]["stall_speed_mps"],
            26.751111,
            places=6,
        )
        self.assertEqual(
            reference["published_x_p4"]["propulsion"][
                "fixed_wing_active_propulsors"
            ],
            2,
        )
        self.assertAlmostEqual(
            reference["simulation_baseline"]["transition_50_percent_rad"],
            -1.029716127,
            places=9,
        )
        self.assertIn("Q_TILT_MAX 20", parameters)
        self.assertIn("Q_TILT_RATE_DN 20", parameters)
        self.assertIn("AIRSPEED_MIN 27", parameters)
        self.assertIn("AIRSPEED_CRUISE 34", parameters)
        self.assertIn("Q_ASSIST_SPEED -1", parameters)
        self.assertIn("Q_OPTIONS 4096", parameters)
        self.assertIn("Q_ASSIST_ANGLE 0", parameters)
        self.assertIn("SCALING_SPEED 34", parameters)
        self.assertIn("PTCH_RATE_P 0.08", parameters)
        self.assertIn("PTCH_RATE_I 0.50", parameters)
        self.assertIn("PTCH_RATE_FF 0.50", parameters)
        self.assertIn("ROLL_LIMIT_DEG 25", parameters)
        self.assertIn("NAVL1_PERIOD 35", parameters)
        self.assertIn("NAVL1_DAMPING 0.90", parameters)
        self.assertIn("RLL2SRV_RMAX 45", parameters)
        self.assertIn("RLL2SRV_TCONST 0.80", parameters)
        self.assertIn("RLL_RATE_FF 0.20", parameters)
        self.assertIn("RLL_RATE_P 0.04", parameters)
        self.assertIn("RLL_RATE_I 0.10", parameters)
        self.assertIn("RLL_RATE_D 0.003", parameters)
        self.assertIn("RLL_RATE_SMAX 90", parameters)
        self.assertIn("TECS_SPDWEIGHT 0.50", parameters)
        self.assertIn("TECS_TIME_CONST 8.0", parameters)
        self.assertIn("TECS_THR_DAMP 0.80", parameters)
        self.assertIn("TECS_PITCH_MIN -5", parameters)
        self.assertIn("ARSPD_TYPE 100", parameters)
        self.assertIn("Q_TRANSITION_MS 12000", parameters)
        self.assertIn("Q_TRANS_FAIL 30", parameters)
        wing_stall_angles = {
            float(value.text)
            for value in model.findall(
                ".//plugin[@name='gz::sim::systems::LiftDrag']/alpha_stall"
            )
            if value.text
        }
        self.assertIn(0.28, wing_stall_angles)

    def test_uncalibrated_experiment_tab_is_locked(self) -> None:
        html = (HERE / "static/index.html").read_text(encoding="utf-8")
        experiment_button = html.split('data-target="tab-experiments"', 1)[1]
        self.assertIn("disabled", experiment_button.split("</button>", 1)[0])

    def test_ardupilot_sdf_actuator_mapping(self) -> None:
        model_file = (
            HERE.parent
            / "gazebo/models/p4_transwing_ardupilot/model.sdf"
        )
        root = ET.parse(model_file).getroot()
        plugin = root.find(".//plugin[@name='ArduPilotPlugin']")
        self.assertIsNotNone(plugin)
        controls = plugin.findall("control")

        motors = {
            int(control.attrib["channel"]): (
                control.findtext("jointName"),
                float(control.findtext("multiplier")),
            )
            for control in controls
            if int(control.attrib["channel"]) < 4
        }
        self.assertEqual(
            motors,
            {
                0: ("rotor_right_inner_joint", 733.04),
                1: ("rotor_left_outer_joint", 733.04),
                2: ("rotor_left_inner_joint", -733.04),
                3: ("rotor_right_outer_joint", -733.04),
            },
        )

        fold = [control for control in controls if control.attrib["channel"] == "4"]
        self.assertEqual(
            {control.findtext("cmd_topic") for control in fold},
            {"/p4/fold_left_cmd", "/p4/fold_right_cmd"},
        )
        self.assertTrue(
            all(float(control.findtext("multiplier")) < 0 for control in fold)
        )

        ailerons = [
            control for control in controls if control.attrib["channel"] == "5"
        ]
        self.assertEqual(len(ailerons), 4)
        signs = {
            control.findtext("jointName"): (
                1 if float(control.findtext("multiplier")) > 0 else -1
            )
            for control in ailerons
        }
        self.assertEqual(
            signs,
            {
                "left_inboard_aileron_joint": -1,
                "left_outboard_aileron_joint": -1,
                "right_inboard_aileron_joint": 1,
                "right_outboard_aileron_joint": 1,
            },
        )

        vtail = {
            int(control.attrib["channel"]): control.findtext("jointName")
            for control in controls
            if control.attrib["channel"] in {"6", "7"}
        }
        self.assertEqual(
            vtail,
            {
                6: "left_ruddervator_joint",
                7: "right_ruddervator_joint",
            },
        )


class ClimbToAltitudeGuardTestCase(unittest.TestCase):
    def manager(self, mode: str) -> "SERVER.MavlinkManager":
        manager = object.__new__(SERVER.MavlinkManager)
        manager._lock = threading.RLock()
        manager.snapshot = lambda: {"mode": mode}
        operation_state = {
            "name": None,
            "status": "idle",
            "message": "",
            "targetAltitude": None,
            "startedAt": None,
        }
        manager._operation_state = operation_state
        manager._operation = SERVER.OperationController(
            operation_state,
            manager._set_operation,
        )
        return manager

    def test_rejects_when_not_in_fbwb(self) -> None:
        manager = self.manager("QLOITER")
        with self.assertRaisesRegex(RuntimeError, "FBWB"):
            manager.climb_to_altitude(50.0)

    def test_rejects_when_operation_already_running(self) -> None:
        manager = self.manager("FBWB")
        manager._operation.start("takeoff", 20.0)
        with self.assertRaisesRegex(RuntimeError, "already running"):
            manager.climb_to_altitude(50.0)


class ExternalRcProtocolTestCase(unittest.TestCase):
    @staticmethod
    def line(*, flags: int = 1, channels: list[int] | None = None) -> str:
        values = channels or [
            1500, 1500, 1000, 1500, 1000,
            1500, 1500, 1500, 1000, 2000,
        ]
        payload = "P4RC,1,42,123456," + ",".join(str(value) for value in values) + f",{flags}"
        return f"{payload},{SERVER.crc16_ccitt_false(payload.encode('ascii')):04X}"

    def test_parses_valid_mkr_frame(self) -> None:
        frame = SERVER.parse_external_rc_line(self.line())
        self.assertEqual(frame.sequence, 42)
        self.assertEqual(frame.channels[2], 1000)
        self.assertEqual(frame.channels[9], 2000)
        self.assertTrue(frame.input_valid)

    def test_rejects_bad_crc_and_out_of_range_channel(self) -> None:
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            SERVER.parse_external_rc_line(self.line()[:-1] + "0")
        with self.assertRaisesRegex(ValueError, "outside the permitted range"):
            SERVER.parse_external_rc_line(
                self.line(channels=[
                    1500, 1500, 2200, 1500, 1000,
                    1500, 1500, 1500, 1000, 2000,
                ])
            )

    def test_rejects_legacy_eight_channel_frame(self) -> None:
        legacy_values = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]
        payload = "P4RC,1,42,123456," + ",".join(str(value) for value in legacy_values) + ",1"
        legacy_line = f"{payload},{SERVER.crc16_ccitt_false(payload.encode('ascii')):04X}"
        with self.assertRaisesRegex(ValueError, "Unsupported external RC protocol"):
            SERVER.parse_external_rc_line(legacy_line)

    def test_timeout_flag_cannot_be_fresh_control_input(self) -> None:
        input_state = SERVER.ExternalRcInput()
        input_state.ingest(SERVER.parse_external_rc_line(self.line(flags=2)))
        snapshot = input_state.snapshot()
        self.assertTrue(snapshot["received"])
        self.assertFalse(snapshot["fresh"])
        self.assertIn("timeout", snapshot["lastError"].lower())

    def test_valid_frame_clears_display_only_failsafe_latch_without_reenabling_control(self) -> None:
        input_state = SERVER.ExternalRcInput()
        valid = SERVER.parse_external_rc_line(self.line())
        input_state.ingest(valid)
        input_state.set_enabled(True)
        input_state.trip_failsafe("test radio timeout")
        self.assertTrue(input_state.snapshot()["failsafeLatched"])

        input_state.ingest(valid)
        recovered = input_state.snapshot()
        self.assertTrue(recovered["fresh"])
        self.assertFalse(recovered["failsafeLatched"])
        self.assertFalse(recovered["enabled"])
        self.assertIn("recovered", recovered["lastError"].lower())

    def test_switch_mapping_is_debounced_and_prioritises_qrtl(self) -> None:
        input_state = SERVER.ExternalRcInput()
        base = [1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        input_state.ingest(SERVER.parse_external_rc_line(self.line(channels=base)))
        input_state.set_enabled(True)

        arm_high = list(base)
        arm_high[6] = 2000  # CH7 / SWA
        input_state.ingest(SERVER.parse_external_rc_line(self.line(channels=arm_high)))
        time.sleep(SERVER.EXTERNAL_RC_SWITCH_DEBOUNCE_S + 0.02)
        input_state.ingest(SERVER.parse_external_rc_line(self.line(channels=arm_high)))
        self.assertEqual(input_state.pop_event(), "arm_high")
        self.assertEqual(input_state.snapshot()["armSwitch"], "HIGH")

        qrtl_high = list(arm_high)
        qrtl_high[7] = 2000  # CH8 / SWB
        qrtl_high[8] = 1500  # CH9 / SWC middle, transition request
        input_state.ingest(SERVER.parse_external_rc_line(self.line(channels=qrtl_high)))
        time.sleep(SERVER.EXTERNAL_RC_SWITCH_DEBOUNCE_S + 0.02)
        input_state.ingest(SERVER.parse_external_rc_line(self.line(channels=qrtl_high)))
        self.assertEqual(input_state.pop_event(), "qrtl")
        self.assertEqual(input_state.snapshot()["qrtlSwitch"], "HIGH")

    def test_explicit_external_takeover_can_replace_safe_user_hold(self) -> None:
        manager = object.__new__(SERVER.MavlinkManager)
        manager._lock = threading.RLock()
        manager._rc_owner = "USER"
        manager._rc_generation = 9
        lease = manager.acquire_rc("EXTERNAL")
        self.assertEqual(lease.owner, "EXTERNAL")
        self.assertEqual(manager._rc_owner, "EXTERNAL")


class FlightLogTestCase(unittest.TestCase):
    def station(self, log_dir: Path) -> "SERVER.ControlStation":
        station = object.__new__(SERVER.ControlStation)
        station.flight_log_dir = log_dir
        return station

    def test_lists_only_matching_bin_files_newest_first(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log_dir = Path(directory)
            (log_dir / "00000001.BIN").write_bytes(b"a")
            (log_dir / "00000002.BIN").write_bytes(b"bb")
            (log_dir / "notes.txt").write_bytes(b"ignore me")
            os.utime(log_dir / "00000001.BIN", (1_000, 1_000))
            os.utime(log_dir / "00000002.BIN", (2_000, 2_000))
            entries = self.station(log_dir).flight_logs()
        self.assertEqual([entry["name"] for entry in entries], ["00000002.BIN", "00000001.BIN"])
        self.assertEqual(entries[1]["sizeBytes"], 1)

    def test_rejects_names_that_do_not_match_pattern(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log_dir = Path(directory)
            (log_dir / "00000001.BIN").write_bytes(b"x")
            station = self.station(log_dir)
            for bad_name in (
                "../secret.txt",
                "00000001.bin.txt",
                "..%2f..%2fetc%2fpasswd",
                "logs/00000001.BIN",
            ):
                self.assertIsNone(station.flight_log_path(bad_name))
            self.assertIsNotNone(station.flight_log_path("00000001.BIN"))

    def test_missing_directory_returns_empty_list(self) -> None:
        station = self.station(Path("/nonexistent/p4-transwing-flight-logs"))
        self.assertEqual(station.flight_logs(), [])


class FlightSafetyTestCase(unittest.TestCase):
    def test_stale_heartbeat_clears_displayed_flight_authority(self) -> None:
        manager = object.__new__(SERVER.MavlinkManager)
        manager._lock = threading.RLock()
        manager._last_heartbeat = time.monotonic() - 10
        manager._rc_override_active = True
        manager._rc_owner = "USER"
        manager._status_messages = collections.deque()
        manager._state = {
            "connected": True,
            "armed": True,
            "mode": "QLOITER",
            "gps": {
                "fixType": 3,
                "satellites": 10,
                "hdop": 1.0,
            },
            "battery": {"voltage": 12.0},
        }
        operation_state = {
            "name": None,
            "status": "idle",
            "message": "",
            "targetAltitude": None,
            "startedAt": None,
        }
        manager._operation_state = operation_state
        manager._operation = SERVER.OperationController(
            operation_state,
            manager._set_operation,
        )

        state = manager.snapshot()

        self.assertFalse(state["connected"])
        self.assertFalse(state["armed"])
        self.assertEqual(state["mode"], "UNKNOWN")
        self.assertFalse(state["rcOverrideActive"])

    def test_missing_sitl_battery_telemetry_does_not_block_prearm(self) -> None:
        manager = object.__new__(SERVER.MavlinkManager)
        manager._lock = threading.RLock()
        manager._last_heartbeat = time.monotonic()
        manager._rc_override_active = False
        manager._rc_owner = None
        manager._status_messages = collections.deque()
        manager._state = {
            "connected": True,
            "armed": False,
            "mode": "QLOITER",
            "homeSet": True,
            "gps": {
                "fixType": 6,
                "satellites": 10,
                "hdop": 1.2,
            },
            "battery": {
                "voltage": None,
                "current": None,
                "remaining": None,
            },
            "mission": {"total": 3},
        }
        operation_state = {
            "name": None,
            "status": "idle",
            "message": "",
            "targetAltitude": None,
            "startedAt": None,
        }
        manager._operation_state = operation_state
        manager._operation = SERVER.OperationController(
            operation_state,
            manager._set_operation,
        )

        state = manager.snapshot()

        self.assertTrue(state["prearm"]["ready"])
        battery_check = next(
            item
            for item in state["prearm"]["checks"]
            if item["id"] == "battery"
        )
        self.assertEqual(battery_check["status"], "warn")

    def test_prearm_waits_for_ahrs_home(self) -> None:
        manager = object.__new__(SERVER.MavlinkManager)
        manager._lock = threading.RLock()
        manager._last_heartbeat = time.monotonic()
        manager._rc_override_active = False
        manager._rc_owner = None
        manager._status_messages = collections.deque()
        manager._state = {
            "connected": True,
            "armed": False,
            "mode": "QLOITER",
            "homeSet": False,
            "gps": {
                "fixType": 6,
                "satellites": 10,
                "hdop": 1.2,
            },
            "battery": {
                "voltage": None,
                "current": None,
                "remaining": None,
            },
            "mission": {"total": 3},
        }
        operation_state = {
            "name": None,
            "status": "idle",
            "message": "",
            "targetAltitude": None,
            "startedAt": None,
        }
        manager._operation_state = operation_state
        manager._operation = SERVER.OperationController(
            operation_state,
            manager._set_operation,
        )

        state = manager.snapshot()

        self.assertFalse(state["prearm"]["ready"])
        self.assertIn("Home", state["prearm"]["message"])



class CsrfGuardTestCase(unittest.TestCase):
    """Tests for _reject_untrusted_post (FIX-03)."""

    def _handler(self, headers: dict) -> "SERVER.ApiHandler":
        handler = object.__new__(SERVER.ApiHandler)
        security = SERVER.RequestSecurity("127.0.0.1", None)
        import email.message
        msg = email.message.Message()
        defaults = {
            "Host": "127.0.0.1:8765",
            "Origin": "http://127.0.0.1:8765",
            "Content-Type": "application/json",
            "X-P4-Transwing-CSRF": security.csrf_token,
        }
        defaults.update(headers)
        for key, value in defaults.items():
            if value is not None:
                msg[key] = value
        handler.headers = msg
        handler.server = types.SimpleNamespace(
            app=types.SimpleNamespace(security=security)
        )
        return handler

    def test_rejects_foreign_origin(self) -> None:
        handler = self._handler({"Origin": "http://evil.example"})
        with self.assertRaises(PermissionError):
            handler._reject_untrusted_post()

    def test_rejects_non_json_content_type(self) -> None:
        handler = self._handler({"Content-Type": "text/plain"})
        with self.assertRaises(PermissionError):
            handler._reject_untrusted_post()

    def test_allows_localhost_json(self) -> None:
        handler = self._handler({})
        handler._reject_untrusted_post()  # must not raise

    def test_rejects_no_origin_with_json(self) -> None:
        handler = self._handler({"Origin": None})
        with self.assertRaises(PermissionError):
            handler._reject_untrusted_post()


if __name__ == "__main__":
    unittest.main()
