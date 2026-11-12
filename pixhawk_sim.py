import time
import math
import numpy as np
from pymavlink import mavutil


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt, is_angle=False):
        if dt <= 0.0:
            return 0.0
        if is_angle:
            error = (error + 180.0) % 360.0 - 180.0

        self.integral += error * dt
        self.integral = float(np.clip(self.integral, -10.0, 10.0))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return float(np.clip(output, self.min_out, self.max_out))


class PixhawkROVSimulator:
    MODE_MANUAL = 0
    MODE_DEPTH_HOLD = 1
    MODE_HEADING_HOLD = 2

    def __init__(self, telemetry_port=14550, command_port=14551):
        print(f"[PIXHAWK SIM] Initializing MAVLink on UDP Out:{telemetry_port}, In:{command_port}...")
        
        #udp send and recv
        self.mav_out = mavutil.mavlink_connection(f'udpout:127.0.0.1:{telemetry_port}', source_system=1)
        self.mav_in = mavutil.mavlink_connection(f'udpin:127.0.0.1:{command_port}')

        #pixhawk state variales
        self.armed = False
        self.mode = self.MODE_MANUAL
        self.target_depth = 0.0     
        self.target_heading = 0.0    

        #pose
        self.lat = 19.9975           
        self.lon = 85.8245           
        self.x = 0.0                 
        self.y = 0.0                 
        self.depth = 0.0             
        self.roll = 0.0              
        self.pitch = 0.0             
        self.yaw = 0.0               

        self.vel = np.zeros(4)
        self.cmd_target = np.zeros(4)
        self.actual_thrust = np.zeros(4)

        # Physics Parameters
        self.tau_thruster = 0.20     # 200ms thruster spool-up delay
        self.mass = 12.0             # kg
        self.inertia_z = 0.8         # kg*m^2
        self.max_force = 45.0        # N
        self.max_torque = 12.0       # N*m
        self.lin_drag = np.array([12.0, 18.0, 15.0, 3.5])
        self.quad_drag = np.array([25.0, 35.0, 30.0, 5.0])
        self.buoyancy_trim = -1.5    # Slight positive buoyancy (-1.5N upward)

        # Battery and timer
        self.battery_voltage = 16.8
        self.last_manual_input_time = time.time()

        # Onboard PID Controllers
        self.depth_pid = PIDController(kp=0.8, ki=0.15, kd=0.4, output_limits=(-1.0, 1.0))
        self.heading_pid = PIDController(kp=0.035, ki=0.002, kd=0.015, output_limits=(-1.0, 1.0))

    def receive_gcs_commands(self):
        while True:
            msg = self.mav_in.recv_match(blocking=False)
            if not msg:
                break

            mtype = msg.get_type()

            if mtype == 'MANUAL_CONTROL':
                fwd = msg.x/ 1000
                lat = msg.y/ 1000
                vert = msg.z/ 1000
                yaw = msg.r/ 1000
                self.cmd_target = np.array([fwd, lat, vert, yaw])

                if np.any(np.abs(self.cmd_target) > 0.01):
                    self.last_manual_input_time = time.time()

            elif mtype == 'COMMAND_LONG':
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.armed = bool(msg.param1 == 1.0)
                    state_str = "ARMED" if self.armed else "DISARMED"
                    print(f"[PIXHAWK SIM] Vehicle State Changed -> {state_str}")
                    if not self.armed:
                        self.cmd_target = np.zeros(4)
                        self.actual_thrust = np.zeros(4)
                        self.depth_pid.reset()
                        self.heading_pid.reset()

                elif msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    new_mode = int(msg.param1)
                    if new_mode != self.mode:
                        self.mode = new_mode
                        self.depth_pid.reset()
                        self.heading_pid.reset()
                        mode_names = {0: "MANUAL", 1: "DEPTH_HOLD", 2: "HEADING_HOLD"}
                        print(f"[PIXHAWK SIM] Flight Mode Changed -> {mode_names.get(self.mode)}")

                elif msg.command == mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT:
                    self.target_depth = max(0.0, float(msg.param1))
                    print(f"[PIXHAWK SIM] New Target Depth -> {self.target_depth:.2f} m")

                elif msg.command == mavutil.mavlink.MAV_CMD_CONDITION_YAW:
                    self.target_heading = float(msg.param1) % 360.0
                    print(f"[PIXHAWK SIM] New Target Heading -> {self.target_heading:.1f} deg")

    def update_physics(self, dt):
        now = time.time()

        #1 sec safety timer
        if now - self.last_manual_input_time > 1.0:
            self.cmd_target = np.zeros(4)

        desired_cmd = np.copy(self.cmd_target)

        #depth hodl
        if self.mode == self.MODE_DEPTH_HOLD:
            depth_error = self.target_depth - self.depth
            desired_cmd[2] = self.depth_pid.compute(depth_error, dt)

        elif self.mode == self.MODE_HEADING_HOLD:
            heading_error = self.target_heading - self.yaw
            desired_cmd[3] = self.heading_pid.compute(heading_error, dt, is_angle=True)

        #thurst=0 when disarmed
        if not self.armed:
            desired_cmd = np.zeros(4)
            self.actual_thrust = np.zeros(4)
        else:
            alpha = dt / (self.tau_thruster + dt)
            self.actual_thrust += alpha * (desired_cmd - self.actual_thrust)

        # Forces & Torques
        forces = np.array([
            self.actual_thrust[0] * self.max_force,
            self.actual_thrust[1] * self.max_force,
            self.actual_thrust[2] * self.max_force + self.buoyancy_trim,
            self.actual_thrust[3] * self.max_torque
        ])

        # Hydrodynamic Drag (Linear + Quadratic)
        drag = (self.lin_drag * self.vel) + (self.quad_drag * np.abs(self.vel) * self.vel)
        net_forces = forces - drag

        accel = np.array([
            net_forces[0] / self.mass,
            net_forces[1] / self.mass,
            net_forces[2] / self.mass,
            net_forces[3] / self.inertia_z
        ])

        self.vel += accel * dt

        # World-Frame Kinematics Integration
        yaw_rad = math.radians(self.yaw)
        vn = self.vel[0] * math.cos(yaw_rad) - self.vel[1] * math.sin(yaw_rad)
        ve = self.vel[0] * math.sin(yaw_rad) + self.vel[1] * math.cos(yaw_rad)

        self.x += vn * dt
        self.y += ve * dt
        self.depth += self.vel[2] * dt
        if self.depth < 0.0:
            self.depth = 0.0
            if self.vel[2] < 0.0:
                self.vel[2] = 0.0

        self.lat += (vn * dt) / 111111.0
        self.lon += (ve * dt) / (111111.0 * math.cos(math.radians(self.lat)))

        self.yaw = (self.yaw + math.degrees(self.vel[3] * dt)) % 360.0
        self.pitch = float(np.clip(-self.actual_thrust[0] * 8.0, -15.0, 15.0))
        self.roll = float(np.clip(self.actual_thrust[1] * 8.0, -15.0, 15.0))

        #bettery simulator
        thrust_load = np.sum(np.abs(self.actual_thrust))
        drain = (0.0005 + 0.002 * thrust_load) * dt if self.armed else 0.0001 * dt
        self.battery_voltage = max(12.0, self.battery_voltage - drain)

    def broadcast_telemetry(self):
        # heartbeat
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        self.mav_out.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_SUBMARINE,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            self.mode,
            mavutil.mavlink.MAV_STATE_ACTIVE if self.armed else mavutil.mavlink.MAV_STATE_STANDBY
        )

        #attitude
        self.mav_out.mav.attitude_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            math.radians(self.roll),
            math.radians(self.pitch),
            math.radians(self.yaw),
            0.0, 0.0, self.vel[3]
        )

        #depth and thrust
        thrust_pct = int(np.clip(np.max(np.abs(self.actual_thrust)) * 100.0, 0, 100))
        self.mav_out.mav.vfr_hud_send(
            float(np.linalg.norm(self.vel[:2])),
            float(np.linalg.norm(self.vel[:2])),
            int(self.yaw),
            thrust_pct,
            -float(self.depth),
            -float(self.vel[2])
        )

        # sys_status
        self.mav_out.mav.sys_status_send(
            0, 0, 0, 500,
            int(self.battery_voltage * 1000),
            -1, -1, 0, 0, 0, 0, 0, 0
        )

        #gPS
        self.mav_out.mav.global_position_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            int(self.lat * 1e7),
            int(self.lon * 1e7),
            int(-self.depth * 1000),
            int(-self.depth * 1000),
            0, 0, 0,
            int(self.yaw * 100)
        )
        #thruster pwms
        if not self.armed:
            pwms = [1500] * 8
        else:
            f, l, v, y = self.actual_thrust
            m1 = int(np.clip(1500 + (f - l + y) * 400, 1100, 1900))
            m2 = int(np.clip(1500 + (f + l - y) * 400, 1100, 1900))
            m3 = int(np.clip(1500 + (-f - l - y) * 400, 1100, 1900))
            m4 = int(np.clip(1500 + (-f + l + y) * 400, 1100, 1900))
            m_vert = int(np.clip(1500 + v * 400, 1100, 1900))
            pwms = [m1, m2, m3, m4, m_vert, m_vert, m_vert, m_vert]

        self.mav_out.mav.servo_output_raw_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            0,
            pwms[0], pwms[1], pwms[2], pwms[3],
            pwms[4], pwms[5], pwms[6], pwms[7]
        )

    def run(self):
        last_time = time.time()
        last_telem = time.time()
        last_print = time.time()
        try:
            while True:
                now = time.time()
                dt = now - last_time
                last_time = now

                self.receive_gcs_commands()
                self.update_physics(dt)

                if now - last_telem >= 0.05:  # 20 Hz Telemetry
                    self.broadcast_telemetry()
                    last_telem = now

                if now - last_print >= 1.0:  # 1 Hz Console Status
                    mode_str = {0: "MANUAL", 1: "DEPTH_HOLD", 2: "HEADING_HOLD"}[self.mode]
                    print(
                        f"Status: {'ARMED' if self.armed else 'DISARMED':<8} | "
                        f"Mode: {mode_str:<12} | "
                        f"Depth: {self.depth:5.2f}m | "
                        f"Yaw: {self.yaw:5.1f}° | "
                        f"Thrust: {np.round(self.actual_thrust, 2)}"
                    )
                    last_print = now

                time.sleep(0.01)  # 100 Hz step
        except KeyboardInterrupt:
            print("\npixhawk shutdown")


if __name__ == '__main__':
    sim = PixhawkROVSimulator()
    sim.run()