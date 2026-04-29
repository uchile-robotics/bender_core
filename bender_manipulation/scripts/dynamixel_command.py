#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import math
from dynamixel_sdk import *

class DynamixelCommander():
    def __init__(self, config_path="config/params.yaml"):

        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)

        # --- Configuración Básica ---
        self.PROTOCOL_VERSION = 1.0
        self.BAUDRATE = self.config["baudrate"]
        self.DEVICE_NAME = self.config["device_name"]
        self.MOTORS = self.config["motors"]

        self.DXL_IDS = [motor["id"] for motor in self.MOTORS]
        self.offsets = {motor["id"]: motor["offset"] for motor in self.MOTORS}

        # --- Direcciones de Memoria (Protocolo 1.0) ---
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_MOVING_SPEED = 32
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_PRESENT_POSITION = 36
        self.ADDR_PRESENT_SPEED = 38
        self.ADDR_PRESENT_LOAD = 40
        self.ADDR_CW_COMPLIANCE_SLOPE = 28
        self.ADDR_CCW_COMPLIANCE_SLOPE = 29

        # Longitud para SyncWrite: 2 bytes Pos + 2 bytes Vel = 4 bytes
        self.LEN_POS_VEL = 4

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.portHandler = PortHandler(self.DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            self.ADDR_GOAL_POSITION,
            self.LEN_POS_VEL
        )

        if not self.portHandler.openPort():
            raise RuntimeError("[ERROR] No se pudo abrir el puerto")

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            raise RuntimeError("[ERROR] No se pudo establecer la velocidad de baudios")

        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id,
                self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
            )

        self.set_compliance_slope(10)

        print(f"[INFO] DynamixelCommander listo. Motores: {self.DXL_IDS}")

    def set_compliance_slope(self, slope_level=64):
        slope = max(0, min(254, slope_level))
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id,
                self.ADDR_CW_COMPLIANCE_SLOPE, slope
            )
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id,
                self.ADDR_CCW_COMPLIANCE_SLOPE, slope
            )

    def decode_present_speed(self, vel_raw):
        """
        Convierte el raw de Present Speed a valor con signo.
        """
        if vel_raw > 1023:
            return -(vel_raw - 1024)
        return vel_raw

    def decode_present_load(self, load_raw):
        """
        Convierte Present Load (0..2047) a porcentaje con signo.
        Magnitud ~ 0..100 %
        Signo según dirección interna del servo.
        """
        if load_raw > 1023:
            signed = -(load_raw - 1024)
        else:
            signed = load_raw

        percent_signed = signed * 0.1
        percent_abs = abs(percent_signed)
        return percent_signed, percent_abs

    def get_joints_data(self):
        """
        Lee posición, velocidad y carga actual.
        positions: raw corregido por offset
        velocities: raw con signo decodificado
        loads_signed_percent: porcentaje con signo
        loads_abs_percent: magnitud en porcentaje
        """
        positions = []
        velocities = []
        loads_signed_percent = []
        loads_abs_percent = []

        for dxl_id in self.DXL_IDS:
            p_data, res_p, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION
            )
            v_data, res_v, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_PRESENT_SPEED
            )
            l_data, res_l, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_PRESENT_LOAD
            )

            if res_p != COMM_SUCCESS or res_v != COMM_SUCCESS or res_l != COMM_SUCCESS:
                positions.append(0)
                velocities.append(0)
                loads_signed_percent.append(0.0)
                loads_abs_percent.append(0.0)
                continue

            positions.append(p_data - self.offsets.get(dxl_id, 0))
            velocities.append(self.decode_present_speed(v_data))

            load_signed, load_abs = self.decode_present_load(l_data)
            loads_signed_percent.append(load_signed)
            loads_abs_percent.append(load_abs)

        return positions, velocities, loads_signed_percent, loads_abs_percent

    def raw_to_radians(self, joint_index, pos_raw):
        """
        Convierte posición raw corregida por offset -> radianes.
        """
        motor_conf = self.MOTORS[joint_index]
        motor_type = motor_conf.get("type", "RX-28")

        if "MX-106" in motor_type:
            resolution = 4095
            max_rad = 2 * math.pi
        else:
            resolution = 1023
            max_rad = math.radians(300)

        pos_rad = (pos_raw / resolution) * max_rad
        pos_rad = (pos_rad + math.pi) % (2 * math.pi) - math.pi
        return pos_rad

    def get_current_joint_angles_rad(self):
        """
        Devuelve todas las articulaciones en radianes usando la lectura actual.
        """
        positions, _, _, _ = self.get_joints_data()
        angles = []
        for i, pos_raw in enumerate(positions):
            angles.append(self.raw_to_radians(i, pos_raw))
        return angles

    def set_joints_with_velocity(self, target_positions, target_velocities):
        self.groupSyncWrite.clearParam()

        if len(target_positions) != len(self.DXL_IDS):
            return

        for i, dxl_id in enumerate(self.DXL_IDS):
            motor_conf = self.MOTORS[i]
            motor_type = motor_conf.get("type", "RX-28")

            if "MX-106" in motor_type:
                resolution = 4095
                max_rad = 2 * math.pi
                vel_unit = 0.114
            else:
                resolution = 1023
                max_rad = math.radians(300)
                vel_unit = 0.111

            pos_rad = target_positions[i]
            pos_raw = int((pos_rad / max_rad) * resolution) + self.offsets[dxl_id]
            pos_raw = max(0, min(resolution, pos_raw))

            if not target_velocities or i >= len(target_velocities):
                rad_s = 0.5
            else:
                rad_s = abs(target_velocities[i])

            rpm = rad_s * 9.5493

            # Nunca mandar 0 porque en protocolo 1.0 puede significar potencia máxima
            if rpm < 0.5:
                speed_raw = 20
            else:
                speed_raw = int(rpm / vel_unit)

            speed_raw = max(20, min(1023, speed_raw))

            data_package = [
                DXL_LOBYTE(DXL_LOWORD(pos_raw)),
                DXL_HIBYTE(DXL_LOWORD(pos_raw)),
                DXL_LOBYTE(DXL_LOWORD(speed_raw)),
                DXL_HIBYTE(DXL_LOWORD(speed_raw))
            ]

            self.groupSyncWrite.addParam(dxl_id, data_package)

        self.groupSyncWrite.txPacket()

    def hold_current_position(self, hold_speed=0.2):
        """
        Congela el robot en su pose actual.
        """
        current_angles = self.get_current_joint_angles_rad()
        self.set_joints_with_velocity(current_angles, [hold_speed] * len(current_angles))

    def shutdown(self):
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id,
                self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
            )
        self.portHandler.closePort()
