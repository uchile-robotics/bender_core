#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import yaml
import math
import os
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
        self.ADDR_CW_COMPLIANCE_SLOPE = 28
        self.ADDR_CCW_COMPLIANCE_SLOPE = 29
        
        # Longitud para SyncWrite: 2 bytes Pos + 2 bytes Vel = 4 bytes
        self.LEN_POS_VEL = 4 

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # --- Inicialización del Puerto ---
        self.portHandler = PortHandler(self.DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        # Inicializar GroupSyncWrite
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_POS_VEL)

        if not self.portHandler.openPort():
            print("[ERROR] No se pudo abrir el puerto")
            exit()

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("[ERROR] No se pudo establecer la velocidad de baudios")
            exit()

        # Habilitar torque y configurar parámetros iniciales
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        
        # --- AQUÍ FALTABA LLAMAR A LA FUNCIÓN ---
        # Activamos la suavidad (Compliance Slope) al iniciar
        self.set_compliance_slope(10) 
        # ----------------------------------------

        print(f"[INFO] DynamixelCommander listo. Motores: {self.DXL_IDS}")

    def set_compliance_slope(self, slope_level=64):
        """
        Ajusta la "elasticidad" del frenado.
        32 = Rígido (default). 64-96 = Suave (bueno para trayectorias).
        """
        slope = max(0, min(254, slope_level))
        
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_CW_COMPLIANCE_SLOPE, slope)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_CCW_COMPLIANCE_SLOPE, slope)

    def get_joints_data(self):
        """Lee posición y velocidad actual."""
        positions = []
        velocities = []
        for dxl_id in self.DXL_IDS:
            p_data, res_p, err_p = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            v_data, res_v, err_v = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_SPEED)
            
            if res_p != COMM_SUCCESS:
                positions.append(0) 
                velocities.append(0)
                continue

            # El offset se resta al leer para devolver radianes "lógicos"
            positions.append(p_data - self.offsets.get(dxl_id, 0))
            velocities.append(v_data)
        
        return positions, velocities

    def set_joints_with_velocity(self, target_positions, target_velocities):
        """
        Envía un paquete SyncWrite con Posición (Bytes 0,1) y Velocidad (Bytes 2,3).
        """
        self.groupSyncWrite.clearParam()

        if len(target_positions) != len(self.DXL_IDS):
            return

        for i, dxl_id in enumerate(self.DXL_IDS):
            motor_conf = self.MOTORS[i]
            motor_type = motor_conf.get("type", "RX-28")
            
            # --- Configuración por modelo ---
            if "MX-106" in motor_type:
                resolution = 4095
                max_rad = 2 * math.pi 
                vel_unit = 0.114 
            else: # RX-28, RX-64
                resolution = 1023
                max_rad = math.radians(300) 
                vel_unit = 0.111 

            # --- 1. Cálculo de Posición ---
            pos_rad = target_positions[i]
            # Fórmula: Calculamos ticks relativos + offset del home
            pos_raw = int((pos_rad / max_rad) * resolution) + self.offsets[dxl_id]
            
            # Clamp de seguridad para no enviar valores fuera de rango de hardware
            pos_raw = max(0, min(resolution, pos_raw)) 

            # --- 2. Cálculo de Velocidad ---
            if not target_velocities or i >= len(target_velocities):
                rad_s = 0.5 
            else:
                rad_s = abs(target_velocities[i])
            
            rpm = rad_s * 9.5493
            
            # SEGURIDAD CRÍTICA:
            # En Protocolo 1.0, velocidad 0 significa "Máxima Potencia".
            # Si MoveIt manda 0 (parada), forzamos una velocidad mínima (20) para que no salte.
            if rpm < 0.5:
                speed_raw = 20 
            else:
                speed_raw = int(rpm / vel_unit)

            # Límite de 1023 para el registro de velocidad
            speed_raw = max(20, min(1023, speed_raw))

            # --- 3. Empaquetado ---
            data_package = [
                DXL_LOBYTE(DXL_LOWORD(pos_raw)),
                DXL_HIBYTE(DXL_LOWORD(pos_raw)),
                DXL_LOBYTE(DXL_LOWORD(speed_raw)),
                DXL_HIBYTE(DXL_LOWORD(speed_raw))
            ]

            self.groupSyncWrite.addParam(dxl_id, data_package)

        # --- 4. Enviar ---
        self.groupSyncWrite.txPacket()

    def shutdown(self):
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.portHandler.closePort()