#!/usr/bin/env python3

from dynamixel_sdk import *

DEVICENAME = '/dev/dynamixel'
BAUDRATE = 2000000
PROTOCOL_VERSION = 1.0

ADDR_LED = 25
ADDR_PRESENT_POSITION = 36

LEN_LED = 1
LEN_PRESENT_POSITION = 2

DXL_ID1 = 11
DXL_ID2 = 13

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

if portHandler.openPort():
    print("Puerto abierto correctamente.")
else:
    print("Error al abrir el puerto.")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Baudrate configurado a 2M bps.")
else:
    print("Error al cambiar el baudrate.")
    quit()

print("-" * 40)
print("Iniciando prueba de Bulk Read...")
print("-" * 40)

dxl_addparam_result = groupBulkRead.addParam(DXL_ID1, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
if not dxl_addparam_result:
    print(f"[ID:{DXL_ID1}] Error al añadir parámetro de Posición Actual al BulkRead.")
    quit()

dxl_addparam_result = groupBulkRead.addParam(DXL_ID2, ADDR_LED, LEN_LED)
if not dxl_addparam_result:
    print(f"[ID:{DXL_ID2}] Error al añadir parámetro de LED al BulkRead.")
    quit()

dxl_comm_result = groupBulkRead.txRxPacket()
if dxl_comm_result != COMM_SUCCESS:
    print("Resultado TxRx: %s" % packetHandler.getTxRxResult(dxl_comm_result))

dxl_getdata_result_1 = groupBulkRead.isAvailable(DXL_ID1, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
if not dxl_getdata_result_1:
    print(f"[ID:{DXL_ID1}] Falló el Bulk Read (Datos no disponibles). El motor podría no soportarlo.")
else:
    dxl1_present_position = groupBulkRead.getData(DXL_ID1, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    print(f"[ID:{DXL_ID1}] Posición Actual leída exitosamente: {dxl1_present_position}")

# Revisar Motor 2
dxl_getdata_result_2 = groupBulkRead.isAvailable(DXL_ID2, ADDR_LED, LEN_LED)
if not dxl_getdata_result_2:
    print(f"[ID:{DXL_ID2}] Falló el Bulk Read (Datos no disponibles). El motor podría no soportarlo.")
else:
    dxl2_led_value = groupBulkRead.getData(DXL_ID2, ADDR_LED, LEN_LED)
    print(f"[ID:{DXL_ID2}] Valor del LED leído exitosamente: {dxl2_led_value}")

# Cerrar puerto
groupBulkRead.clearParam()
portHandler.closePort()
print("-" * 40)
