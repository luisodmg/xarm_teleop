import asyncio
import websockets
import json
from xarm.wrapper import XArmAPI

async def send_pose(websocket):
    ip_maestro = "192.168.1.167"
    print(f"Conectando al brazo Maestro en {ip_maestro}...")
    
    arm = XArmAPI(ip_maestro)
    arm.connect()
    
    print("¡Conectado! Transmitiendo posición a 50Hz...")

    while True:
        code, pose = arm.get_position()
        
        if code == 0:
            # Convertimos mm a metros para que coincida con tu IK
            data = {
                "x": pose[0] / 1000.0,
                "y": pose[1] / 1000.0,
                "z": pose[2] / 1000.0
            }
            try:
                await websocket.send(json.dumps(data))
            except websockets.exceptions.ConnectionClosed:
                print("El esclavo se ha desconectado.")
                break
        
        await asyncio.sleep(0.02) # 50Hz

async def async_main():
    print("Iniciando Servidor WebSocket del Maestro en ws://127.0.0.1:8765")
    async with websockets.serve(send_pose, "127.0.0.1", 8765):
        await asyncio.Future()

def main(args=None):
    asyncio.run(async_main())

if __name__ == "__main__":
    main()
