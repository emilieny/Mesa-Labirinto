import serial
import json
import time
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# ==================================================
# SERIAL CONFIG
# ==================================================
SERIAL_PORT = "COM8"        # AJUSTE PARA SUA PORTA
BAUDRATE = 115200

# ==================================================
# INFLUXDB CONFIG
# ==================================================
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "AXz61M0qU7HSXL5zTa1wdSW--F7M-su0yqq0tH7hiDvD-h3Djk8FJ-SpVHTmE3_XsEYmb3zQeaK1Uc1MuocCGg=="
INFLUX_ORG = "IFPBLAB"
INFLUX_BUCKET = "labirinto"

# ==================================================
# INIT SERIAL
# ==================================================
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUDRATE,
    timeout=1
)

# ==================================================
# INIT INFLUXDB
# ==================================================
client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG
)

write_api = client.write_api(write_options=SYNCHRONOUS)
# Buffer utilizado para garantir que apenas linhas completas (terminadas em '\n')
# sejam processadas, evitando erros causados por leituras parciais da serial
buffer = ""

print("📡 Sistema iniciado — aguardando dados do ESP32...")

# ==================================================
# MAIN LOOP
# ==================================================
while True:
    try:
        # Lê bytes disponíveis da serial
        data = ser.read(ser.in_waiting or 1).decode(errors="ignore")
        if not data:
            continue

        buffer += data

        # Processa linhas completas
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()

            if not line:
                continue

            # ============================
            # PARSE JSON
            # ============================
            try:
                # decodifica a linha como JSON
                payload = json.loads(line)
            except json.JSONDecodeError:
                # Ignora JSON incompleto ou inválido
                continue

            msg_type = payload.get("type")

            # ============================
            # MPU DATA
            # ============================
            if msg_type == "mpu":
                pitch = float(payload["pitch"])
                roll  = float(payload["roll"])

                print(f"MPU | Pitch: {pitch:.2f} | Roll: {roll:.2f}")

                point = (
                    Point("orientacao_mesa")
                    .field("pitch", pitch)
                    .field("roll", roll)
                )

            # ============================
            # EVENTO (VITÓRIA)
            # ============================
            elif msg_type == "event":
                status = payload.get("status", "UNKNOWN")

                print(f"🎉 EVENTO: {status}")

                point = (
                    Point("evento_labirinto")
                    .field("status", status)
                )

            # ============================
            # CALIBRAÇÃO DO JOYSTICK
            # ============================
            elif msg_type == "calib":
                axis  = payload.get("axis")
                limit = payload.get("limit")
                value = float(payload.get("value", 0))

                print(f"🔧 CALIBRAÇÃO | Eixo {axis} | {limit} = {value:.2f}")

                point = (
                    Point("calibracao_joystick")
                    .tag("axis", axis)
                    .tag("limit", limit)
                    .field("value", value)
                )

            else:
                continue

            #Envia o ponto para o InfluxDB, permitindo visualização e monitoramento em tempo real no Grafana
            write_api.write(
                bucket=INFLUX_BUCKET,
                org=INFLUX_ORG,
                record=point
            )

    except Exception as e:
        print("❌ Erro inesperado:", e)
        time.sleep(0.2)
