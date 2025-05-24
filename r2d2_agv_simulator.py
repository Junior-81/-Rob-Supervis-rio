import pybullet as p
import pybullet_data
import time
import random
import json
import paho.mqtt.client as mqtt

# === Parâmetros de velocidade ===
MAX_VEL = 8.0
TURN_VEL = 3.0
LEFT, RIGHT = [2, 4], [3, 5]

# === Configuração MQTT ===
BROKER = "localhost"
TOPIC_CMD = "robo/comando"
TOPIC_POSICAO = "robo/posicao"
TOPIC_ALERTA = "robo/alerta"

comando_atual = "parar"
ultimo_comando = None
alertado = False

# === Funções MQTT ===
def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Conectado com código: {rc}")
    client.subscribe(TOPIC_CMD)

def on_message(client, userdata, msg):
    global comando_atual
    try:
        payload = msg.payload.decode().strip()
        print(f"[MQTT] Comando recebido: '{payload}'")

        try:
            data = json.loads(payload)
            novo_comando = data.get("comando", payload).lower().strip()
        except json.JSONDecodeError:
            novo_comando = payload.lower().strip()

        if novo_comando == "re":
            novo_comando = "ré"

        if novo_comando in ["frente", "ré", "esquerda", "direita", "parar"]:
            comando_atual = novo_comando
            client.publish(TOPIC_ALERTA, f"Executando: {comando_atual}")
        else:
            client.publish(TOPIC_ALERTA, f"Comando inválido: {novo_comando}")
    except Exception as e:
        print(f"[MQTT] Erro: {e}")

# === Inicializa MQTT ===
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.loop_start()
client.publish(TOPIC_ALERTA, "Robô pronto para comandos")

# === Inicializa PyBullet ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.resetDebugVisualizerCamera(8, 60, -35, [3, 3, 0])
p.loadURDF("plane.urdf")

# === Robô e obstáculos ===
husky = p.loadURDF("husky/husky.urdf", [1, 1, 0.1])
obstaculos = [
    p.loadURDF("cube_small.urdf", [random.uniform(2, 8), random.uniform(2, 8), 0.15], globalScaling=10.0)
    for _ in range(5)
]

# === Aplica comando ao robô ===
def aplica_comando(cmd):
    if cmd == "frente":
        lv = rv = MAX_VEL
    elif cmd == "ré":
        lv = rv = -MAX_VEL
    elif cmd == "esquerda":
        lv, rv = -TURN_VEL, TURN_VEL
    elif cmd == "direita":
        lv, rv = TURN_VEL, -TURN_VEL
    else:
        lv = rv = 0

    for j in LEFT:
        p.setJointMotorControl2(husky, j, p.VELOCITY_CONTROL, targetVelocity=lv, force=400)
    for j in RIGHT:
        p.setJointMotorControl2(husky, j, p.VELOCITY_CONTROL, targetVelocity=rv, force=400)

# === Verifica se há obstáculo por perto ===
def detecta_colisao(robot, obstacles):
    for obs in obstacles:
        if p.getClosestPoints(robot, obs, distance=1.0):
            return True
    return False

# === Loop de simulação ===
contador = 0
try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1 / 60)
        contador += 1

        # Alerta de obstáculo, mas sem bloquear nada
        if detecta_colisao(husky, obstaculos):
            if not alertado:
                print("[AVISO] Obstáculo detectado!")
                client.publish(TOPIC_ALERTA, "AVISO: Obstáculo detectado!")
                alertado = True
        else:
            if alertado:
                print("[INFO] Caminho livre.")
                client.publish(TOPIC_ALERTA, "Caminho livre")
                alertado = False

        # Aplica comando se mudou
        if comando_atual != ultimo_comando:
            aplica_comando(comando_atual)
            ultimo_comando = comando_atual

        # Publica posição a cada 90 ciclos (~1.5s)
        if contador % 90 == 0:
            pos, _ = p.getBasePositionAndOrientation(husky)
            x, y = round(pos[0], 2), round(pos[1], 2)
            posicao_json = json.dumps({"x": x, "y": y})
            client.publish(TOPIC_POSICAO, posicao_json)

except KeyboardInterrupt:
    print("[SISTEMA] Encerrando...")

finally:
    client.publish(TOPIC_ALERTA, "Robô desconectado")
    client.loop_stop()
    client.disconnect()
    p.disconnect()
    print("[SISTEMA] Desconectado do MQTT e PyBullet.")