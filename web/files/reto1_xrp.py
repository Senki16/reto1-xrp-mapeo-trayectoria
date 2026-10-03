# =============================================================================
#  Reto 1 - Mapeo de trayectoria con robot XRP
#  Actuadores y Sensores - Universidad EAFIT 2026-2
#
#  MARCHA: seguidor de linea con control de velocidad por rueda (PI con
#  encoders, 100 Hz). ESTIMACION: odometria (encoders + giroscopio) + LiDAR 360
#  (scan matching PL-ICP) fusionados con un filtro de Kalman extendido (EKF).
#  Al llegar a la meta se detiene y guarda la trayectoria x,y en
#  datos_<IDENTIFICADOR>.csv (se sobreescribe en cada recorrido) y datos.txt.
#
#  Placa: SparkFun XRP Controller (RP2350) con XRPLib.
#  Subir este archivo a la raiz de la XRP y ejecutarlo desde XRPCode.
# =============================================================================
import math
import time
import gc

from machine import UART, Pin
from XRPLib.board import Board
from XRPLib.imu import IMU
from XRPLib.encoded_motor import EncodedMotor
from XRPLib.reflectance import Reflectance

try:
    from array import array
except ImportError:
    array = None

VERSION_CODIGO = "v14 (2-oct, marcha por velocidad de rueda + LiDAR diferido)"

# =============================================================================
#  1. CONFIGURACION  (todo lo que se calibra esta aqui)
# =============================================================================
# -- Robot ---------------------------------------------------------------------
RADIO_RUEDA   = 3.0      # cm   rueda XRP de 6.0 cm de diametro -> calibrar (p6)
TRACK_WIDTH   = 15.5     # cm   separacion entre ruedas (XRPLib usa 15.5) -> calibrar (p6)
TICKS_POR_REV = 585      # cuentas por vuelta de RUEDA: 12 CPR x 48.75 de reduccion
                         # (Encoder.resolution en XRPLib). OJO: la plantilla dice 360.
DT            = 0.10     # s    periodo de la ESTIMACION (10 Hz). Las ruedas se controlan
                         #      aparte, a 100 Hz (LOOP_DT)
IDENTIFICADOR = "NicolasZapata"         # aparece en el nombre de los archivos de salida
LOG_FILE      = "datos.txt"             # entregable de la guia: x,y
LOG_CSV       = "datos_" + IDENTIFICADOR + ".csv"              # el que se entrega (x,y)
LOG_COMPLETO  = "datos_" + IDENTIFICADOR + "_completos.csv"    # extra para el informe
LOG_COMPLETO_CADA = 2                   # guardar fila completa cada N ciclos
MAX_FILAS     = 4000                    # proteccion de memoria (400 s a 10 Hz)


# -- Marcha (control de velocidad por rueda + seguidor) --------------------------
# Tomado del programa que ya recorre bien la pista con ESTE robot: cada rueda
# tiene su propio control PI de velocidad con el encoder, asi que no importa que
# un motor sea mas fuerte que el otro (reemplaza a MOTOR_L_TRIM).
# Unidades: cm/s y rad/s. Para ir mas rapido o mas lento: V_RECTA_CM_S y V_CURVA_CM_S.
LOOP_DT       = 0.010    # s   control de ruedas a 100 Hz
MAX_CONTROL_GAP_S = 0.120
RADIO_CONTROL = 2.9      # cm  radio usado por el control de velocidad (no por la odometria)
CIRC_CONTROL  = 2.0 * math.pi * RADIO_CONTROL
LINEA_INVERTIDA = False  # True si el sensor quedo montado al reves (izq <-> der), ver p2

# Reflectancia: blanco y negro medidos por sensor (p2_reflectancia.py)
WHITE_LEFT    = 0.5618
WHITE_RIGHT   = 0.5082
BLACK_LEFT    = 0.8725
BLACK_RIGHT   = 0.8620
CROSS_RAW_LEFT = 0.80    # los DOS sensores por encima de esto = linea transversal
CROSS_RAW_RIGHT = 0.80
SENSOR_SAMPLES = 3
SIGNAL_ON     = 0.25     # presencia de linea (con histeresis contra el ruido)
SIGNAL_OFF    = 0.12
ERROR_DEADBAND = 0.035

V_RECTA_CM_S  = 6.0      # velocidad en recta
V_CURVA_CM_S  = 3.5      # velocidad en curva
V_CRUCE_CM_S  = 5.0      # sobre la linea transversal
V_PERDIDA_CM_S = 3.5     # sin linea y sin saber de que lado quedo
V_RUEDA_RECUPERACION_CM_S = 8.0   # rueda que avanza al recuperar la linea (la otra, quieta)
V_RUEDA_MAX_CM_S = 18.0
ACEL_AVANCE_CM_S2 = 12.0
KP_LINEA      = 0.55     # rad/s por unidad de error
KD_LINEA      = 0.010
DERIVATIVE_FILTER_S = 0.040
LINE_ERROR_FILTER_S = 0.15
OMEGA_LINEA_MAX = 0.35   # rad/s, giro maximo que pide el seguidor
YAW_DAMPING   = 0.18     # amortiguamiento con el giroscopio
STEERING_SLEW_RAD_S2 = 0.65
CURVA_ENTER_ERROR = 0.14
CURVA_EXIT_ERROR = 0.08
MEMORIA_CENTRADO_S = 0.25
MIN_INNER_WHEEL_RATIO = 0.25   # en curvas normales las dos ruedas avanzan
IMU_FILTER_S  = 0.08
IMU_GYRO_SIGN = 1.0
ENCODER_L_SIGN = 1.0     # +1 si avanzar incrementa get_position()
ENCODER_R_SIGN = 1.0

# PI de velocidad por rueda. KS ~ friccion estatica, KV ~ esfuerzo por cm/s.
KS_L          = 0.32
KS_R          = 0.40
KV_L          = 0.022
KV_R          = 0.030
KP_RUEDA      = 0.025
KI_RUEDA      = 0.14
INTEGRAL_MAX  = 0.45
MAX_EFFORT    = 0.85
EFFORT_RISE_PER_S = 3.0
VEL_WINDOW_S  = 0.030
VEL_FILTER_S  = 0.020
ZERO_SPEED_CM_S = 0.12
REVERSAL_BRAKE_S = 0.04
REVERSAL_MAX_S = 0.20
ATASCO_VEL_CM_S = 0.30
ATASCO_BOOST_AFTER_S = 0.12
ATASCO_BOOST_MAX = 0.18
ATASCO_STOP_S = 1.50
ENCODER_MAX_CM_S = 150.0

# Salida y meta: los dos sensores sobre la transversal durante este tiempo
START_CONFIRM_S = 0.04
CROSS_CONFIRM_S = 0.06
CROSS_CLEAR_S = 0.25     # debe salir de la linea de salida antes de aceptar la meta
MIN_FIN_DIST_CM = 20.0   # y recorrer al menos esto
MIN_FIN_TIME_S = 3.0     # y andar al menos este tiempo

# -- IMU -----------------------------------------------------------------------
USAR_GIROSCOPIO = True   # True: dtheta del giroscopio (yaw); False: de los encoders
T_CALIBRACION_IMU = 2    # s con el robot quieto

# -- LiDAR -----------------------------------------------------------------------
USAR_LIDAR    = True
# LiDAR LDROBOT (paquetes 54 2C de 47 bytes, 230400 baudios) conectado directo.
# Al arrancar se prueban los dos puertos y se usa el que tenga datos validos:
#   Qwiic 0 : IO5 = RX del UART1 (recibe del LiDAR), IO4 = TX del UART1
#   Dist    : IO1 = RX del UART0, IO0 = TX del UART0
PUERTOS_LIDAR = ((1, 4, 5), (0, 0, 1))   # (uart, pin TX, pin RX)
LIDAR_UART_ID = 1
LIDAR_TX_PIN  = 4
LIDAR_RX_PIN  = 5
LIDAR_PROTOCOLO = "auto" # "auto" busca el puerto; "ld06" usa LIDAR_UART_ID/TX/RX tal cual
LIDAR_BAUD    = 0        # 0 = 230400
LIDAR_VERIFICAR_CRC = True
N_ANGULOS     = 360
LIDAR_OFFSET_DEG = 0     # indice del barrido que apunta al FRENTE del robot (p4)
LIDAR_SENTIDO = 1        # +1 si el indice crece antihorario, -1 si horario (p4)
LIDAR_X       = 0.0      # cm posicion del LiDAR respecto al centro del eje (adelante +)
LIDAR_Y       = 0.0      # cm (izquierda +)
D_MIN_MM      = 120      # descarta el propio robot / cables
D_MAX_MM      = 6000     # descarta lecturas muy lejanas (ruidosas)
LIDAR_TIMEOUT_MS = 1000  # sin tramas por mas de esto -> solo odometria

# El LiDAR trabaja en DIFERIDO: durante el recorrido la XRP solo guarda "fotos"
# del LiDAR (un barrido cada pocos cm) y NO calcula nada, asi las ruedas nunca
# esperan. Al llegar a la meta el robot se detiene, procesa los barridos (ICP +
# filtro de Kalman) y escribe el CSV.
BARRIDO_CADA_CM = 5.0    # tomar un barrido cada ~5 cm recorridos...
BARRIDO_CADA_DEG = 10.0  # ...o cada 10 grados girados...
BARRIDO_CADA_S = 1.5     # ...o cada 1.5 s (por si el robot queda casi quieto)
MAX_BARRIDOS  = 160      # 720 bytes cada uno
MEM_MIN_BARRIDOS = 45000 # bytes libres minimos para seguir guardando barridos
LIDAR_T_AJUSTE = 0.0     # s que se suman al instante de cada barrido (sincronizacion)

METODO_LIDAR  = "icp"    # "icp" = PL-ICP (nivel 3)  |  "simple" = niveles 1+2 de la guia
USAR_KEYFRAMES = True    # comparar contra un barrido de referencia (menos deriva)
KEYFRAME_DIST = 12.0     # cm   -> cambiar de referencia al moverse esto
KEYFRAME_ANG  = 12.0     # grados
PASO_GRADOS   = 4        # usar 1 de cada N grados del barrido actual (velocidad)
ICP_ITER      = 5
ICP_UMBRALES  = (25.0, 15.0, 10.0, 6.0, 5.0, 5.0)   # cm, rechazo de pares lejanos
ICP_MIN_PUNTOS = 30
ICP_MIN_INLIERS = 0.35   # fraccion minima de puntos emparejados
ICP_RMS_MAX   = 4.0      # cm
ICP_COV_ESCALA = 4.0     # la covarianza "hessiana" del ICP es optimista -> inflar
MAX_SALTO_CM  = 15.0     # diferencia maxima aceptada ICP vs odometria
MAX_SALTO_DEG = 20.0

# -- Filtro de Kalman ----------------------------------------------------------
# Covarianza inicial: en el cruce se DEFINE el origen, asi que la incertidumbre es baja.
P0 = [[0.01, 0.0, 0.0],
      [0.0, 0.01, 0.0],
      [0.0, 0.0, 1e-4]]
# Ruido de proceso Q (encoders + IMU). Se construye en cada paso como
#   Q = G * diag(sigma_s^2, sigma_th^2) * G^T + Q_MIN
#   sigma_s^2  = K_S * |ds|                    (error del encoder crece con lo recorrido)
#   sigma_th^2 = K_G * dt + (K_ESC * dth)^2    (deriva del giroscopio + error de escala)
K_S   = 0.02             # cm^2 por cm recorrido        (calibrar con p6)
K_G   = 3e-3             # rad^2 / s   incertidumbre del giro (sesgo residual, escala) -> calibrar
K_ESC = 0.02             # error relativo de escala del giro
Q = [[1e-4, 0.0, 0.0],   # Q_MIN (piso numerico), en cm^2 y rad^2
     [0.0, 1e-4, 0.0],
     [0.0, 0.0, 1e-7]]
# Ruido de medicion R (LiDAR), en el marco del robot. Piso minimo; el ICP entrega
# ademas su propia covarianza en cada barrido (se suma). Calibrar con p5 (robot quieto).
R = [[0.05, 0.0, 0.0],
     [0.0, 0.05, 0.0],
     [0.0, 0.0, 3e-5]]
R_SIMPLE = [[1.0, 0.0, 0.0],   # metodo "simple": menos preciso; su giro es poco confiable
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.05]]    # -> el rumbo lo manda el giroscopio
GATE_CHI2 = 11.34        # chi^2 3 GDL al 99 %: por encima se atenua la medicion
GATE_RECHAZO = 50.0      # d2 > GATE_RECHAZO * GATE_CHI2 -> se descarta

# =============================================================================
#  2. ESTADO GLOBAL
# =============================================================================
board = imu = motor1 = motor2 = reflectance = lidar_uart = None

x_hat = [0.0, 0.0, 0.0]          # [x (cm), y (cm), theta (rad)]
P = [row[:] for row in P0]
x_odo = [0.0, 0.0, 0.0]          # solo encoders + IMU (para comparar en el informe)
x_lid = [0.0, 0.0, 0.0]          # solo LiDAR encadenado (para comparar)

enc_L_prev = 0
enc_R_prev = 0
yaw_prev = 0.0
ultimo_ds = 0.0
ultimo_dth_enc = 0.0

scan_anterior = None             # barrido de referencia (keyframe)
ref_cache = None                 # puntos + normales del barrido de referencia
pose_ref = [0.0, 0.0, 0.0]       # x_hat cuando se tomo la referencia
pose_ref_lid = [0.0, 0.0, 0.0]   # x_lid cuando se tomo la referencia
_rx_buf = b""


diag = {"n": 0, "rms": 0.0, "ms": 0, "ok": False, "d2": 0.0, "R": None}
stats = {"scans": 0, "aceptadas": 0, "atenuadas": 0, "rech_gate": 0, "rech_icp": 0, "ms_total": 0}

_DEG = math.pi / 180.0
_COS = [math.cos(i * _DEG) for i in range(360)]
_SIN = [math.sin(i * _DEG) for i in range(360)]
if array is not None:                       # 4 bytes por valor en vez de un objeto float
    _COS = array("f", _COS)
    _SIN = array("f", _SIN)

# =============================================================================
#  3. UTILIDADES MATEMATICAS (3x3 a mano: MicroPython no trae numpy)
# =============================================================================
def wrap(a):
    """Normaliza un angulo a (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def mat_mul(A, B):
    return [[A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
             for j in range(3)] for i in range(3)]


def mat_T(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def mat_add(A, B):
    return [[A[i][j] + B[i][j] for j in range(3)] for i in range(3)]


def mat_sub(A, B):
    return [[A[i][j] - B[i][j] for j in range(3)] for i in range(3)]


def mat_vec(A, v):
    return [A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2] for i in range(3)]


def simetrizar(A):
    return [[0.5 * (A[i][j] + A[j][i]) for j in range(3)] for i in range(3)]


def mat_inv3(m):
    """Inversa 3x3 por la adjunta. Retorna None si es singular."""
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    A = e * i - f * h
    B = -(d * i - f * g)
    C = d * h - e * g
    det = a * A + b * B + c * C
    if abs(det) < 1e-15:
        return None
    k = 1.0 / det
    return [[A * k, -(b * i - c * h) * k, (b * f - c * e) * k],
            [B * k, (a * i - c * g) * k, -(a * f - c * d) * k],
            [C * k, -(a * h - b * g) * k, (a * e - b * d) * k]]


I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _nuevo_array():
    return array("f") if array is not None else []


def _array_ceros(n):
    return array("f", bytes(4 * n)) if array is not None else [0.0] * n


def componer(pose, delta):
    """pose (+) delta : aplica un desplazamiento expresado en el marco del robot."""
    c = math.cos(pose[2])
    s = math.sin(pose[2])
    return [pose[0] + c * delta[0] - s * delta[1],
            pose[1] + s * delta[0] + c * delta[1],
            wrap(pose[2] + delta[2])]


def relativa(a, b):
    """Desplazamiento de b visto desde a (en el marco del robot en a)."""
    c = math.cos(a[2])
    s = math.sin(a[2])
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return [c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])]


def rotar_cov(Rrel, th):
    """Lleva una covarianza del marco del robot al marco global: J R J^T."""
    c = math.cos(th)
    s = math.sin(th)
    J = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    return mat_mul(mat_mul(J, Rrel), mat_T(J))


# =============================================================================
#  4. HARDWARE
# =============================================================================
BAUD_DE = {"ld06": 230400}
lidar_rpm = 0.0


def abrir_lidar(baud=None):
    """UART del LiDAR. rxbuf grande para no perder bytes mientras se calcula."""
    global lidar_uart
    if baud is None:
        baud = LIDAR_BAUD or BAUD_DE.get(LIDAR_PROTOCOLO, 230400)
    try:
        lidar_uart.deinit()
    except Exception:
        pass
    lidar_uart = UART(LIDAR_UART_ID, baudrate=baud,
                      tx=Pin(LIDAR_TX_PIN), rx=Pin(LIDAR_RX_PIN),
                      rxbuf=8192, timeout=0)
    if LIDAR_PROTOCOLO == "auto":
        detectar_lidar()
    elif LIDAR_PROTOCOLO == "ld06":
        enviar_start_ld06()
    return lidar_uart


def _crc8_ld(datos):
    c = 0
    for b in datos:
        c = _CRC8[(c ^ b) & 0xFF]
    return c


def enviar_start_ld06():
    """El LiDAR del salon (LDROBOT, paquetes 54 2C a 230400) solo arranca a
    girar y transmitir cuando recibe este comando por su RX (IO4 del Qwiic 0).
    Luego sigue girando por su cuenta."""
    cmd = bytearray([0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])
    cmd[7] = _crc8_ld(cmd[:7])
    lidar_uart.write(cmd)


def detener_lidar():
    """Comando STOP del LiDAR LDROBOT (LD14P/LD06): 54 A1 04 00 00 00 00 + CRC8.
    Para el motor y la transmision; el LiDAR queda encendido pero quieto."""
    try:
        cmd = bytearray([0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])
        cmd[7] = _crc8_ld(cmd[:7])
        lidar_uart.write(cmd)
        time.sleep(0.05)
        lidar_uart.write(cmd)
    except Exception:
        pass


def _escuchar(ms):
    datos = b""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        n = lidar_uart.any()
        if n:
            datos += lidar_uart.read(n)
        time.sleep(0.01)
    return datos


def _abrir_uart(uid, tx, rx, baud):
    global lidar_uart
    try:
        lidar_uart.deinit()
    except Exception:
        pass
    lidar_uart = UART(uid, baudrate=baud, tx=Pin(tx), rx=Pin(rx), rxbuf=8192, timeout=0)
    time.sleep(0.05)
    if lidar_uart.any():
        lidar_uart.read(lidar_uart.any())       # descartar lo viejo


def detectar_lidar():
    """Prueba cada puerto y protocolo (enviando el comando de arranque del LD06)
    y se queda con el que entregue paquetes con checksum valido. Si no encuentra
    nada, desactiva el LiDAR y el reto sigue solo con encoders + IMU."""
    global LIDAR_PROTOCOLO, LIDAR_BAUD, _rx_buf, USAR_LIDAR
    global LIDAR_UART_ID, LIDAR_TX_PIN, LIDAR_RX_PIN
    for uid, tx, rx in PUERTOS_LIDAR:
        for proto in ("ld06",):
            baud = BAUD_DE[proto]
            try:
                _abrir_uart(uid, tx, rx, baud)
                if proto == "ld06":
                    enviar_start_ld06()
                    time.sleep(1.0)                  # que tome velocidad
                    lidar_uart.read(lidar_uart.any() or 1)
                datos = _escuchar(700)
            except Exception:
                continue
            _contador["ok"] = 0
            PARSERS[proto](datos)
            if _contador["ok"] >= 2:
                LIDAR_PROTOCOLO = proto
                LIDAR_BAUD = baud
                LIDAR_UART_ID, LIDAR_TX_PIN, LIDAR_RX_PIN = uid, tx, rx
                _rx_buf = b""
                _reiniciar_barrido()
                print("LiDAR detectado: {} a {} baudios en UART{} (RX = IO{})".format(
                    proto, baud, uid, rx))
                return proto
    print("ATENCION: no se encontro el LiDAR -> se sigue SOLO con encoders + IMU")
    LIDAR_PROTOCOLO = "ld06"
    USAR_LIDAR = False
    try:
        lidar_uart.deinit()
    except Exception:
        pass
    return None


def iniciar_hardware():
    global board, imu, motor1, motor2, reflectance
    board = Board.get_default_board()
    imu = IMU.get_default_imu()
    motor1 = EncodedMotor.get_default_encoded_motor(1)   # izquierdo
    motor2 = EncodedMotor.get_default_encoded_motor(2)   # derecho
    reflectance = Reflectance.get_default_reflectance()
    iniciar_marcha()
    if USAR_LIDAR:
        try:
            abrir_lidar()
        except Exception as e:                   # nunca dejar que el LiDAR tumbe el reto
            print("ATENCION: error abriendo el LiDAR ({}); se sigue sin el".format(e))
            _desactivar_lidar()


def _desactivar_lidar():
    global USAR_LIDAR
    USAR_LIDAR = False


def led(r, g, b):
    try:
        board.set_rgb_led(r, g, b)
    except Exception:
        pass


def parar_motores():
    """Detiene las dos ruedas (se intenta la segunda aunque falle la primera)."""
    for m in (motor1, motor2):
        try:
            m.set_effort(0.0)
            if hasattr(m, "brake"):
                m.brake()
        except Exception:
            pass


# =============================================================================
#  5. MARCHA: CONTROL DE VELOCIDAD POR RUEDA + SEGUIDOR DE LINEA
# =============================================================================
# Esta parte SOLO mueve el robot. Corre a 100 Hz con su propio reloj
# (servir_control) y no depende del LiDAR ni del filtro de Kalman.
rueda_l = rueda_r = seguidor = None
_control_activo = False       # True desde la salida hasta la meta
_t_control_us = 0
arranque_pendiente = False    # los dos sensores estan sobre la transversal de salida
meta_detectada = False
meta_habilitada = False
tiempo_cruce = 0.0
tiempo_fuera_cruce = 0.0
tiempo_marcha = 0.0
dist_control = 0.0
max_gap_s = 0.0
izq_c = der_c = 0.0           # ultima lectura de los sensores de linea
objetivo_l = objetivo_r = 0.0
omega_filtrada = 0.0          # rad/s (giroscopio + encoders)
yaw_control = 0.0
imu_bias_rad_s = 0.0
_min_l = _min_r = 1.0         # lo mas blanco / mas negro que vio cada sensor
_max_l = _max_r = 0.0


def limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


def leer_linea():
    """Promedio de SENSOR_SAMPLES lecturas de cada sensor (menos ruido)."""
    suma_l = 0.0
    suma_r = 0.0
    for _ in range(SENSOR_SAMPLES):
        suma_l += reflectance.get_left()
        suma_r += reflectance.get_right()
    if LINEA_INVERTIDA:
        return suma_r / SENSOR_SAMPLES, suma_l / SENSOR_SAMPLES
    return suma_l / SENSOR_SAMPLES, suma_r / SENSOR_SAMPLES


def normalizar_linea(valor, blanco, negro):
    """0 = blanco, 1 = negro."""
    return limitar((valor - blanco) / (negro - blanco), 0.0, 1.0)


def es_cruce(left, right):
    """Linea transversal: LOS DOS sensores en negro a la vez."""
    return left >= CROSS_RAW_LEFT and right >= CROSS_RAW_RIGHT


def frenar_motor(motor):
    if motor is not None:
        motor.set_effort(0.0)
        if hasattr(motor, "brake"):
            motor.brake()


class ControlRueda:
    """Velocidad de UNA rueda con su encoder: feedforward (friccion + kv*v) +
    PI con anti-windup + refuerzo limitado de arranque."""

    def __init__(self, motor, nombre, signo_encoder, ks, kv):
        self.motor = motor
        self.nombre = nombre
        self.signo_encoder = signo_encoder
        self.ks = ks
        self.kv = kv
        self.posicion = motor.get_position() * signo_encoder
        self.velocidad = 0.0
        self.dist_ventana = 0.0
        self.t_ventana = 0.0
        self.integral = 0.0
        self.esfuerzo = 0.0
        self.direccion = 0
        self.invirtiendo = False
        self.t_inversion = 0.0
        self.t_atasco = 0.0
        self.t_signo_incorrecto = 0.0

    def medir(self, dt):
        posicion = self.motor.get_position() * self.signo_encoder
        delta = (posicion - self.posicion) * CIRC_CONTROL
        if abs(delta) > ENCODER_MAX_CM_S * dt + 0.2:
            raise RuntimeError("Lectura de encoder invalida: " + self.nombre)
        self.posicion = posicion
        self.dist_ventana += delta
        self.t_ventana += dt
        if self.t_ventana >= VEL_WINDOW_S:
            medida = self.dist_ventana / self.t_ventana
            alpha = self.t_ventana / (VEL_FILTER_S + self.t_ventana)
            self.velocidad += alpha * (medida - self.velocidad)
            self.dist_ventana = 0.0
            self.t_ventana = 0.0
        return delta

    def parar(self):
        self.integral = 0.0
        self.esfuerzo = 0.0
        self.direccion = 0
        self.invirtiendo = False
        self.t_atasco = 0.0
        self.t_signo_incorrecto = 0.0
        frenar_motor(self.motor)

    def actualizar(self, objetivo, dt):
        objetivo = limitar(objetivo, -V_RUEDA_MAX_CM_S, V_RUEDA_MAX_CM_S)
        if abs(objetivo) < ZERO_SPEED_CM_S:
            self.parar()
            return

        direccion = 1 if objetivo > 0 else -1
        if direccion != self.direccion:
            self.integral = 0.0
            self.t_atasco = 0.0
            self.t_signo_incorrecto = 0.0
            self.invirtiendo = (self.direccion != 0 or
                                direccion * self.velocidad < -0.5)
            self.t_inversion = 0.0
            self.direccion = direccion

        # Frenar antes de invertir: no se invierte a plena potencia
        if self.invirtiendo:
            self.t_inversion += dt
            if (self.t_inversion < REVERSAL_BRAKE_S or
                    (abs(self.velocidad) > 0.7 and self.t_inversion < REVERSAL_MAX_S)):
                self.esfuerzo = 0.0
                frenar_motor(self.motor)
                return
            self.invirtiendo = False

        velocidad_dir = self.velocidad * direccion
        error = abs(objetivo) - velocidad_dir
        if velocidad_dir < -0.6:
            self.t_signo_incorrecto += dt
        else:
            self.t_signo_incorrecto = 0.0
        if self.t_signo_incorrecto > 0.4:
            raise RuntimeError("Revisar signo/cable del encoder: " + self.nombre)

        if (abs(objetivo) >= 0.6 and abs(self.velocidad) < ATASCO_VEL_CM_S
                and abs(self.esfuerzo) >= 0.24):
            self.t_atasco += dt
        else:
            self.t_atasco = max(0.0, self.t_atasco - 2.0 * dt)
        if self.t_atasco >= ATASCO_STOP_S:
            raise RuntimeError("Rueda bloqueada o encoder sin pulsos: " + self.nombre)

        boost = ATASCO_BOOST_MAX * limitar(
            (self.t_atasco - ATASCO_BOOST_AFTER_S) / 0.4, 0.0, 1.0)
        feedforward = self.ks + self.kv * abs(objetivo) + boost
        propuesta_i = limitar(self.integral + KI_RUEDA * error * dt,
                              -INTEGRAL_MAX, INTEGRAL_MAX)
        propuesta = feedforward + KP_RUEDA * error + propuesta_i
        techo_rampa = min(MAX_EFFORT, abs(self.esfuerzo) + EFFORT_RISE_PER_S * dt)
        # No acumular integral cuando el actuador o la rampa ya saturaron
        if ((0.0 <= propuesta <= techo_rampa) or
                (propuesta > techo_rampa and error < 0.0) or
                (propuesta < 0.0 and error > 0.0)):
            self.integral = propuesta_i
        magnitud = limitar(feedforward + KP_RUEDA * error + self.integral,
                           0.0, techo_rampa)
        self.esfuerzo = direccion * magnitud
        if magnitud == 0.0:
            frenar_motor(self.motor)
        else:
            self.motor.set_effort(self.esfuerzo)


class SeguidorLinea:
    """Decide la velocidad de cada rueda (cm/s) a partir de los dos sensores.
    Con linea visible: avance con correcciones suaves. Si se pierde: una rueda
    quieta y la otra avanzando hacia el ultimo lado donde se vio la linea."""

    def __init__(self):
        self.estado = "RECTA"
        self.error = self.derivada = self.avance = self.omega = 0.0
        self.presente = True
        self.en_curva = self.recuperando = False
        self.t_recuperacion = self.dist_perdida = 0.0
        self.rumbo = None
        self.lado_linea_memoria = 0.0
        self.curvatura_memoria = 0.0
        self.error_memoria = 0.0
        self.t_centrado_memoria = 0.0

    def ruedas(self, avance, omega):
        relacion = (1.0 - MIN_INNER_WHEEL_RATIO) / (1.0 + MIN_INNER_WHEEL_RATIO)
        limite = 2.0 * avance * relacion / TRACK_WIDTH
        omega = limitar(omega, -limite, limite)
        return avance - omega * TRACK_WIDTH / 2.0, avance + omega * TRACK_WIDTH / 2.0

    def actualizar(self, left, right, dt, rumbo, omega_medida=0.0):
        dt = limitar(dt, 0.001, MAX_CONTROL_GAP_S)
        if self.rumbo is None:
            self.rumbo = rumbo
        sl = normalizar_linea(left, WHITE_LEFT, BLACK_LEFT)
        sr = normalizar_linea(right, WHITE_RIGHT, BLACK_RIGHT)
        intensidad = max(sl, sr)
        if intensidad >= SIGNAL_ON:
            self.presente = True
        elif intensidad <= SIGNAL_OFF:
            self.presente = False
        cruce = es_cruce(left, right)
        if cruce:
            self.estado = "CRUCE"
            self.rumbo = rumbo
            self.recuperando = self.en_curva = False
            self.t_recuperacion = self.dist_perdida = 0.0
            self.error = self.derivada = 0.0
            self.curvatura_memoria = self.error_memoria = 0.0
            self.lado_linea_memoria = 0.0
            self.t_centrado_memoria = 0.0
            avance_obj = V_CRUCE_CM_S
            omega_obj = -YAW_DAMPING * omega_medida
        elif not self.presente:
            if not self.recuperando:
                self.rumbo = rumbo
            self.recuperando = True
            self.t_recuperacion += dt
            self.derivada = 0.0
            self.error = self.error_memoria
            if self.lado_linea_memoria:
                # Una rueda detenida y la otra avanza hacia el ultimo lado visto
                self.estado = ("RECUPERAR_IZQUIERDA" if self.lado_linea_memoria > 0
                               else "RECUPERAR_DERECHA")
                self.en_curva = True
                velocidad = V_RUEDA_RECUPERACION_CM_S
                izquierda = velocidad if self.lado_linea_memoria < 0 else 0.0
                derecha = velocidad if self.lado_linea_memoria > 0 else 0.0
                self.avance = velocidad * 0.5
                self.omega = (derecha - izquierda) / TRACK_WIDTH
            else:
                # Sin un lado observado no se inventa una direccion de giro
                self.estado = "AVANCE_SIN_LINEA"
                self.en_curva = False
                izquierda = derecha = V_PERDIDA_CM_S
                self.avance = V_PERDIDA_CM_S
                self.omega = 0.0
            self.dist_perdida += self.avance * dt
            return izquierda, derecha
        else:
            if self.recuperando:
                self.derivada = 0.0
            self.recuperando = False
            self.t_recuperacion = self.dist_perdida = 0.0
            error_raw = (sl - sr) / max(sl + sr, 1.0)
            # Guardar el lado antes del filtro y de los limites de giro
            if abs(error_raw) >= CURVA_ENTER_ERROR:
                self.lado_linea_memoria = 1.0 if error_raw > 0 else -1.0
                self.error_memoria = error_raw
            anterior = self.error
            self.error += dt / (LINE_ERROR_FILTER_S + dt) * (error_raw - self.error)
            error = 0.0 if abs(self.error) <= ERROR_DEADBAND else self.error
            derivada = (self.error - anterior) / dt
            self.derivada += dt / (DERIVATIVE_FILTER_S + dt) * (derivada - self.derivada)
            self.en_curva = abs(error) > CURVA_ENTER_ERROR
            self.estado = "CURVA" if self.en_curva else "RECTA"
            avance_obj = V_RECTA_CM_S - (V_RECTA_CM_S - V_CURVA_CM_S) * abs(error)
            omega_obj = KP_LINEA * error + limitar(KD_LINEA * self.derivada, -0.08, 0.08)
            omega_obj -= YAW_DAMPING * omega_medida
            self.rumbo = rumbo
        paso = ACEL_AVANCE_CM_S2 * dt
        self.avance += limitar(avance_obj - self.avance, -paso, paso)
        omega_obj = limitar(omega_obj, -OMEGA_LINEA_MAX, OMEGA_LINEA_MAX)
        paso_giro = STEERING_SLEW_RAD_S2 * dt
        self.omega += limitar(omega_obj - self.omega, -paso_giro, paso_giro)
        izquierda, derecha = self.ruedas(self.avance, self.omega)
        if self.presente and not cruce:
            if abs(self.error) >= CURVA_ENTER_ERROR:
                self.t_centrado_memoria = 0.0
                omega_aplicado = (derecha - izquierda) / TRACK_WIDTH
                if omega_aplicado * self.error > 0 and self.avance > 0.1:
                    self.curvatura_memoria = omega_aplicado / self.avance
                    self.error_memoria = self.error
            elif abs(self.error) <= CURVA_EXIT_ERROR:
                self.t_centrado_memoria += dt
                if self.t_centrado_memoria >= MEMORIA_CENTRADO_S:
                    self.curvatura_memoria = self.error_memoria = 0.0
            else:
                self.t_centrado_memoria = 0.0
        return izquierda, derecha


def leer_omega_imu(omega_fallback):
    """Velocidad de giro del giroscopio en rad/s (XRPLib la entrega en mdps).
    Si la IMU falla, se usa la de los encoders."""
    try:
        omega = math.radians(imu.get_gyro_z_rate() / 1000.0) * IMU_GYRO_SIGN - imu_bias_rad_s
        if abs(omega) <= 17.5:                   # 1000 grados/s
            return omega
    except Exception:
        pass
    return omega_fallback


def calibrar_bias_imu():
    """Sesgo residual del giroscopio con el robot quieto (2 s)."""
    global imu_bias_rad_s, omega_filtrada
    imu_bias_rad_s = omega_filtrada = 0.0
    suma = 0.0
    suma2 = 0.0
    n = 100
    for _ in range(n):
        v = leer_omega_imu(0.0)
        suma += v
        suma2 += v * v
        time.sleep(0.02)
    media = suma / n
    dispersion = math.sqrt(max(0.0, suma2 / n - media * media))
    if dispersion > math.radians(2.0) or abs(media) > math.radians(10.0):
        print("ATENCION: el robot se movio al calibrar el giroscopio; repita si gira raro")
        return
    imu_bias_rad_s = media


def iniciar_marcha():
    """Crea el control de las dos ruedas y el seguidor (motores frenados)."""
    global rueda_l, rueda_r, seguidor
    for m in (motor1, motor2):
        try:
            m.set_speed(0)                       # apaga el control de velocidad de XRPLib
        except Exception:
            pass
        frenar_motor(m)
    rueda_l = ControlRueda(motor1, "izquierda", ENCODER_L_SIGN, KS_L, KV_L)
    rueda_r = ControlRueda(motor2, "derecha", ENCODER_R_SIGN, KS_R, KV_R)
    seguidor = SeguidorLinea()


def parar_ruedas():
    try:
        rueda_l.parar()
    finally:
        rueda_r.parar()


def reiniciar_recorrido():
    """Pone en cero lo que decide la meta (se llama al salir)."""
    global tiempo_marcha, dist_control, meta_habilitada, meta_detectada, yaw_control
    global max_gap_s
    tiempo_marcha = dist_control = yaw_control = max_gap_s = 0.0
    meta_habilitada = meta_detectada = False
    seguidor.estado = "CRUCE"


def servir_control(forzar=False):
    """
    Control de la marcha con su PROPIO reloj (100 Hz). Se llama desde el lazo
    principal y tambien entre los calculos del filtro y del LiDAR, de modo que
    las ruedas nunca esperan a nadie.
      - mide las dos ruedas (encoders) y lee los sensores de linea
      - detecta la transversal de salida (robot quieto) y la de meta (en marcha)
      - el seguidor pide una velocidad a cada rueda y cada PI la cumple
    """
    global _t_control_us, max_gap_s, izq_c, der_c, objetivo_l, objetivo_r
    global tiempo_cruce, tiempo_fuera_cruce, meta_habilitada, meta_detectada
    global arranque_pendiente, tiempo_marcha, dist_control
    global omega_filtrada, yaw_control, _min_l, _min_r, _max_l, _max_r
    if rueda_l is None:
        return
    ahora = time.ticks_us()
    dt = time.ticks_diff(ahora, _t_control_us) / 1e6
    if not forzar and dt < LOOP_DT:
        return
    _t_control_us = ahora
    if dt < 0.001:
        dt = 0.001
    if _control_activo and dt > max_gap_s:
        max_gap_s = dt

    dl = rueda_l.medir(dt)
    dr = rueda_r.medir(dt)
    if dt > MAX_CONTROL_GAP_S:                   # hueco largo: no dar un salto al PI
        dt = MAX_CONTROL_GAP_S
    # giro: giroscopio (80 %) + encoders (20 %), filtrado
    omega_enc = (dr - dl) / (TRACK_WIDTH * dt)
    omega = 0.8 * leer_omega_imu(omega_enc) + 0.2 * omega_enc
    omega_filtrada += dt / (IMU_FILTER_S + dt) * (omega - omega_filtrada)
    yaw_control += omega_filtrada * dt

    izq_c, der_c = leer_linea()
    if izq_c < _min_l:
        _min_l = izq_c
    elif izq_c > _max_l:
        _max_l = izq_c
    if der_c < _min_r:
        _min_r = der_c
    elif der_c > _max_r:
        _max_r = der_c
    if es_cruce(izq_c, der_c):
        tiempo_cruce += dt
        tiempo_fuera_cruce = 0.0
    else:
        tiempo_cruce = 0.0
        tiempo_fuera_cruce += dt

    if not _control_activo:                      # esperando la salida, o ya en la meta
        rueda_l.parar()
        rueda_r.parar()
        if tiempo_cruce >= START_CONFIRM_S and not meta_detectada:
            arranque_pendiente = True
        return

    tiempo_marcha += dt
    dist_control += max(0.0, (dl + dr) * 0.5)
    if (tiempo_fuera_cruce >= CROSS_CLEAR_S and dist_control >= MIN_FIN_DIST_CM
            and tiempo_marcha >= MIN_FIN_TIME_S):
        meta_habilitada = True
    if meta_habilitada and tiempo_cruce >= CROSS_CONFIRM_S:
        meta_detectada = True
    if meta_detectada:
        rueda_l.parar()
        rueda_r.parar()
        return

    objetivo_l, objetivo_r = seguidor.actualizar(izq_c, der_c, dt, yaw_control, omega_filtrada)
    rueda_l.actualizar(objetivo_l, dt)
    rueda_r.actualizar(objetivo_r, dt)


def atender():
    """Lo que nunca debe esperar: las ruedas y los bytes del LiDAR."""
    servir_control()
    if USAR_LIDAR and lidar_uart is not None:
        bombear_diferido()


def umbral_cruce():
    return min(CROSS_RAW_LEFT, CROSS_RAW_RIGHT)


# =============================================================================
#  6. LECTURA DEL LiDAR
# =============================================================================
# Todos los protocolos terminan en lo mismo: una lista de 360 distancias en mm
# (indice = grado, 0 = sin dato) por cada vuelta del LiDAR.
_contador = {"ok": 0, "malos": 0}
lidar_bytes = 0
_acum = [0] * N_ANGULOS
_ang_prev = -1
_scan_listo = None


def _reiniciar_barrido():
    global _acum, _ang_prev, _scan_listo
    _acum = [0] * N_ANGULOS
    _ang_prev = -1
    _scan_listo = None


def _poner(ang, d):
    """Guarda un punto; si el angulo volvio a empezar, cierra la vuelta anterior."""
    global _acum, _ang_prev, _scan_listo
    if _ang_prev >= 0 and ang + 180 < _ang_prev:
        _scan_listo = _acum
        _acum = [0] * N_ANGULOS
    _ang_prev = ang
    _acum[ang] = d


# --- LDROBOT LD06 / LD19: 47 bytes, 12 puntos, CRC-8 (polinomio 0x4D) ---------
def _tabla_crc8():
    t = bytearray(256)
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t[i] = c
    return t


_CRC8 = _tabla_crc8()


def _parse_ld06(buf):
    i = 0
    L = len(buf)
    tabla = _CRC8
    while i + 47 <= L:
        if buf[i] != 0x54 or buf[i + 1] != 0x2C:
            i += 1
            continue
        if LIDAR_VERIFICAR_CRC:
            c = 0
            for k in range(i, i + 46):
                c = tabla[(c ^ buf[k]) & 0xFF]
            if c != buf[i + 46]:
                _contador["malos"] += 1
                i += 1
                continue
        a0 = (buf[i + 4] | (buf[i + 5] << 8)) * 0.01
        a1 = (buf[i + 42] | (buf[i + 43] << 8)) * 0.01
        if a1 < a0:
            a1 += 360.0
        paso = (a1 - a0) / 11.0
        for k in range(12):
            o = i + 6 + 3 * k
            d = buf[o] | (buf[o + 1] << 8)
            if buf[o + 2] < 20:                  # intensidad muy baja: dato dudoso
                d = 0
            _poner(int(a0 + paso * k + 0.5) % 360, d)
        _contador["ok"] += 1
        i += 47
    return i


PARSERS = {"ld06": _parse_ld06}




def bombear_lidar(max_bytes=None):
    """Lee el UART y procesa como maximo 'max_bytes' (unos pocos paquetes, ~1 ms)
    para no demorar el control. La vuelta completa queda en _scan_listo."""
    global _rx_buf, lidar_bytes
    n = lidar_uart.any()
    if n:
        nuevo = lidar_uart.read(n)
        if nuevo:
            _rx_buf = _rx_buf + nuevo
            lidar_bytes += len(nuevo)
    if len(_rx_buf) > 12000:                    # demasiado atraso: quedarse con lo ultimo
        _rx_buf = _rx_buf[-6000:]
    if max_bytes is None or len(_rx_buf) <= max_bytes:
        usado = PARSERS[LIDAR_PROTOCOLO](_rx_buf)
    else:
        usado = PARSERS[LIDAR_PROTOCOLO](_rx_buf[:max_bytes])
        if usado == 0:                           # nada completo en el trozo: saltar basura
            usado = max_bytes // 2
    if usado:
        _rx_buf = _rx_buf[usado:]


def leer_lidar(max_bytes=None):
    """
    Lee lo que haya en el UART, arma las vueltas del LiDAR y devuelve la vuelta
    completa MAS RECIENTE (360 distancias en mm, 0 = sin dato) o None.

    Mejora sobre la funcion de la plantilla: aquella consume 1 byte por llamada
    cuando se desincroniza (nunca alcanza el flujo de datos) y entrega la trama
    mas vieja. Aqui se acumula en un buffer, se busca la cabecera, se valida el
    checksum de cada paquete y siempre se entrega la vuelta mas nueva.
    """
    global _scan_listo
    bombear_lidar(max_bytes)
    s = _scan_listo
    _scan_listo = None
    return s


# --- Modo diferido: tomar un barrido de vez en cuando, sin calcular nada ------
_basura = bytearray(512)
_cap_estado = 0          # 0 libre (se descartan bytes), 1 esperando inicio de vuelta, 2 capturando
_cap_t0 = 0
_cap_t_vuelta = 0
_barrido_listo = None
_MAX_BYTES_DIFERIDO = {"ld06": 47 * 3}


def _vaciar_uart():
    """Descarta lo que haya llegado por el UART (barato: sin decodificar)."""
    global lidar_bytes
    for _ in range(40):
        if not lidar_uart.any():
            break
        k = lidar_uart.readinto(_basura)
        if not k:
            break
        lidar_bytes += k


def solicitar_barrido():
    """Empieza a capturar UNA vuelta completa del LiDAR."""
    global _cap_estado, _cap_t0, _rx_buf, _scan_listo
    if lidar_uart is None or _cap_estado != 0:
        return
    _vaciar_uart()
    _rx_buf = b""
    _reiniciar_barrido()
    _scan_listo = None
    _cap_estado = 1
    _cap_t0 = time.ticks_ms()


def _t_llegada():
    """Instante (ticks_ms) en que llego el ultimo byte ya decodificado: lo que
    queda sin decodificar es mas nuevo (a 230400 baudios llegan 23 bytes/ms)."""
    pendiente = len(_rx_buf) + lidar_uart.any()
    bpms = (LIDAR_BAUD or BAUD_DE.get(LIDAR_PROTOCOLO, 230400)) / 10000.0
    return time.ticks_add(time.ticks_ms(), -int(pendiente / bpms))


def bombear_diferido():
    """
    Lo unico que hace el LiDAR durante el recorrido en modo diferido: si no se
    esta capturando, tira los bytes; si se esta capturando, decodifica pocos
    paquetes por llamada (~1-2 ms) hasta completar UNA vuelta. Anota tambien el
    instante REAL de la mitad de la vuelta (para asociarla a la pose correcta).
    """
    global _cap_estado, _barrido_listo, _scan_listo, _cap_t_vuelta
    if lidar_uart is None:
        return
    if _cap_estado == 0:
        _vaciar_uart()
        return
    bombear_lidar(_MAX_BYTES_DIFERIDO.get(LIDAR_PROTOCOLO, 200))
    if _scan_listo is not None:
        s = _scan_listo
        _scan_listo = None
        t_fin = _t_llegada()
        if _cap_estado == 1:
            _cap_estado = 2                      # la 1ra vuelta estaba incompleta
            _cap_t_vuelta = t_fin                # aqui empieza la vuelta buena
        else:
            t_mid = time.ticks_add(_cap_t_vuelta, time.ticks_diff(t_fin, _cap_t_vuelta) // 2)
            _barrido_listo = (s, t_mid)
            _cap_estado = 0
    elif time.ticks_diff(time.ticks_ms(), _cap_t0) > 600:
        _cap_estado = 0                          # no llego nada: se intenta despues


def tomar_barrido():
    """Devuelve (vuelta, ticks_ms de la mitad de la vuelta) o None, y la olvida."""
    global _barrido_listo
    s = _barrido_listo
    _barrido_listo = None
    return s


# =============================================================================
#  7. ESTIMACION DE MOVIMIENTO CON EL LiDAR
# =============================================================================
def _idx_a_robot(i):
    """Indice del barrido -> angulo (en grados enteros 0..359) en el marco del robot."""
    return (LIDAR_SENTIDO * (i - LIDAR_OFFSET_DEG)) % 360


def scan_a_puntos(scan, paso):
    """Barrido polar -> puntos (x, y) en cm, en el marco del robot."""
    xs = _nuevo_array()
    ys = _nuevo_array()
    for i in range(0, N_ANGULOS, paso):
        d = scan[i]
        if D_MIN_MM < d < D_MAX_MM:
            k = _idx_a_robot(i)
            dc = d * 0.1
            xs.append(LIDAR_X + dc * _COS[k])
            ys.append(LIDAR_Y + dc * _SIN[k])
    return xs, ys


def preparar_referencia(scan):
    """Version de una sola llamada (la usan las pruebas)."""
    res = {}
    for _ in _preparar_gen(scan, res):
        pass
    return res["ref"]


def _preparar_gen(scan, res):
    """Puntos del barrido de referencia (1 por grado) + normal de la superficie
    en cada punto (necesaria para ICP punto-a-linea). Generador: se hace por
    trozos para no frenar el control de los motores."""
    rx = _array_ceros(N_ANGULOS)         # array('f'): 4 bytes por valor (poca RAM)
    ry = _array_ceros(N_ANGULOS)
    nx = _array_ceros(N_ANGULOS)
    ny = _array_ceros(N_ANGULOS)
    ok = bytearray(N_ANGULOS)            # 0 = sin punto, 1 = punto, 2 = punto + normal
    for i in range(N_ANGULOS):
        d = scan[i]
        if D_MIN_MM < d < D_MAX_MM:
            k = _idx_a_robot(i)
            dc = d * 0.1
            rx[i] = LIDAR_X + dc * _COS[k]
            ry[i] = LIDAR_Y + dc * _SIN[k]
            ok[i] = 1
        if i % 60 == 59:
            yield
    for i in range(N_ANGULOS):
        if i % 60 == 59:
            yield
        if not ok[i]:
            continue
        a = (i - 1) % N_ANGULOS
        b = (i + 1) % N_ANGULOS
        if ok[a] and ok[b]:
            tx = rx[b] - rx[a]
            ty = ry[b] - ry[a]
            esc = 2.0
        elif ok[b]:
            tx = rx[b] - rx[i]
            ty = ry[b] - ry[i]
            esc = 1.0
        elif ok[a]:
            tx = rx[i] - rx[a]
            ty = ry[i] - ry[a]
            esc = 1.0
        else:
            continue
        Lt = math.sqrt(tx * tx + ty * ty)
        # si los vecinos estan muy separados es un borde / discontinuidad
        if Lt < 1e-6 or Lt > esc * (0.06 * scan[i] * 0.1 + 3.0):
            continue
        nx[i] = -ty / Lt
        ny[i] = tx / Lt
        ok[i] = 2
    res["ref"] = (rx, ry, nx, ny, ok)


def _icp_gen(px, py, ref, guess, res):
    """
    PL-ICP (Censi 2008, simplificado). Busca (tx, ty, th) tal que
    R(th)*p + t cae sobre las superficies del barrido de referencia.
    - Asociacion proyectiva: el punto transformado se convierte a angulo y se
      busca su vecino solo en los indices j-1, j, j+1 (O(n), apto para MicroPython).
    - Error punto-a-linea: r = n . (R p + t - q)  -> no penaliza deslizar sobre
      una pared, que es justo lo que sesga al ICP punto-a-punto.
    - Gauss-Newton: A*delta = -b con A = sum J^T J.
    Covarianza del resultado ~ sigma_r^2 * A^-1 (metodo de la hessiana).
    Es un GENERADOR: hace 'yield' despues de cada iteracion para que el lazo de
    control no se congele mientras se calcula. El resultado queda en res["icp"].
    """
    res["icp"] = None
    rx, ry, nx, ny, ok = ref
    tx, ty, th = guess[0], guess[1], guess[2]
    atan2 = math.atan2
    npts = len(px)
    Ai = None
    n = 0
    sr2 = 0.0
    nu = len(ICP_UMBRALES)
    for it in range(ICP_ITER):
        c = math.cos(th)
        s = math.sin(th)
        u = ICP_UMBRALES[it if it < nu else nu - 1]
        u2 = u * u
        A00 = A01 = A02 = A11 = A12 = A22 = 0.0
        b0 = b1 = b2 = 0.0
        n = 0
        sr2 = 0.0
        for k in range(npts):
            if k % 30 == 29:
                yield                              # trozos cortos: el control no espera
            x = px[k]
            y = py[k]
            ax = c * x - s * y
            ay = s * x + c * y
            qx = ax + tx
            qy = ay + ty
            ang = atan2(qy - LIDAR_Y, qx - LIDAR_X) / _DEG
            j0 = int(round(LIDAR_OFFSET_DEG + LIDAR_SENTIDO * ang))
            best = -1
            bd = u2
            for j in (j0 - 1, j0, j0 + 1):
                j %= N_ANGULOS
                if ok[j] == 2:
                    ex = qx - rx[j]
                    ey = qy - ry[j]
                    d2 = ex * ex + ey * ey
                    if d2 < bd:
                        bd = d2
                        best = j
            if best < 0:
                continue
            n0 = nx[best]
            n1 = ny[best]
            r = n0 * (qx - rx[best]) + n1 * (qy - ry[best])
            J2 = n1 * ax - n0 * ay
            A00 += n0 * n0
            A01 += n0 * n1
            A02 += n0 * J2
            A11 += n1 * n1
            A12 += n1 * J2
            A22 += J2 * J2
            b0 += n0 * r
            b1 += n1 * r
            b2 += J2 * r
            n += 1
            sr2 += r * r
        if n < ICP_MIN_PUNTOS:
            return
        lam = 1e-6 * (A00 + A11 + A22) + 1e-9          # amortiguamiento (Levenberg)
        Ai = mat_inv3([[A00 + lam, A01, A02], [A01, A11 + lam, A12], [A02, A12, A22 + lam]])
        if Ai is None:
            return
        d0 = -(Ai[0][0] * b0 + Ai[0][1] * b1 + Ai[0][2] * b2)
        d1 = -(Ai[1][0] * b0 + Ai[1][1] * b1 + Ai[1][2] * b2)
        d2_ = -(Ai[2][0] * b0 + Ai[2][1] * b1 + Ai[2][2] * b2)
        tx += d0
        ty += d1
        th += d2_
        if it >= 2 and abs(d0) < 0.02 and abs(d1) < 0.02 and abs(d2_) < 0.0003:
            break
        yield
    s2 = sr2 / (n - 3) if n > 3 else sr2
    cov = [[Ai[i][j] * s2 for j in range(3)] for i in range(3)]
    res["icp"] = (tx, ty, wrap(th), n, npts, math.sqrt(sr2 / n), cov)


def _mediana(v):
    if not v:
        return None
    v = sorted(v)
    return v[len(v) // 2]


def _estimar_simple(scan_actual, scan_anterior, guess):
    """
    Niveles 1 + 2 de la guia.
    1) Rotacion: corrimiento circular del arreglo que minimiza la diferencia
       (busqueda alrededor del giro del giroscopio + refinamiento parabolico).
    2) Traslacion: con la rotacion compensada, cambio de distancia en conos de
       +-5 grados al frente y atras (dx). dy sale del modelo de arco (no holonomico).
    """
    c0 = -int(round(LIDAR_SENTIDO * guess[2] / _DEG))
    costos = []
    for sft in range(c0 - 6, c0 + 7):
        tot = 0.0
        m = 0
        for i in range(0, N_ANGULOS, 2):
            a = scan_anterior[i]
            b = scan_actual[(i + sft) % N_ANGULOS]
            if D_MIN_MM < a < D_MAX_MM and D_MIN_MM < b < D_MAX_MM:
                d = a - b
                if d < 0:
                    d = -d
                tot += d if d < 300 else 300
                m += 1
        costos.append(tot / m if m > 20 else 1e9)
    kmin = 0
    for k in range(len(costos)):
        if costos[k] < costos[kmin]:
            kmin = k
    fino = 0.0
    if 0 < kmin < len(costos) - 1:
        y0, y1, y2 = costos[kmin - 1], costos[kmin], costos[kmin + 1]
        den = y0 - 2 * y1 + y2
        if den > 1e-9:
            fino = 0.5 * (y0 - y2) / den
    sft = c0 - 6 + kmin
    dth = -LIDAR_SENTIDO * (sft + fino) * _DEG
    s_ent = int(round(sft + fino))

    def cono(ang_robot):
        vals = []
        for dk in range(-5, 6):
            i = (LIDAR_OFFSET_DEG + LIDAR_SENTIDO * (ang_robot + dk)) % N_ANGULOS
            a = scan_anterior[i]
            b = scan_actual[(i + s_ent) % N_ANGULOS]
            if D_MIN_MM < a < D_MAX_MM and D_MIN_MM < b < D_MAX_MM:
                vals.append((a - b) * 0.1)
        return _mediana(vals)

    frente = cono(0)
    atras = cono(180)
    dxs = [v for v in (frente, None if atras is None else -atras) if v is not None]
    if not dxs:
        return None
    dx = sum(dxs) / len(dxs)
    # Lateral: los lados dan malas medidas (la pared casi paralela al rayo amplifica
    # el error). El robot diferencial no se mueve de lado: en un arco, dy = dx*tan(dth/2).
    dy = dx * math.tan(0.5 * dth)
    return dx, dy, dth


def trabajo_lidar(scan_actual, scan_anterior, guess, res):
    """
    Estimacion de movimiento por pasos (generador). Cada 'next()' hace un trozo
    de trabajo de pocos ms. Al terminar deja en res["d"] = (dx, dy, dtheta) y en
    'diag' la calidad y la covarianza R del resultado.
    """
    global ref_cache
    diag["ok"] = False
    diag["n"] = 0
    res["d"] = (guess[0], guess[1], guess[2])
    out = None
    if METODO_LIDAR == "icp":
        if ref_cache is None or ref_cache[0] is not scan_anterior:
            rr = {}
            for _ in _preparar_gen(scan_anterior, rr):
                yield
            ref_cache = (scan_anterior, rr["ref"])
            yield
        px, py = scan_a_puntos(scan_actual, PASO_GRADOS)
        if len(px) >= ICP_MIN_PUNTOS:
            yield
            for _ in _icp_gen(px, py, ref_cache[1], guess, res):
                yield
            icp = res["icp"]
            if icp is not None:
                tx, ty, th, n, npts, rms, cov = icp
                diag["n"] = n
                diag["rms"] = rms
                if n >= ICP_MIN_INLIERS * npts and rms <= ICP_RMS_MAX:
                    out = (tx, ty, th)
                    diag["R"] = mat_add([[cov[i][j] * ICP_COV_ESCALA for j in range(3)]
                                         for i in range(3)], R)
    else:
        out = _estimar_simple(scan_actual, scan_anterior, guess)
        if out is not None:
            diag["n"] = 1
            diag["R"] = R_SIMPLE
    if out is not None:
        if (abs(out[0] - guess[0]) < MAX_SALTO_CM and abs(out[1] - guess[1]) < MAX_SALTO_CM
                and abs(wrap(out[2] - guess[2])) < MAX_SALTO_DEG * _DEG):
            diag["ok"] = True
            res["d"] = out


def estimar_movimiento_lidar(scan_actual, scan_anterior, guess=None):
    """
    Estima (dx, dy, dtheta) del robot entre 'scan_anterior' y 'scan_actual'
    (funcion pedida en la plantilla). Unidades: cm y radianes, en el marco del
    robot en 'scan_anterior' -> luego se COMPONE con la pose (no se suma directo).
    'guess' = desplazamiento que predice la odometria (semilla del ICP).
    Si el emparejamiento falla, devuelve la prediccion (diag["ok"] = False).
    """
    if guess is None:
        guess = [0.0, 0.0, 0.0]
    t0 = time.ticks_ms()
    res = {}
    for _ in trabajo_lidar(scan_actual, scan_anterior, guess, res):
        pass
    diag["ms"] = time.ticks_diff(time.ticks_ms(), t0)
    return res["d"]


# =============================================================================
#  8. ODOMETRIA: velocidades a partir de encoders + IMU
# =============================================================================
def calcular_v_omega(dt=DT):
    """
    v     = (vR + vL)/2 con v_rueda = d_ticks * 2*pi*r / (N * dt)   [ec. (3)-(4)]
    omega = d(yaw)/dt del giroscopio (XRPLib integra el giroscopio a ~200 Hz,
            mas preciso que muestrear la tasa una vez por ciclo); si
            USAR_GIROSCOPIO = False se usa (vR - vL)/L.
    """
    global enc_L_prev, enc_R_prev, yaw_prev, ultimo_ds, ultimo_dth_enc
    eL = motor1.get_position_counts()
    eR = motor2.get_position_counts()
    yaw = imu.get_yaw()
    k = 2.0 * math.pi * RADIO_RUEDA / TICKS_POR_REV
    dL = (eL - enc_L_prev) * k
    dR = (eR - enc_R_prev) * k
    enc_L_prev = eL
    enc_R_prev = eR
    dyaw = (yaw - yaw_prev) * _DEG
    yaw_prev = yaw
    ds = 0.5 * (dL + dR)
    dth_enc = (dR - dL) / TRACK_WIDTH
    ultimo_ds = ds
    ultimo_dth_enc = dth_enc
    dth = dyaw if USAR_GIROSCOPIO else dth_enc
    if dt <= 0:
        dt = DT
    return ds / dt, dth / dt


# =============================================================================
#  9. FILTRO DE KALMAN EXTENDIDO
# =============================================================================
def calcular_Q(ds, dth, dt, th_m):
    """Q = G M G^T + Q_MIN: el ruido se modela en las entradas (ds, dth) y se
    propaga al estado. Asi Q ~ 0 con el robot quieto y crece con lo recorrido."""
    c = math.cos(th_m)
    s = math.sin(th_m)
    vs = K_S * abs(ds) + 1e-6
    vt = K_G * dt + (K_ESC * dth) ** 2 + 1e-9
    G = [[c, -0.5 * ds * s], [s, 0.5 * ds * c], [0.0, 1.0]]
    Qk = [[G[i][0] * vs * G[j][0] + G[i][1] * vt * G[j][1] for j in range(3)]
          for i in range(3)]
    return mat_add(Qk, Q)


def kalman_prediccion(x_hat_prev, P_prev, v, omega, dt):
    """
    PREDICCION. Modelo de movimiento (ec. 5-7) integrado en el punto medio
    (theta + omega*dt/2), mas exacto que Euler en curvas:
        x = x + v dt cos(th_m) ;  y = y + v dt sin(th_m) ;  th = th + omega dt
    Jacobiano F = [[1,0,-v dt sin(th_m)], [0,1, v dt cos(th_m)], [0,0,1]]
    P_pred = F P F^T + Q
    """
    ds = v * dt
    dth = omega * dt
    th_m = x_hat_prev[2] + 0.5 * dth
    c = math.cos(th_m)
    s = math.sin(th_m)
    x_pred = [x_hat_prev[0] + ds * c,
              x_hat_prev[1] + ds * s,
              wrap(x_hat_prev[2] + dth)]
    F = [[1.0, 0.0, -ds * s],
         [0.0, 1.0, ds * c],
         [0.0, 0.0, 1.0]]
    P_pred = mat_add(mat_mul(mat_mul(F, P_prev), mat_T(F)), calcular_Q(ds, dth, dt, th_m))
    return x_pred, simetrizar(P_pred)


def kalman_actualizacion(x_pred, P_pred, z, Rk=None):
    """
    CORRECCION con el LiDAR (H = I):
        y = z - x_pred            (innovacion, con el angulo normalizado)
        S = P_pred + R
        K = P_pred S^-1
        x = x_pred + K y
        P = (I-K) P_pred (I-K)^T + K R K^T   (forma de Joseph: P siempre simetrica
                                              y definida positiva)
    Validacion robusta (gating suave): d2 = y^T S^-1 y (distancia de Mahalanobis).
      d2 <= chi2(3 GDL, 99 %)       -> se usa tal cual (codigo 1)
      d2 mayor                      -> se infla R por d2/chi2 (la medicion pesa menos)
      d2 > GATE_RECHAZO * chi2      -> se descarta (dato aberrante)
    Rechazar del todo mediciones "algo raras" hace que el filtro pierda la
    correccion justo cuando mas la necesita; atenuarlas es mas estable.
    """
    if Rk is None:
        Rk = R
    y = [z[0] - x_pred[0], z[1] - x_pred[1], wrap(z[2] - x_pred[2])]
    S = mat_add(P_pred, Rk)
    Si = mat_inv3(S)
    if Si is None:
        diag["d2"] = -1.0
        return x_pred, P_pred
    Siy = mat_vec(Si, y)
    d2 = y[0] * Siy[0] + y[1] * Siy[1] + y[2] * Siy[2]
    diag["d2"] = d2
    if d2 > GATE_CHI2:
        f = d2 / GATE_CHI2
        if f > GATE_RECHAZO:
            return x_pred, P_pred
        Rk = [[Rk[i][j] * f for j in range(3)] for i in range(3)]
        Si = mat_inv3(mat_add(P_pred, Rk))
        if Si is None:
            return x_pred, P_pred
    K = mat_mul(P_pred, Si)
    Ky = mat_vec(K, y)
    x_new = [x_pred[0] + Ky[0], x_pred[1] + Ky[1], wrap(x_pred[2] + Ky[2])]
    IK = mat_sub(I3, K)
    P_new = mat_add(mat_mul(mat_mul(IK, P_pred), mat_T(IK)),
                    mat_mul(mat_mul(K, Rk), mat_T(K)))
    return x_new, simetrizar(P_new)


# =============================================================================
#  10. REGISTRO DE DATOS (en RAM; se escribe al final)
# =============================================================================
# Escribir en la flash en cada ciclo (como la plantilla) desactiva las
# interrupciones varios ms: se pierden muestras del giroscopio (yaw) y bytes del
# UART del LiDAR. Por eso se guarda en RAM y se escribe al final (o al detener).
COLUMNAS = ("t", "x", "y", "th", "x_odo", "y_odo", "th_odo", "x_lid", "y_lid",
            "Pxx", "Pyy", "Ptt", "lidar", "n_icp", "ms_icp", "izq", "der", "us_cm")


reg_x = _nuevo_array()
reg_y = _nuevo_array()
reg_full = [_nuevo_array() for _ in COLUMNAS]


def guardar_punto(px, py):
    if len(reg_x) < MAX_FILAS:
        reg_x.append(px)
        reg_y.append(py)


def guardar_fila(valores):
    if len(reg_full[0]) < MAX_FILAS:
        for k in range(len(COLUMNAS)):
            reg_full[k].append(valores[k])


def escribir_archivos():
    with open(LOG_FILE, "w") as f:
        f.write("x,y\n")
        for k in range(len(reg_x)):
            f.write("{:.4f},{:.4f}\n".format(reg_x[k], reg_y[k]))
    with open(LOG_CSV, "w") as f:                # el que se grafica: se SOBREESCRIBE
        f.write("x,y\n")
        ult = None
        for k in range(len(reg_x)):
            fila = "{:.4f},{:.4f}\n".format(reg_x[k], reg_y[k])
            if fila != ult:                      # robot quieto: no repetir puntos
                f.write(fila)
                ult = fila
    with open(LOG_COMPLETO, "w") as f:
        f.write(",".join(COLUMNAS) + "\n")
        for k in range(len(reg_full[0])):
            f.write(",".join(["{:.5g}".format(col[k]) for col in reg_full]) + "\n")
    print("Guardado:", LOG_FILE, "y", LOG_CSV, "(", len(reg_x), "puntos ) y", LOG_COMPLETO)


# =============================================================================
#  11. PROGRAMA PRINCIPAL
# =============================================================================
def iniciar_grabacion(scan_ultimo):
    """Primer cruce: se define el origen (0, 0, 0)."""
    global x_hat, P, x_odo, x_lid, scan_anterior, ref_cache, pose_ref, pose_ref_lid
    global yaw_prev, enc_L_prev, enc_R_prev
    x_hat = [0.0, 0.0, 0.0]
    P = [row[:] for row in P0]
    x_odo = [0.0, 0.0, 0.0]
    x_lid = [0.0, 0.0, 0.0]
    imu.reset_yaw()
    yaw_prev = imu.get_yaw()
    enc_L_prev = motor1.get_position_counts()
    enc_R_prev = motor2.get_position_counts()
    scan_anterior = None            # la referencia sera el primer barrido tomado DESPUES del origen
    ref_cache = None
    pose_ref = [0.0, 0.0, 0.0]
    pose_ref_lid = [0.0, 0.0, 0.0]
    guardar_punto(0.0, 0.0)


def inversa(d):
    """Desplazamiento inverso: inversa(d) (+) d = identidad."""
    return relativa(d, [0.0, 0.0, 0.0])


def iniciar_trabajo(scan):
    """Arranca la estimacion LiDAR del barrido 'scan' (se ejecuta por partes)."""
    guess = relativa(pose_ref, x_hat)               # lo que dice la odometria
    res = {"scan": scan, "x_scan": x_hat[:], "ms": 0}
    res["gen"] = trabajo_lidar(scan, scan_anterior, guess, res)
    stats["scans"] += 1
    return res


def terminar_trabajo(res):
    """
    CORRECCION del EKF cuando el scan matching termina. Como el calculo toma
    varios ciclos, el robot se movio 'delta' desde que se tomo el barrido; esa
    parte la conoce la odometria, asi que la medicion se lleva al instante actual:
        z = x_ref (+) d_lidar (+) delta
    Retorna codigo para el log: 1 aceptada (o atenuada), 2 descartada, 3 fallo ICP.
    """
    global x_hat, P, x_lid, scan_anterior, pose_ref, pose_ref_lid
    d = res["d"]
    delta = relativa(res["x_scan"], x_hat)
    stats["ms_total"] += res["ms"]
    diag["ms"] = res["ms"]
    codigo = 3
    if diag["ok"]:
        z = componer(componer(pose_ref, d), delta)
        Rg = rotar_cov(diag["R"], pose_ref[2])
        x_hat, P = kalman_actualizacion(x_hat, P, z, Rg)
        if 0 <= diag["d2"] <= GATE_RECHAZO * GATE_CHI2:
            codigo = 1
            stats["aceptadas"] += 1
            if diag["d2"] > GATE_CHI2:
                stats["atenuadas"] += 1
        else:
            codigo = 2
            stats["rech_gate"] += 1
        x_lid_scan = componer(pose_ref_lid, d)
    else:
        stats["rech_icp"] += 1
        x_lid_scan = componer(pose_ref_lid, relativa(pose_ref, res["x_scan"]))
    x_lid = componer(x_lid_scan, delta)
    # Cambio de barrido de referencia (keyframe)
    pose_scan = componer(x_hat, inversa(delta))     # pose corregida en el instante del barrido
    mov = relativa(pose_ref, pose_scan)
    if (not USAR_KEYFRAMES or codigo != 1 or
            math.sqrt(mov[0] ** 2 + mov[1] ** 2) > KEYFRAME_DIST or
            abs(mov[2]) > KEYFRAME_ANG * _DEG):
        scan_anterior = res["scan"]
        pose_ref = pose_scan
        pose_ref_lid = x_lid_scan
    return codigo


def mover(pose, ds, dth):
    """Solo odometria (sin filtro), para comparar en el informe."""
    th_m = pose[2] + 0.5 * dth
    return [pose[0] + ds * math.cos(th_m), pose[1] + ds * math.sin(th_m), wrap(pose[2] + dth)]


# --- Modo diferido: lo que se guarda durante el recorrido ------------------------
rec_t = _nuevo_array()       # s desde el origen
rec_dt = _nuevo_array()      # s
rec_ds = _nuevo_array()      # cm avanzados en el ciclo (encoders)
rec_dth = _nuevo_array()     # rad girados en el ciclo (giroscopio)
rec_izq = _nuevo_array()
rec_der = _nuevo_array()
barridos = []                # (t en s de la mitad de la vuelta, 360 distancias en mm)
_factor_barridos = 1.0       # crece si la pista es larga y se llena la memoria


def guardar_ciclo(t, dt, ds, dth, izq, der):
    if len(rec_t) < MAX_FILAS:
        rec_t.append(t)
        rec_dt.append(dt)
        rec_ds.append(ds)
        rec_dth.append(dth)
        rec_izq.append(izq)
        rec_der.append(der)


def guardar_barrido(scan, t):
    """Guarda la vuelta y su instante (720 bytes con array 'H')."""
    global _factor_barridos
    if len(rec_t) == 0:
        return False
    lleno = len(barridos) >= MAX_BARRIDOS
    try:
        lleno = lleno or gc.mem_free() < MEM_MIN_BARRIDOS
    except Exception:
        pass
    if lleno:
        # pista larga: quedarse con 1 de cada 2 y seguir tomando al doble de distancia
        if len(barridos) < 8:
            return False
        barridos[:] = barridos[::2]
        _factor_barridos *= 2.0
        gc.collect()
    try:
        s = array("H", scan) if array is not None else list(scan)
    except MemoryError:
        return False
    barridos.append((t + LIDAR_T_AJUSTE, s))
    return True


def reprocesar(usar_lidar=True):
    """
    Se ejecuta con el robot YA DETENIDO. Repite el filtro de Kalman ciclo a ciclo
    con los datos guardados (prediccion con encoders + giroscopio) y, en los
    ciclos donde se guardo un barrido, hace el scan matching y la correccion.
    Es exactamente el mismo EKF que en vivo, pero sin prisa: cada barrido se
    procesa completo y en el instante correcto. Reescribe el CSV.
    """
    global x_hat, P, x_odo, x_lid, scan_anterior, ref_cache, pose_ref, pose_ref_lid
    global reg_x, reg_y, reg_full
    x_hat = [0.0, 0.0, 0.0]
    P = [row[:] for row in P0]
    x_odo = [0.0, 0.0, 0.0]
    x_lid = [0.0, 0.0, 0.0]
    scan_anterior = None
    ref_cache = None
    pose_ref = [0.0, 0.0, 0.0]
    pose_ref_lid = [0.0, 0.0, 0.0]
    for k in stats:
        stats[k] = 0
    reg_x = _nuevo_array()
    reg_y = _nuevo_array()
    reg_full = None
    gc.collect()
    reg_full = [_nuevo_array() for _ in COLUMNAS]
    guardar_punto(0.0, 0.0)
    n_b = len(barridos)
    j = 0
    t0 = time.ticks_ms()
    for k in range(len(rec_t)):
        dt = rec_dt[k]
        ds = rec_ds[k]
        dth = rec_dth[k]
        x_hat, P = kalman_prediccion(x_hat, P, ds / dt, dth / dt, dt)
        x_odo = mover(x_odo, ds, dth)
        codigo = 0
        # el barrido se aplica en el ciclo mas cercano a la mitad de su vuelta
        t_lim = rec_t[k] + 0.5 * (rec_t[k + 1] - rec_t[k] if k + 1 < len(rec_t) else dt)
        while j < n_b and barridos[j][0] <= t_lim:
            sc = barridos[j][1]
            j += 1
            if not usar_lidar:
                continue
            if scan_anterior is None:
                scan_anterior = sc
                pose_ref = x_hat[:]
                pose_ref_lid = x_lid[:]
                continue
            t_p = time.ticks_ms()
            res = iniciar_trabajo(sc)
            for _ in res["gen"]:
                pass
            res["ms"] = time.ticks_diff(time.ticks_ms(), t_p)
            codigo = terminar_trabajo(res)
            if j % 10 == 0:
                print("  barrido {}/{}  ({:.0f} s)".format(
                    j, n_b, time.ticks_diff(time.ticks_ms(), t0) / 1000))
                led(0, 0, 40 if (j // 10) % 2 else 0)
                gc.collect()
        guardar_punto(x_hat[0], x_hat[1])
        if k % LOG_COMPLETO_CADA == 0 or codigo:
            guardar_fila((rec_t[k], x_hat[0], x_hat[1], x_hat[2],
                          x_odo[0], x_odo[1], x_odo[2], x_lid[0], x_lid[1],
                          P[0][0], P[1][1], P[2][2], codigo, diag["n"] if codigo else 0,
                          diag["ms"] if codigo else 0, rec_izq[k], rec_der[k], 0.0))


def procesar_al_final():
    """Procesa los barridos guardados; si algo falla, deja la odometria."""
    if len(rec_t) == 0:
        return
    if USAR_LIDAR and len(barridos) >= 2:
        print("Robot detenido. Procesando {} barridos del LiDAR... (no lo apague)".format(
            len(barridos)))
        led(0, 0, 40)
        try:
            reprocesar(True)
            print("LiDAR procesado en {:.1f} s".format(stats["ms_total"] / 1000))
            return
        except (Exception, KeyboardInterrupt) as e:
            print("ATENCION: fallo el procesamiento del LiDAR ({}); se guarda la odometria".format(e))
            gc.collect()
    else:
        print("Sin barridos del LiDAR: se guarda la odometria (encoders + IMU)")
    reprocesar(False)


def main():
    global x_hat, P, x_odo, USAR_LIDAR
    global _control_activo, arranque_pendiente, _t_control_us
    iniciar_hardware()
    led(0, 0, 40)
    print("=== Reto 1 XRP: EKF encoders + IMU + LiDAR ===  version", VERSION_CODIGO)
    try:
        print("Bateria: {:.2f} V".format(board.get_battery_voltage()))
    except Exception:
        pass
    print("1) Ponga el robot con LOS DOS sensores sobre la linea transversal de salida.")
    print("2) Presione el boton de usuario y NO lo mueva: calibra y arranca solo.")
    board.wait_for_button()
    print("Calibrando IMU {} s...".format(T_CALIBRACION_IMU + 2))
    imu.calibrate(T_CALIBRACION_IMU)
    calibrar_bias_imu()
    imu.reset_yaw()
    calcular_v_omega(DT)                             # inicializa referencias

    # Esperar a que el LiDAR entregue datos (maximo 3 s)
    ultimo_scan = None
    t_ultimo_scan = time.ticks_ms()
    if USAR_LIDAR:
        t0 = time.ticks_ms()
        while ultimo_scan is None and time.ticks_diff(time.ticks_ms(), t0) < 3000:
            ultimo_scan = leer_lidar()
            time.sleep(0.02)
        if ultimo_scan is not None:
            print("LiDAR OK")
        else:
            print("ATENCION: sin datos del LiDAR, se usara solo encoders + IMU")
            USAR_LIDAR = False

    estado = "ESPERANDO_SALIDA"
    print("Esperando la linea transversal de salida bajo los dos sensores...")
    led(40, 30, 0)
    dist_recorrida = 0.0
    ciclo = 0
    t_ini = time.ticks_ms()
    t_diag = t_ini
    pose_ult_barrido = [0.0, 0.0, 0.0]
    t_ult_barrido = t_ini
    t_prev = time.ticks_us()
    gc.collect()
    try:
        gc.threshold(gc.mem_alloc() + gc.mem_free() // 4)
    except Exception:
        pass
    arranque_pendiente = False
    _control_activo = False
    _t_control_us = time.ticks_us()
    try:
        while True:
            atender()                           # ruedas (100 Hz) + bytes del LiDAR

            if estado == "ESPERANDO_SALIDA":
                if arranque_pendiente:
                    arranque_pendiente = False
                    iniciar_grabacion(ultimo_scan)
                    reiniciar_recorrido()
                    estado = "GRABANDO"
                    t_ini = time.ticks_ms()
                    pose_ult_barrido = [0.0, 0.0, 0.0]
                    t_ult_barrido = t_ini
                    if USAR_LIDAR:
                        solicitar_barrido()          # barrido de referencia en el origen
                    led(0, 40, 0)
                    print("Salida: x=0, y=0. Recta {} cm/s, curva {} cm/s".format(
                        V_RECTA_CM_S, V_CURVA_CM_S))
                    t_prev = time.ticks_us()
                    _t_control_us = t_prev           # no contar el tiempo del aviso
                    _control_activo = True
                elif time.ticks_diff(time.ticks_ms(), t_diag) >= 1000:
                    t_diag = time.ticks_ms()
                    print("ESPERA  izq={:.3f}  der={:.3f}  (salida: los dos >= {:.2f})".format(
                        izq_c, der_c, umbral_cruce()))
                time.sleep(0.001)
                continue

            # ---------------- GRABANDO ----------------
            if meta_detectada:
                print("META detectada a los {:.0f} cm".format(dist_recorrida))
                estado = "FIN"
                break
            t_now = time.ticks_us()
            dt = time.ticks_diff(t_now, t_prev) / 1e6
            if dt < DT:
                time.sleep(0.001)
                continue
            t_prev = t_now
            if dt > 1.0:
                dt = DT
            ciclo += 1
            izq = izq_c
            der = der_c

            v, omega = calcular_v_omega(dt)

            # 1) PREDICCION con encoders + IMU
            x_hat, P = kalman_prediccion(x_hat, P, v, omega, dt)
            x_odo = mover(x_odo, v * dt, omega * dt)
            dist_recorrida += abs(v * dt)
            atender()

            # 2) LiDAR
            lidar_vivo = time.ticks_diff(time.ticks_ms(), t_ultimo_scan) < LIDAR_TIMEOUT_MS + 2000
            if USAR_LIDAR:
                # solo se GUARDA: nada de calculos mientras el robot anda
                guardar_ciclo(time.ticks_diff(time.ticks_ms(), t_ini) / 1000.0,
                              dt, v * dt, omega * dt, izq, der)
                sc = tomar_barrido()
                if sc is not None and guardar_barrido(
                        sc[0], time.ticks_diff(sc[1], t_ini) / 1000.0):
                    t_ultimo_scan = time.ticks_ms()
                    pose_ult_barrido = x_hat[:]
                    t_ult_barrido = time.ticks_ms()
                mov = relativa(pose_ult_barrido, x_hat)
                if (_cap_estado == 0 and _barrido_listo is None and
                        (mov[0] * mov[0] + mov[1] * mov[1] > (BARRIDO_CADA_CM * _factor_barridos) ** 2 or
                         abs(mov[2]) > BARRIDO_CADA_DEG * _DEG or
                         time.ticks_diff(time.ticks_ms(), t_ult_barrido) > BARRIDO_CADA_S * 1000 * _factor_barridos)):
                    solicitar_barrido()
            atender()
            if ciclo % 20 == 0:
                gc.collect()                         # evita fragmentar la RAM
                servir_control()
                led(0, 40, 0) if (lidar_vivo or not USAR_LIDAR) else led(40, 0, 40)

            guardar_punto(x_hat[0], x_hat[1])
            if not USAR_LIDAR and ciclo % LOG_COMPLETO_CADA == 0:
                guardar_fila((time.ticks_diff(time.ticks_ms(), t_ini) / 1000.0,
                              x_hat[0], x_hat[1], x_hat[2],
                              x_odo[0], x_odo[1], x_odo[2], x_lid[0], x_lid[1],
                              P[0][0], P[1][1], P[2][2], 0, 0, 0, izq, der, 0.0))
    except KeyboardInterrupt:
        print("Detenido por el usuario")
    except Exception as e:
        print("PARADA POR ERROR:", repr(e))
        raise
    finally:
        # Primero detener las ruedas; despues el LiDAR, los calculos y los archivos
        _control_activo = False
        try:
            parar_ruedas()
        except Exception:
            pass
        parar_motores()
        if lidar_uart is not None and LIDAR_PROTOCOLO == "ld06":
            detener_lidar()
        led(40, 0, 0)
        if estado != "ESPERANDO_SALIDA":
            if USAR_LIDAR and len(rec_t) > 0:
                # ultimo tramo (frenada) para que el punto final quede en el CSV
                try:
                    dtf = time.ticks_diff(time.ticks_us(), t_prev) / 1e6
                    if 0 < dtf < 1.0:
                        v, omega = calcular_v_omega(dtf)
                        guardar_ciclo(time.ticks_diff(time.ticks_ms(), t_ini) / 1000.0,
                                      dtf, v * dtf, omega * dtf, izq_c, der_c)
                except Exception:
                    pass
                procesar_al_final()
            if len(reg_x) > 0:
                escribir_archivos()
                resumen(dist_recorrida)
        else:
            print("No hubo recorrido: no se tocan los archivos anteriores.")
        led(0, 0, 0)


def resumen(dist):
    """Metricas para el informe."""
    def cierre(p):
        return math.sqrt(p[0] ** 2 + p[1] ** 2)
    print("---------------- RESUMEN ----------------")
    print("Sensor izq: blanco {:.3f} negro {:.3f} | sensor der: blanco {:.3f} negro {:.3f}".format(
        _min_l, _max_l, _min_r, _max_r))
    if abs(_min_l - WHITE_LEFT) > 0.06 or abs(_min_r - WHITE_RIGHT) > 0.06:
        print("  OJO: el blanco medido no coincide con WHITE_LEFT / WHITE_RIGHT ({:.3f} / {:.3f}).".format(
            WHITE_LEFT, WHITE_RIGHT))
        print("  Si el robot siguio mal la linea, ponga esos valores medidos en la configuracion.")
    if _max_l < CROSS_RAW_LEFT + 0.02 or _max_r < CROSS_RAW_RIGHT + 0.02:
        print("  OJO: el negro medido queda muy cerca del umbral de cruce (CROSS_RAW_*).")
    print("Distancia recorrida: {:.1f} cm".format(dist))
    print("Posicion final EKF: x = {:.1f} cm, y = {:.1f} cm, rumbo {:.1f} grados".format(
        x_hat[0], x_hat[1], x_hat[2] / _DEG))
    print("(Si la pista es CERRADA, la distancia al origen es el error de cierre:)")
    print("  EKF {:.2f} cm | solo odometria {:.2f} cm".format(cierre(x_hat), cierre(x_odo)))
    if stats["scans"]:
        print("  solo LiDAR {:.2f} cm".format(cierre(x_lid)))
    else:
        print("LiDAR: no se uso en esta vuelta (solo encoders + IMU)")
    n = stats["scans"]
    if n:
        print("Barridos: {}  aceptados: {} (atenuados {})  rechazados: {}  fallo ICP: {}".format(
            n, stats["aceptadas"], stats["atenuadas"], stats["rech_gate"], stats["rech_icp"]))
        print("Tiempo medio scan matching: {:.1f} ms".format(stats["ms_total"] / n))
    print("P final diag: {:.3f} cm2, {:.3f} cm2, {:.6f} rad2".format(P[0][0], P[1][1], P[2][2]))
    print("Marcha: {:.1f} s, mayor pausa del control {:.0f} ms".format(tiempo_marcha, max_gap_s * 1000))


# Se ejecuta al correr el archivo; NO al importarlo desde las pruebas/simulador.
if __name__ != "reto1_xrp":
    main()
