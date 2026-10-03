/* =============================================================================
 *  Simulador del Reto 1 - XRP
 *  Puerto a JavaScript de reto1_xrp.py (v14): seguidor de linea + PI por rueda
 *  (100 Hz), odometria encoders + giroscopio, PL-ICP con LiDAR 360 y EKF (10 Hz).
 *  El "mundo" (motores, encoders, giroscopio, reflectancia, LiDAR) es un modelo
 *  fisico con ruido; el "robot" solo ve lo que veria la XRP real.
 *  Unidades: cm, s, rad. Marco del mundo: x a la derecha, y hacia arriba.
 * ===========================================================================*/
(function (G) {
  'use strict';

  // ---------------------------------------------------------------------------
  //  1. CONFIGURACION (copiada de reto1_xrp.py)
  // ---------------------------------------------------------------------------
  const C = {
    RADIO_RUEDA: 3.0, TRACK_WIDTH: 15.5, TICKS_POR_REV: 585, DT: 0.10,
    LOOP_DT: 0.010, MAX_CONTROL_GAP_S: 0.120, RADIO_CONTROL: 2.9,
    WHITE_LEFT: 0.5618, WHITE_RIGHT: 0.5082, BLACK_LEFT: 0.8725, BLACK_RIGHT: 0.8620,
    CROSS_RAW_LEFT: 0.80, CROSS_RAW_RIGHT: 0.80, SENSOR_SAMPLES: 3,
    SIGNAL_ON: 0.25, SIGNAL_OFF: 0.12, ERROR_DEADBAND: 0.035,
    V_RECTA_CM_S: 6.0, V_CURVA_CM_S: 3.5, V_CRUCE_CM_S: 5.0, V_PERDIDA_CM_S: 3.5,
    V_RUEDA_RECUPERACION_CM_S: 8.0, V_RUEDA_MAX_CM_S: 18.0, ACEL_AVANCE_CM_S2: 12.0,
    KP_LINEA: 0.55, KD_LINEA: 0.010, DERIVATIVE_FILTER_S: 0.040, LINE_ERROR_FILTER_S: 0.15,
    OMEGA_LINEA_MAX: 0.35, YAW_DAMPING: 0.18, STEERING_SLEW_RAD_S2: 0.65,
    CURVA_ENTER_ERROR: 0.14, CURVA_EXIT_ERROR: 0.08, MEMORIA_CENTRADO_S: 0.25,
    MIN_INNER_WHEEL_RATIO: 0.25, IMU_FILTER_S: 0.08,
    KS_L: 0.32, KS_R: 0.40, KV_L: 0.022, KV_R: 0.030, KP_RUEDA: 0.025, KI_RUEDA: 0.14,
    INTEGRAL_MAX: 0.45, MAX_EFFORT: 0.85, EFFORT_RISE_PER_S: 3.0,
    VEL_WINDOW_S: 0.030, VEL_FILTER_S: 0.020, ZERO_SPEED_CM_S: 0.12,
    REVERSAL_BRAKE_S: 0.04, REVERSAL_MAX_S: 0.20,
    ATASCO_VEL_CM_S: 0.30, ATASCO_BOOST_AFTER_S: 0.12, ATASCO_BOOST_MAX: 0.18,
    START_CONFIRM_S: 0.04, CROSS_CONFIRM_S: 0.06, CROSS_CLEAR_S: 0.25,
    MIN_FIN_DIST_CM: 20.0, MIN_FIN_TIME_S: 3.0,
    // LiDAR
    N_ANGULOS: 360, D_MIN_MM: 120, D_MAX_MM: 6000,
    BARRIDO_CADA_CM: 5.0, BARRIDO_CADA_DEG: 10.0, BARRIDO_CADA_S: 1.5,
    KEYFRAME_DIST: 12.0, KEYFRAME_ANG: 12.0, PASO_GRADOS: 4, ICP_ITER: 5,
    ICP_UMBRALES: [25.0, 15.0, 10.0, 6.0, 5.0, 5.0], ICP_MIN_PUNTOS: 30,
    ICP_MIN_INLIERS: 0.35, ICP_RMS_MAX: 4.0, ICP_COV_ESCALA: 4.0,
    MAX_SALTO_CM: 15.0, MAX_SALTO_DEG: 20.0,
    // EKF
    K_S: 0.02, K_G: 3e-3, K_ESC: 0.02,
    GATE_CHI2: 11.34, GATE_RECHAZO: 50.0,
  };
  const P0 = [[0.01, 0, 0], [0, 0.01, 0], [0, 0, 1e-4]];
  const QMIN = [[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-7]];
  const RMIN = [[0.05, 0, 0], [0, 0.05, 0], [0, 0, 3e-5]];
  const DEG = Math.PI / 180;
  const CIRC_CONTROL = 2 * Math.PI * C.RADIO_CONTROL;

  // Geometria fisica del robot (no la conoce el programa del robot)
  const GEO = {
    sensorAdelante: 7.5,   // cm desde el eje de las ruedas
    sensorSeparacion: 2.0, // cm entre los dos sensores de reflectancia
    sensorRadio: 0.6,      // cm, huella del sensor
    anchoLinea: 1.9,       // cm, linea principal
    anchoTransversal: 1.6, // cm, linea transversal (menor que la separacion)
    largoTransversal: 9.0, // cm
  };

  // ---------------------------------------------------------------------------
  //  2. UTILIDADES
  // ---------------------------------------------------------------------------
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  function wrap(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
  }
  // generador pseudoaleatorio reproducible (mulberry32) + gaussiana
  function rng(seed) {
    let t = seed >>> 0;
    const u = () => {
      t += 0x6D2B79F5;
      let r = Math.imul(t ^ (t >>> 15), 1 | t);
      r ^= r + Math.imul(r ^ (r >>> 7), 61 | r);
      return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
    };
    let spare = null;
    u.gauss = () => {
      if (spare !== null) { const s = spare; spare = null; return s; }
      let a, b, s;
      do { a = u() * 2 - 1; b = u() * 2 - 1; s = a * a + b * b; } while (s >= 1 || s === 0);
      const m = Math.sqrt(-2 * Math.log(s) / s);
      spare = b * m; return a * m;
    };
    return u;
  }
  const matMul = (A, B) => [0, 1, 2].map(i => [0, 1, 2].map(j => A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]));
  const matT = A => [0, 1, 2].map(i => [0, 1, 2].map(j => A[j][i]));
  const matAdd = (A, B) => [0, 1, 2].map(i => [0, 1, 2].map(j => A[i][j] + B[i][j]));
  const matSub = (A, B) => [0, 1, 2].map(i => [0, 1, 2].map(j => A[i][j] - B[i][j]));
  const matVec = (A, v) => [0, 1, 2].map(i => A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2]);
  const simetrizar = A => [0, 1, 2].map(i => [0, 1, 2].map(j => 0.5 * (A[i][j] + A[j][i])));
  const I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  function matInv3(m) {
    const [a, b, c] = m[0], [d, e, f] = m[1], [g, h, i] = m[2];
    const A = e * i - f * h, B = -(d * i - f * g), Cc = d * h - e * g;
    const det = a * A + b * B + c * Cc;
    if (Math.abs(det) < 1e-15) return null;
    const k = 1 / det;
    return [[A * k, -(b * i - c * h) * k, (b * f - c * e) * k],
            [B * k, (a * i - c * g) * k, -(a * f - c * d) * k],
            [Cc * k, -(a * h - b * g) * k, (a * e - b * d) * k]];
  }
  function componer(p, d) {
    const c = Math.cos(p[2]), s = Math.sin(p[2]);
    return [p[0] + c * d[0] - s * d[1], p[1] + s * d[0] + c * d[1], wrap(p[2] + d[2])];
  }
  function relativa(a, b) {
    const c = Math.cos(a[2]), s = Math.sin(a[2]);
    const dx = b[0] - a[0], dy = b[1] - a[1];
    return [c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])];
  }
  const inversa = d => relativa(d, [0, 0, 0]);
  function rotarCov(R, th) {
    const c = Math.cos(th), s = Math.sin(th);
    const J = [[c, -s, 0], [s, c, 0], [0, 0, 1]];
    return matMul(matMul(J, R), matT(J));
  }

  // ---------------------------------------------------------------------------
  //  3. MUNDO: pista, entorno del LiDAR
  // ---------------------------------------------------------------------------
  class Pista {
    constructor(pts, start, heading) {
      this.pts = pts;
      this.start = start;
      this.heading = heading;
      // rejilla de segmentos para distancia rapida
      this.cell = 4;
      this.grid = new Map();
      const n = pts.length;
      for (let i = 0; i < n; i++) {
        const a = pts[i], b = pts[(i + 1) % n];
        const x0 = Math.floor((Math.min(a[0], b[0]) - 3) / this.cell), x1 = Math.floor((Math.max(a[0], b[0]) + 3) / this.cell);
        const y0 = Math.floor((Math.min(a[1], b[1]) - 3) / this.cell), y1 = Math.floor((Math.max(a[1], b[1]) + 3) / this.cell);
        for (let gx = x0; gx <= x1; gx++) for (let gy = y0; gy <= y1; gy++) {
          const k = gx * 100003 + gy;
          if (!this.grid.has(k)) this.grid.set(k, []);
          this.grid.get(k).push(i);
        }
      }
      // linea transversal: perpendicular a la pista en el punto de salida
      const hx = Math.cos(heading), hy = Math.sin(heading);
      this.tc = start; this.tdir = [-hy, hx]; this.tn = [hx, hy];
      // longitud acumulada
      this.s = [0];
      for (let i = 1; i <= n; i++) {
        const a = pts[i - 1], b = pts[i % n];
        this.s.push(this.s[i - 1] + Math.hypot(b[0] - a[0], b[1] - a[1]));
      }
      this.largo = this.s[n];
    }
    distancia(x, y) {
      const k = Math.floor(x / this.cell) * 100003 + Math.floor(y / this.cell);
      const segs = this.grid.get(k);
      if (!segs) return 99;
      let best = 99;
      const n = this.pts.length;
      for (const i of segs) {
        const a = this.pts[i], b = this.pts[(i + 1) % n];
        const vx = b[0] - a[0], vy = b[1] - a[1];
        const L2 = vx * vx + vy * vy || 1e-9;
        let t = ((x - a[0]) * vx + (y - a[1]) * vy) / L2;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const dx = x - a[0] - t * vx, dy = y - a[1] - t * vy;
        const d = Math.sqrt(dx * dx + dy * dy);
        if (d < best) best = d;
      }
      return best;
    }
    // indice del punto de la pista mas cercano (para el error)
    masCercano(x, y) {
      let best = 1e9, bi = 0;
      for (let i = 0; i < this.pts.length; i += 2) {
        const p = this.pts[i]; const d = (p[0] - x) ** 2 + (p[1] - y) ** 2;
        if (d < best) { best = d; bi = i; }
      }
      return { i: bi, d: Math.sqrt(best) };
    }
    // cobertura de negro (0..1) de un sensor circular de radio r en (x, y)
    cobertura(x, y) {
      const r = GEO.sensorRadio;
      const d = this.distancia(x, y);
      let c = clamp((GEO.anchoLinea / 2 + r - d) / (2 * r), 0, 1);
      // transversal
      const rx = x - this.tc[0], ry = y - this.tc[1];
      const along = rx * this.tdir[0] + ry * this.tdir[1];
      const across = rx * this.tn[0] + ry * this.tn[1];
      if (Math.abs(along) < GEO.largoTransversal / 2) {
        const ct = clamp((GEO.anchoTransversal / 2 + r - Math.abs(across)) / (2 * r), 0, 1);
        if (ct > c) c = ct;
      }
      return c;
    }
  }

  // Entorno que "ve" el LiDAR (a 10 cm del piso): paredes y objetos
  function crearEntorno(tipo) {
    const seg = [];
    const objetos = [];
    const rect = (x0, y0, x1, y1, t) => {
      seg.push({ a: [x0, y0], b: [x1, y0], tipo: t }, { a: [x1, y0], b: [x1, y1], tipo: t },
               { a: [x1, y1], b: [x0, y1], tipo: t }, { a: [x0, y1], b: [x0, y0], tipo: t });
    };
    const caja = (x, y, w, h, t, extra) => { rect(x, y, x + w, y + h, t); objetos.push(Object.assign({ tipo: t, x, y, w, h }, extra || {})); };
    let muros;
    if (tipo === 'abierto') {
      // espacio abierto como en la prueba real: muros lejanos (varios casi fuera de rango)
      muros = [-380, -320, 520, 600];
      seg.push({ a: [-380, -320], b: [520, -320], tipo: 'muro' });
      seg.push({ a: [520, -320], b: [520, 600], tipo: 'muro' });
      seg.push({ a: [-380, 600], b: [-380, -320], tipo: 'muro' });
      caja(118, 120, 20, 15, 'caja');
      caja(-52, 20, 26, 18, 'morral');
      caja(18, -32, 11, 7, 'zapato'); caja(46, -33, 11, 7, 'zapato');
    } else {
      // salon: muros cerca de la lona, columna, caja del kit, banco, morral y un taburete
      muros = [-42, -48, 142, 248];
      seg.push({ a: [-42, -48], b: [142, -48], tipo: 'muro' });
      seg.push({ a: [142, -48], b: [142, 150], tipo: 'muro' });
      seg.push({ a: [142, 178], b: [142, 248], tipo: 'muro' });
      seg.push({ a: [142, 248], b: [-42, 248], tipo: 'muro' });
      seg.push({ a: [-42, 248], b: [-42, -48], tipo: 'muro' });
      caja(-42, 168, 14, 40, 'columna');
      caja(116, 118, 20, 15, 'caja');
      caja(108, 214, 32, 32, 'banco');
      caja(-36, 18, 22, 16, 'morral');
      caja(36, -46, 64, 10, 'mueble');
      caja(-34, 96, 6, 6, 'pata'); caja(-34, 120, 6, 6, 'pata'); caja(-20, 96, 6, 6, 'pata'); caja(-20, 120, 6, 6, 'pata');
    }
    return { seg, objetos, muros, tipo };
  }

  // ---------------------------------------------------------------------------
  //  4. MODELOS DE SENSORES Y ACTUADORES (el "hardware")
  // ---------------------------------------------------------------------------
  class Motor {
    // esfuerzo [-1,1] -> velocidad de la rueda (cm/s), primer orden + friccion
    constructor(ks, kv, r) {
      this.ks = ks; this.kv = kv; this.v = 0; this.effort = 0; this.freno = false;
      this.angulo = 0; this.r = r;
    }
    paso(dt, bateria) {
      const u = this.effort * bateria;
      let vss = 0;
      if (Math.abs(u) > this.ks) vss = Math.sign(u) * (Math.abs(u) - this.ks) / this.kv;
      const tau = (this.freno && this.effort === 0) ? 0.02 : 0.065;
      this.v += (vss - this.v) * (1 - Math.exp(-dt / tau));
      this.angulo += this.v * dt / this.r;
    }
  }

  // ---------------------------------------------------------------------------
  //  5. EL SIMULADOR
  // ---------------------------------------------------------------------------
  class Sim {
    constructor(data, opts) {
      this.data = data;
      this.pista = new Pista(data.track, data.start, data.startHeading);
      this.opts = Object.assign({ ruido: 1.0, lidar: true, seed: 7, entorno: 'salon', persona: false }, opts || {});
      this.entorno = crearEntorno(this.opts.entorno);
      this.reset();
    }

    reset(seed) {
      const o = this.opts;
      if (seed !== undefined) o.seed = seed;
      const R = rng(o.seed);
      this.R = R;
      const k = o.ruido;
      // --- hardware "real" (diferente de lo que cree el programa) ---
      this.hw = {
        radioReal: 3.0 * (1 + 0.03 * k + 0.01 * k * R.gauss()), // radio efectivo de la rueda
        trackReal: 15.5 * (1 + 0.01 * k * R.gauss()),
        escalaGiro: 1 + 0.015 * k * R.gauss(),
        sesgoGiro: 0.04 * k * R.gauss() * DEG,  // rad/s residual
        ruidoGiro: 0.25 * k * DEG,             // rad/s por muestra
        bateria: 1.0,
        deslizamiento: 0.01 * k,
      };
      this.motorL = new Motor(C.KS_L * (1 + 0.06 * k * R.gauss()), C.KV_L * (1 + 0.06 * k * R.gauss()), C.RADIO_CONTROL);
      this.motorR = new Motor(C.KS_R * (1 + 0.06 * k * R.gauss()), C.KV_R * (1 + 0.06 * k * R.gauss()), C.RADIO_CONTROL);
      // pose real: los dos sensores sobre la transversal
      const h = this.data.startHeading;
      this.real = [this.pista.start[0] - GEO.sensorAdelante * Math.cos(h),
                   this.pista.start[1] - GEO.sensorAdelante * Math.sin(h), h];
      this.realV = 0; this.realW = 0;
      this.yawImu = 0;           // grados, integrado por la "IMU"
      this.t = 0;
      this.distReal = 0;

      // --- estado del programa del robot ---
      this.estado = 'ESPERANDO_SALIDA';
      this.ctl = {
        activo: false, tiempoCruce: 0, tiempoFueraCruce: 0, tiempoMarcha: 0, dist: 0,
        metaHabilitada: false, metaDetectada: false, arranquePendiente: false,
        omegaFiltrada: 0, yawControl: 0, izq: 0, der: 0, tAcum: 0,
      };
      this.ruedaL = this._nuevaRueda(this.motorL, C.KS_L, C.KV_L);
      this.ruedaR = this._nuevaRueda(this.motorR, C.KS_R, C.KV_R);
      this.seg = this._nuevoSeguidor();
      this.est = {
        xHat: [0, 0, 0], P: P0.map(r => r.slice()), xOdo: [0, 0, 0], xLid: [0, 0, 0],
        encLPrev: 0, encRPrev: 0, yawPrev: 0, tAcum: 0,
        scanRef: null, refCache: null, poseRef: [0, 0, 0], poseRefLid: [0, 0, 0],
        poseUltBarrido: [0, 0, 0], tUltBarrido: 0, distRecorrida: 0,
      };
      this.stats = { scans: 0, aceptadas: 0, atenuadas: 0, rechGate: 0, rechIcp: 0 };
      this.diag = { ok: false, n: 0, rms: 0, d2: 0, R: null };
      this.trail = { real: [], ekf: [], odo: [], t: [] };
      this.ultimoScan = null;   // para dibujar
      this.scanPose = null;
      this.lidarAng = 0;
      this.saltos = [];
      this.eventos = [];
      this.fin = null;
      this._origen = null;      // pose real en el origen
      this.anim = { lean: 0, leanV: 0, aPrev: 0, polvo: [] };
    }

    // ---------- sensores (lo que lee la XRP) ----------
    leerReflectancia() {
      const [x, y, th] = this.real;
      const c = Math.cos(th), s = Math.sin(th);
      const fx = x + GEO.sensorAdelante * c, fy = y + GEO.sensorAdelante * s;
      const half = GEO.sensorSeparacion / 2;
      const lx = fx - half * s, ly = fy + half * c;   // izquierda
      const rx = fx + half * s, ry = fy - half * c;   // derecha
      const k = this.opts.ruido;
      const cl = this.pista.cobertura(lx, ly), cr = this.pista.cobertura(rx, ry);
      const l = C.WHITE_LEFT + (C.BLACK_LEFT - C.WHITE_LEFT) * cl + 0.006 * k * this.R.gauss();
      const r = C.WHITE_RIGHT + (C.BLACK_RIGHT - C.WHITE_RIGHT) * cr + 0.006 * k * this.R.gauss();
      this._sens = { lx, ly, rx, ry, cl, cr };
      return [clamp(l, 0, 1), clamp(r, 0, 1)];
    }
    getPosition(m) { return Math.floor(m.angulo / (2 * Math.PI) * C.TICKS_POR_REV) / C.TICKS_POR_REV; }
    getCounts(m) { return Math.floor(m.angulo / (2 * Math.PI) * C.TICKS_POR_REV); }
    gyroRate() { // rad/s medidos (ya sin el sesgo calibrado)
      return this.realW * this.hw.escalaGiro + this.hw.sesgoGiro + this.hw.ruidoGiro * this.R.gauss();
    }

    // ---------- PI de velocidad por rueda (ControlRueda) ----------
    _nuevaRueda(motor, ks, kv) {
      return { motor, ks, kv, posicion: 0, velocidad: 0, distVentana: 0, tVentana: 0, integral: 0,
               esfuerzo: 0, direccion: 0, invirtiendo: false, tInversion: 0, tAtasco: 0 };
    }
    _medir(w, dt) {
      const pos = this.getPosition(w.motor);
      const delta = (pos - w.posicion) * CIRC_CONTROL;
      w.posicion = pos;
      w.distVentana += delta; w.tVentana += dt;
      if (w.tVentana >= C.VEL_WINDOW_S) {
        const medida = w.distVentana / w.tVentana;
        const alpha = w.tVentana / (C.VEL_FILTER_S + w.tVentana);
        w.velocidad += alpha * (medida - w.velocidad);
        w.distVentana = 0; w.tVentana = 0;
      }
      return delta;
    }
    _parar(w) {
      w.integral = 0; w.esfuerzo = 0; w.direccion = 0; w.invirtiendo = false; w.tAtasco = 0;
      w.motor.effort = 0; w.motor.freno = true;
    }
    _actualizarRueda(w, objetivo, dt) {
      objetivo = clamp(objetivo, -C.V_RUEDA_MAX_CM_S, C.V_RUEDA_MAX_CM_S);
      if (Math.abs(objetivo) < C.ZERO_SPEED_CM_S) { this._parar(w); return; }
      const direccion = objetivo > 0 ? 1 : -1;
      if (direccion !== w.direccion) {
        w.integral = 0; w.tAtasco = 0;
        w.invirtiendo = (w.direccion !== 0 || direccion * w.velocidad < -0.5);
        w.tInversion = 0; w.direccion = direccion;
      }
      if (w.invirtiendo) {
        w.tInversion += dt;
        if (w.tInversion < C.REVERSAL_BRAKE_S || (Math.abs(w.velocidad) > 0.7 && w.tInversion < C.REVERSAL_MAX_S)) {
          w.esfuerzo = 0; w.motor.effort = 0; w.motor.freno = true; return;
        }
        w.invirtiendo = false;
      }
      const velDir = w.velocidad * direccion;
      const error = Math.abs(objetivo) - velDir;
      if (Math.abs(objetivo) >= 0.6 && Math.abs(w.velocidad) < C.ATASCO_VEL_CM_S && Math.abs(w.esfuerzo) >= 0.24) w.tAtasco += dt;
      else w.tAtasco = Math.max(0, w.tAtasco - 2 * dt);
      const boost = C.ATASCO_BOOST_MAX * clamp((w.tAtasco - C.ATASCO_BOOST_AFTER_S) / 0.4, 0, 1);
      const ff = w.ks + w.kv * Math.abs(objetivo) + boost;
      const propI = clamp(w.integral + C.KI_RUEDA * error * dt, -C.INTEGRAL_MAX, C.INTEGRAL_MAX);
      const prop = ff + C.KP_RUEDA * error + propI;
      const techo = Math.min(C.MAX_EFFORT, Math.abs(w.esfuerzo) + C.EFFORT_RISE_PER_S * dt);
      if ((prop >= 0 && prop <= techo) || (prop > techo && error < 0) || (prop < 0 && error > 0)) w.integral = propI;
      const mag = clamp(ff + C.KP_RUEDA * error + w.integral, 0, techo);
      w.esfuerzo = direccion * mag;
      w.motor.freno = mag === 0;
      w.motor.effort = w.esfuerzo;
    }

    // ---------- SeguidorLinea ----------
    _nuevoSeguidor() {
      return { estado: 'RECTA', error: 0, derivada: 0, avance: 0, omega: 0, presente: true,
               enCurva: false, recuperando: false, tRec: 0, ladoMem: 0, errorMem: 0, curvMem: 0, tCentrado: 0 };
    }
    _ruedas(avance, omega) {
      const rel = (1 - C.MIN_INNER_WHEEL_RATIO) / (1 + C.MIN_INNER_WHEEL_RATIO);
      const lim = 2 * avance * rel / C.TRACK_WIDTH;
      omega = clamp(omega, -lim, lim);
      return [avance - omega * C.TRACK_WIDTH / 2, avance + omega * C.TRACK_WIDTH / 2];
    }
    _seguidor(left, right, dt, omegaMedida) {
      const S = this.seg;
      dt = clamp(dt, 0.001, C.MAX_CONTROL_GAP_S);
      const sl = clamp((left - C.WHITE_LEFT) / (C.BLACK_LEFT - C.WHITE_LEFT), 0, 1);
      const sr = clamp((right - C.WHITE_RIGHT) / (C.BLACK_RIGHT - C.WHITE_RIGHT), 0, 1);
      const intensidad = Math.max(sl, sr);
      if (intensidad >= C.SIGNAL_ON) S.presente = true;
      else if (intensidad <= C.SIGNAL_OFF) S.presente = false;
      const cruce = left >= C.CROSS_RAW_LEFT && right >= C.CROSS_RAW_RIGHT;
      let avanceObj, omegaObj;
      if (cruce) {
        S.estado = 'CRUCE'; S.recuperando = S.enCurva = false; S.tRec = 0;
        S.error = S.derivada = 0; S.curvMem = S.errorMem = 0; S.ladoMem = 0; S.tCentrado = 0;
        avanceObj = C.V_CRUCE_CM_S; omegaObj = -C.YAW_DAMPING * omegaMedida;
      } else if (!S.presente) {
        S.recuperando = true; S.tRec += dt; S.derivada = 0; S.error = S.errorMem;
        let izq, der;
        if (S.ladoMem) {
          S.estado = S.ladoMem > 0 ? 'RECUPERAR_IZQ' : 'RECUPERAR_DER';
          S.enCurva = true;
          const v = C.V_RUEDA_RECUPERACION_CM_S;
          izq = S.ladoMem < 0 ? v : 0; der = S.ladoMem > 0 ? v : 0;
          S.avance = v * 0.5; S.omega = (der - izq) / C.TRACK_WIDTH;
        } else {
          S.estado = 'SIN_LINEA'; S.enCurva = false;
          izq = der = C.V_PERDIDA_CM_S; S.avance = C.V_PERDIDA_CM_S; S.omega = 0;
        }
        return [izq, der];
      } else {
        if (S.recuperando) S.derivada = 0;
        S.recuperando = false; S.tRec = 0;
        const errRaw = (sl - sr) / Math.max(sl + sr, 1.0);
        if (Math.abs(errRaw) >= C.CURVA_ENTER_ERROR) { S.ladoMem = errRaw > 0 ? 1 : -1; S.errorMem = errRaw; }
        const anterior = S.error;
        S.error += dt / (C.LINE_ERROR_FILTER_S + dt) * (errRaw - S.error);
        const error = Math.abs(S.error) <= C.ERROR_DEADBAND ? 0 : S.error;
        const der = (S.error - anterior) / dt;
        S.derivada += dt / (C.DERIVATIVE_FILTER_S + dt) * (der - S.derivada);
        S.enCurva = Math.abs(error) > C.CURVA_ENTER_ERROR;
        S.estado = S.enCurva ? 'CURVA' : 'RECTA';
        avanceObj = C.V_RECTA_CM_S - (C.V_RECTA_CM_S - C.V_CURVA_CM_S) * Math.abs(error);
        omegaObj = C.KP_LINEA * error + clamp(C.KD_LINEA * S.derivada, -0.08, 0.08);
        omegaObj -= C.YAW_DAMPING * omegaMedida;
      }
      const paso = C.ACEL_AVANCE_CM_S2 * dt;
      S.avance += clamp(avanceObj - S.avance, -paso, paso);
      omegaObj = clamp(omegaObj, -C.OMEGA_LINEA_MAX, C.OMEGA_LINEA_MAX);
      const pg = C.STEERING_SLEW_RAD_S2 * dt;
      S.omega += clamp(omegaObj - S.omega, -pg, pg);
      const [izq, der] = this._ruedas(S.avance, S.omega);
      if (S.presente && !cruce) {
        if (Math.abs(S.error) >= C.CURVA_ENTER_ERROR) {
          S.tCentrado = 0;
          const wa = (der - izq) / C.TRACK_WIDTH;
          if (wa * S.error > 0 && S.avance > 0.1) { S.curvMem = wa / S.avance; S.errorMem = S.error; }
        } else if (Math.abs(S.error) <= C.CURVA_EXIT_ERROR) {
          S.tCentrado += dt;
          if (S.tCentrado >= C.MEMORIA_CENTRADO_S) S.curvMem = S.errorMem = 0;
        } else S.tCentrado = 0;
      }
      return [izq, der];
    }

    // ---------- servir_control (100 Hz) ----------
    _servirControl(dt) {
      const K = this.ctl;
      const dl = this._medir(this.ruedaL, dt), dr = this._medir(this.ruedaR, dt);
      const omegaEnc = (dr - dl) / (C.TRACK_WIDTH * dt);
      const omega = 0.8 * this.gyroRate() + 0.2 * omegaEnc;
      K.omegaFiltrada += dt / (C.IMU_FILTER_S + dt) * (omega - K.omegaFiltrada);
      K.yawControl += K.omegaFiltrada * dt;
      let sl = 0, sr = 0;
      for (let i = 0; i < C.SENSOR_SAMPLES; i++) { const [a, b] = this.leerReflectancia(); sl += a; sr += b; }
      K.izq = sl / C.SENSOR_SAMPLES; K.der = sr / C.SENSOR_SAMPLES;
      const esCruce = K.izq >= C.CROSS_RAW_LEFT && K.der >= C.CROSS_RAW_RIGHT;
      if (esCruce) { K.tiempoCruce += dt; K.tiempoFueraCruce = 0; }
      else { K.tiempoCruce = 0; K.tiempoFueraCruce += dt; }
      if (!K.activo) {
        this._parar(this.ruedaL); this._parar(this.ruedaR);
        if (K.tiempoCruce >= C.START_CONFIRM_S && !K.metaDetectada) K.arranquePendiente = true;
        return;
      }
      K.tiempoMarcha += dt;
      K.dist += Math.max(0, (dl + dr) * 0.5);
      if (K.tiempoFueraCruce >= C.CROSS_CLEAR_S && K.dist >= C.MIN_FIN_DIST_CM && K.tiempoMarcha >= C.MIN_FIN_TIME_S) K.metaHabilitada = true;
      if (K.metaHabilitada && K.tiempoCruce >= C.CROSS_CONFIRM_S) K.metaDetectada = true;
      if (K.metaDetectada) { this._parar(this.ruedaL); this._parar(this.ruedaR); return; }
      const [ol, or] = this._seguidor(K.izq, K.der, dt, K.omegaFiltrada);
      this.objL = ol; this.objR = or;
      this._actualizarRueda(this.ruedaL, ol, dt);
      this._actualizarRueda(this.ruedaR, or, dt);
    }

    // ---------- odometria + EKF (10 Hz) ----------
    _calcularVOmega(dt) {
      const E = this.est;
      const eL = this.getCounts(this.motorL), eR = this.getCounts(this.motorR);
      const yaw = this.yawImu;
      const k = 2 * Math.PI * C.RADIO_RUEDA / C.TICKS_POR_REV;
      const dL = (eL - E.encLPrev) * k, dR = (eR - E.encRPrev) * k;
      E.encLPrev = eL; E.encRPrev = eR;
      const dyaw = (yaw - E.yawPrev) * DEG; E.yawPrev = yaw;
      const ds = 0.5 * (dL + dR);
      return [ds / dt, dyaw / dt];
    }
    _calcularQ(ds, dth, dt, thm) {
      const c = Math.cos(thm), s = Math.sin(thm);
      const vs = C.K_S * Math.abs(ds) + 1e-6;
      const vt = C.K_G * dt + (C.K_ESC * dth) ** 2 + 1e-9;
      const Gm = [[c, -0.5 * ds * s], [s, 0.5 * ds * c], [0, 1]];
      const Q = [0, 1, 2].map(i => [0, 1, 2].map(j => Gm[i][0] * vs * Gm[j][0] + Gm[i][1] * vt * Gm[j][1]));
      return matAdd(Q, QMIN);
    }
    _prediccion(x, P, v, w, dt) {
      const ds = v * dt, dth = w * dt, thm = x[2] + 0.5 * dth;
      const c = Math.cos(thm), s = Math.sin(thm);
      const xp = [x[0] + ds * c, x[1] + ds * s, wrap(x[2] + dth)];
      const F = [[1, 0, -ds * s], [0, 1, ds * c], [0, 0, 1]];
      const Pp = matAdd(matMul(matMul(F, P), matT(F)), this._calcularQ(ds, dth, dt, thm));
      return [xp, simetrizar(Pp)];
    }
    _actualizacion(xp, Pp, z, Rk) {
      const y = [z[0] - xp[0], z[1] - xp[1], wrap(z[2] - xp[2])];
      let Si = matInv3(matAdd(Pp, Rk));
      if (!Si) { this.diag.d2 = -1; return [xp, Pp]; }
      const Siy = matVec(Si, y);
      const d2 = y[0] * Siy[0] + y[1] * Siy[1] + y[2] * Siy[2];
      this.diag.d2 = d2;
      if (d2 > C.GATE_CHI2) {
        const f = d2 / C.GATE_CHI2;
        if (f > C.GATE_RECHAZO) return [xp, Pp];
        Rk = Rk.map(r => r.map(v => v * f));
        Si = matInv3(matAdd(Pp, Rk));
        if (!Si) return [xp, Pp];
      }
      const K = matMul(Pp, Si);
      const Ky = matVec(K, y);
      const xn = [xp[0] + Ky[0], xp[1] + Ky[1], wrap(xp[2] + Ky[2])];
      const IK = matSub(I3, K);
      const Pn = matAdd(matMul(matMul(IK, Pp), matT(IK)), matMul(matMul(K, Rk), matT(K)));
      return [xn, simetrizar(Pn)];
    }

    // ---------- LiDAR ----------
    persona() {
      // persona que camina alrededor de la pista (obstaculo movil para el LiDAR)
      const t = this.t * 0.11;
      const cx = 50 + 78 * Math.cos(t), cy = 100 + 128 * Math.sin(t);
      const hx = -Math.sin(t), hy = Math.cos(t);
      const paso = Math.sin(this.t * 4.2) * 9;
      const nx = -hy, ny = hx;
      return { cx, cy, hx, hy, piernas: [[cx + nx * 9 + hx * paso, cy + ny * 9 + hy * paso], [cx - nx * 9 - hx * paso, cy - ny * 9 - hy * paso]] };
    }
    _segPersona() {
      if (!this.opts.persona) return [];
      const out = [];
      for (const [x, y] of this.persona().piernas) {
        const r = 6;
        for (let k = 0; k < 6; k++) {
          const a0 = k * Math.PI / 3, a1 = (k + 1) * Math.PI / 3;
          out.push({ a: [x + r * Math.cos(a0), y + r * Math.sin(a0)], b: [x + r * Math.cos(a1), y + r * Math.sin(a1)] });
        }
      }
      return out;
    }
    _rayo(ox, oy, dx, dy, extra) {
      let best = 1e9;
      for (const s of (extra ? this.entorno.seg.concat(extra) : this.entorno.seg)) {
        const ax = s.a[0], ay = s.a[1], bx = s.b[0] - ax, by = s.b[1] - ay;
        const den = dx * by - dy * bx;
        if (Math.abs(den) < 1e-12) continue;
        const t = ((ax - ox) * by - (ay - oy) * bx) / den;
        const u = ((ax - ox) * dy - (ay - oy) * dx) / den;
        if (t > 0 && u >= 0 && u <= 1 && t < best) best = t;
      }
      return best;
    }
    tomarBarrido() {
      const [x, y, th] = this.real;
      const scan = new Uint16Array(C.N_ANGULOS);
      const k = this.opts.ruido;
      const extra = this._segPersona();
      for (let i = 0; i < C.N_ANGULOS; i++) {
        const a = th + i * DEG;
        let d = this._rayo(x, y, Math.cos(a), Math.sin(a), extra.length ? extra : null); // cm
        if (d > 1200 || this.R() < 0.03 * k) { scan[i] = 0; continue; }
        d = d * 10 * (1 + 0.004 * k * this.R.gauss()) + 4 * k * this.R.gauss();
        scan[i] = Math.max(0, Math.round(d));
      }
      return scan;
    }
    _prepararReferencia(scan) {
      const N = C.N_ANGULOS;
      const rx = new Float32Array(N), ry = new Float32Array(N), nx = new Float32Array(N), ny = new Float32Array(N);
      const ok = new Uint8Array(N);
      for (let i = 0; i < N; i++) {
        const d = scan[i];
        if (d > C.D_MIN_MM && d < C.D_MAX_MM) { const dc = d * 0.1; rx[i] = dc * Math.cos(i * DEG); ry[i] = dc * Math.sin(i * DEG); ok[i] = 1; }
      }
      for (let i = 0; i < N; i++) {
        if (!ok[i]) continue;
        const a = (i - 1 + N) % N, b = (i + 1) % N;
        let tx, ty, esc;
        if (ok[a] && ok[b]) { tx = rx[b] - rx[a]; ty = ry[b] - ry[a]; esc = 2; }
        else if (ok[b]) { tx = rx[b] - rx[i]; ty = ry[b] - ry[i]; esc = 1; }
        else if (ok[a]) { tx = rx[i] - rx[a]; ty = ry[i] - ry[a]; esc = 1; }
        else continue;
        const Lt = Math.hypot(tx, ty);
        if (Lt < 1e-6 || Lt > esc * (0.06 * scan[i] * 0.1 + 3.0)) continue;
        nx[i] = -ty / Lt; ny[i] = tx / Lt; ok[i] = 2;
      }
      return { rx, ry, nx, ny, ok };
    }
    _icp(px, py, ref, guess) {
      const { rx, ry, nx, ny, ok } = ref;
      let [tx, ty, th] = guess;
      let Ai = null, n = 0, sr2 = 0;
      const N = C.N_ANGULOS, U = C.ICP_UMBRALES;
      for (let it = 0; it < C.ICP_ITER; it++) {
        const c = Math.cos(th), s = Math.sin(th);
        const u = U[Math.min(it, U.length - 1)], u2 = u * u;
        let A00 = 0, A01 = 0, A02 = 0, A11 = 0, A12 = 0, A22 = 0, b0 = 0, b1 = 0, b2 = 0;
        n = 0; sr2 = 0;
        for (let k = 0; k < px.length; k++) {
          const ax = c * px[k] - s * py[k], ay = s * px[k] + c * py[k];
          const qx = ax + tx, qy = ay + ty;
          const j0 = Math.round(Math.atan2(qy, qx) / DEG);
          let best = -1, bd = u2;
          for (let jj = j0 - 1; jj <= j0 + 1; jj++) {
            const j = ((jj % N) + N) % N;
            if (ok[j] === 2) {
              const ex = qx - rx[j], ey = qy - ry[j], d2 = ex * ex + ey * ey;
              if (d2 < bd) { bd = d2; best = j; }
            }
          }
          if (best < 0) continue;
          const n0 = nx[best], n1 = ny[best];
          const r = n0 * (qx - rx[best]) + n1 * (qy - ry[best]);
          const J2 = n1 * ax - n0 * ay;
          A00 += n0 * n0; A01 += n0 * n1; A02 += n0 * J2; A11 += n1 * n1; A12 += n1 * J2; A22 += J2 * J2;
          b0 += n0 * r; b1 += n1 * r; b2 += J2 * r; n++; sr2 += r * r;
        }
        if (n < C.ICP_MIN_PUNTOS) return null;
        const lam = 1e-6 * (A00 + A11 + A22) + 1e-9;
        Ai = matInv3([[A00 + lam, A01, A02], [A01, A11 + lam, A12], [A02, A12, A22 + lam]]);
        if (!Ai) return null;
        const d0 = -(Ai[0][0] * b0 + Ai[0][1] * b1 + Ai[0][2] * b2);
        const d1 = -(Ai[1][0] * b0 + Ai[1][1] * b1 + Ai[1][2] * b2);
        const d2_ = -(Ai[2][0] * b0 + Ai[2][1] * b1 + Ai[2][2] * b2);
        tx += d0; ty += d1; th += d2_;
        if (it >= 2 && Math.abs(d0) < 0.02 && Math.abs(d1) < 0.02 && Math.abs(d2_) < 0.0003) break;
      }
      const s2 = n > 3 ? sr2 / (n - 3) : sr2;
      const cov = Ai.map(r => r.map(v => v * s2));
      return { tx, ty, th: wrap(th), n, npts: px.length, rms: Math.sqrt(sr2 / n), cov };
    }
    _procesarBarrido(scan) {
      const E = this.est;
      if (!E.scanRef) { E.scanRef = scan; E.refCache = this._prepararReferencia(scan); E.poseRef = E.xHat.slice(); E.poseRefLid = E.xLid.slice(); return 0; }
      this.stats.scans++;
      const guess = relativa(E.poseRef, E.xHat);
      const px = [], py = [];
      for (let i = 0; i < C.N_ANGULOS; i += C.PASO_GRADOS) {
        const d = scan[i];
        if (d > C.D_MIN_MM && d < C.D_MAX_MM) { px.push(d * 0.1 * Math.cos(i * DEG)); py.push(d * 0.1 * Math.sin(i * DEG)); }
      }
      let out = null; this.diag.ok = false; this.diag.n = 0;
      if (px.length >= C.ICP_MIN_PUNTOS) {
        const icp = this._icp(px, py, E.refCache, guess);
        if (icp) {
          this.diag.n = icp.n; this.diag.rms = icp.rms;
          if (icp.n >= C.ICP_MIN_INLIERS * icp.npts && icp.rms <= C.ICP_RMS_MAX) {
            out = [icp.tx, icp.ty, icp.th];
            this.diag.R = matAdd(icp.cov.map(r => r.map(v => v * C.ICP_COV_ESCALA)), RMIN);
          }
        }
      }
      if (out && Math.abs(out[0] - guess[0]) < C.MAX_SALTO_CM && Math.abs(out[1] - guess[1]) < C.MAX_SALTO_CM &&
          Math.abs(wrap(out[2] - guess[2])) < C.MAX_SALTO_DEG * DEG) this.diag.ok = true;
      let codigo = 3, xLidScan;
      if (this.diag.ok) {
        const z = componer(E.poseRef, out);
        const Rg = rotarCov(this.diag.R, E.poseRef[2]);
        const antes = E.xHat.slice();
        [E.xHat, E.P] = this._actualizacion(E.xHat, E.P, z, Rg);
        const salto = Math.hypot(E.xHat[0] - antes[0], E.xHat[1] - antes[1]);
        if (salto > 0.8) this.saltos.push({ t: this.t, salto, p: E.xHat.slice() });
        if (this.diag.d2 >= 0 && this.diag.d2 <= C.GATE_RECHAZO * C.GATE_CHI2) {
          codigo = 1; this.stats.aceptadas++;
          if (this.diag.d2 > C.GATE_CHI2) this.stats.atenuadas++;
        } else { codigo = 2; this.stats.rechGate++; }
        xLidScan = componer(E.poseRefLid, out);
      } else {
        this.stats.rechIcp++;
        xLidScan = componer(E.poseRefLid, relativa(E.poseRef, E.xHat));
      }
      E.xLid = xLidScan;
      const mov = relativa(E.poseRef, E.xHat);
      if (codigo !== 1 || Math.hypot(mov[0], mov[1]) > C.KEYFRAME_DIST || Math.abs(mov[2]) > C.KEYFRAME_ANG * DEG) {
        E.scanRef = scan; E.refCache = this._prepararReferencia(scan); E.poseRef = E.xHat.slice(); E.poseRefLid = E.xLid.slice();
      }
      return codigo;
    }

    // ---------- integracion fisica ----------
    _fisica(dt) {
      const bat = this.hw.bateria;
      this.motorL.paso(dt, bat); this.motorR.paso(dt, bat);
      const k = this.opts.ruido;
      // velocidad sobre el piso (con deslizamiento) vs rotacion de la rueda (encoder)
      const sl = 1 - this.hw.deslizamiento * (1 + 0.5 * this.R.gauss()) * k;
      const sr = 1 - this.hw.deslizamiento * (1 + 0.5 * this.R.gauss()) * k;
      const r = this.hw.radioReal / C.RADIO_CONTROL;  // el motor modela la velocidad que mide el control (r = 2,9 cm)
      const vL = this.motorL.v * r * sl, vR = this.motorR.v * r * sr;
      const v = 0.5 * (vL + vR), w = (vR - vL) / this.hw.trackReal;
      const [x, y, th] = this.real;
      const thm = th + 0.5 * w * dt;
      this.real = [x + v * dt * Math.cos(thm), y + v * dt * Math.sin(thm), wrap(th + w * dt)];
      // aceleracion -> inclinacion del chasis (animacion procedural)
      const a = (v - this.realV) / dt;
      const A = this.anim;
      A.leanV += (-(a * 0.012) - A.lean * 120 - A.leanV * 14) * dt;
      A.lean += A.leanV * dt;
      this.realV = v; this.realW = w;
      this.distReal += Math.abs(v * dt);
      this.yawImu += (w * this.hw.escalaGiro + this.hw.sesgoGiro + this.hw.ruidoGiro * 0.2 * this.R.gauss()) * dt / DEG;
      this.lidarAng += 2 * Math.PI * 10 * dt;
    }

    // ---------- un paso de simulacion de 'dt' segundos ----------
    paso(dtTotal) {
      const h = 0.002;   // fisica a 500 Hz
      let n = Math.max(1, Math.round(dtTotal / h));
      while (n-- > 0) {
        this._fisica(h);
        this.t += h;
        this.ctl.tAcum += h;
        if (this.ctl.tAcum >= C.LOOP_DT - 1e-9) {
          const dtc = this.ctl.tAcum; this.ctl.tAcum = 0;
          this._servirControl(dtc);
          this._maquina();
        }
        if (this.estado === 'GRABANDO') {
          this.est.tAcum += h;
          if (this.est.tAcum >= C.DT - 1e-9) {
            const dte = this.est.tAcum; this.est.tAcum = 0;
            this._estimar(dte);
          }
        }
        if (this.estado === 'FIN') break;
      }
    }
    _maquina() {
      const K = this.ctl;
      if (this.estado === 'ESPERANDO_SALIDA' && K.arranquePendiente) {
        K.arranquePendiente = false;
        // iniciar_grabacion + reiniciar_recorrido
        const E = this.est;
        E.xHat = [0, 0, 0]; E.P = P0.map(r => r.slice()); E.xOdo = [0, 0, 0]; E.xLid = [0, 0, 0];
        this.yawImu = 0; E.yawPrev = 0;
        E.encLPrev = this.getCounts(this.motorL); E.encRPrev = this.getCounts(this.motorR);
        E.scanRef = null; E.refCache = null; E.poseRef = [0, 0, 0]; E.poseRefLid = [0, 0, 0];
        E.poseUltBarrido = [0, 0, 0]; E.tUltBarrido = this.t;
        K.tiempoMarcha = K.dist = K.yawControl = 0; K.metaHabilitada = K.metaDetectada = false;
        this.seg.estado = 'CRUCE';
        K.activo = true;
        this.estado = 'GRABANDO';
        this._origen = this.real.slice();
        this.tInicio = this.t;
        if (this.opts.lidar) this._capturar();
        this._registrar();
        this.eventos.push({ t: this.t, txt: 'Salida: origen (0, 0, 0) en la linea transversal' });
      }
      if (this.estado === 'GRABANDO' && K.metaDetectada) {
        this.estado = 'FIN';
        K.activo = false;
        this._parar(this.ruedaL); this._parar(this.ruedaR);
        const E = this.est;
        const real = this.poseRealEnOrigen();
        this.fin = {
          t: this.t - this.tInicio,
          dist: E.distRecorrida,
          cierreEkf: Math.hypot(E.xHat[0], E.xHat[1]),
          cierreOdo: Math.hypot(E.xOdo[0], E.xOdo[1]),
          cierreLid: Math.hypot(E.xLid[0], E.xLid[1]),
          errEkf: Math.hypot(E.xHat[0] - real[0], E.xHat[1] - real[1]),
          errOdo: Math.hypot(E.xOdo[0] - real[0], E.xOdo[1] - real[1]),
        };
        this.eventos.push({ t: this.t, txt: 'META detectada: robot detenido' });
      }
    }
    _capturar() { this.ultimoScan = this.tomarBarrido(); this.scanPose = this.real.slice(); return this.ultimoScan; }
    _estimar(dt) {
      const E = this.est;
      const [v, w] = this._calcularVOmega(dt);
      [E.xHat, E.P] = this._prediccion(E.xHat, E.P, v, w, dt);
      const ds = v * dt, dth = w * dt, thm = E.xOdo[2] + 0.5 * dth;
      E.xOdo = [E.xOdo[0] + ds * Math.cos(thm), E.xOdo[1] + ds * Math.sin(thm), wrap(E.xOdo[2] + dth)];
      E.distRecorrida += Math.abs(ds);
      if (this.opts.lidar) {
        const mov = relativa(E.poseUltBarrido, E.xHat);
        if (Math.hypot(mov[0], mov[1]) > C.BARRIDO_CADA_CM || Math.abs(mov[2]) > C.BARRIDO_CADA_DEG * DEG ||
            this.t - E.tUltBarrido > C.BARRIDO_CADA_S) {
          const scan = this._capturar();
          this._procesarBarrido(scan);
          E.poseUltBarrido = E.xHat.slice(); E.tUltBarrido = this.t;
        }
      }
      this._registrar();
    }
    // pose del robot real expresada en el marco del origen (para comparar)
    poseRealEnOrigen() {
      if (!this._origen) return [0, 0, 0];
      return relativa(this._origen, this.real);
    }
    // lleva un punto del marco del robot (origen) al mundo
    aMundo(p) { return this._origen ? componer(this._origen, [p[0], p[1], p[2] || 0]) : p; }
    _registrar() {
      const E = this.est, T = this.trail;
      T.real.push(this.real.slice(0, 2));
      T.ekf.push(this.aMundo(E.xHat).slice(0, 2));
      T.odo.push(this.aMundo(E.xOdo).slice(0, 2));
      T.t.push(this.t);
    }
  }

  G.XRP = { Sim, C, GEO, wrap };
})(typeof window !== 'undefined' ? window : globalThis);
