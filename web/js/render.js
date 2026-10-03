/* =============================================================================
 *  Render pixel art del simulador.
 *  Todo se dibuja a baja resolucion (2 px por cm) en un buffer y se escala con
 *  vecino mas cercano. El robot es un sprite procedural que se regenera en cada
 *  cuadro (ruedas, LED, LiDAR, cables, inclinacion) y se rota con "nearest".
 * ===========================================================================*/
(function (G) {
  'use strict';
  const PPC = 2;                 // pixeles por cm
  const X0 = -45, X1 = 145;      // rango del mundo en x (cm)  -> eje vertical en pantalla
  const Y0 = -50, Y1 = 250;      // rango del mundo en y (cm)  -> eje horizontal en pantalla
  const W = (Y1 - Y0) * PPC, H = (X1 - X0) * PPC;

  const PAL = {
    floor: ['#22263a', '#262a40', '#2b3046', '#30354d', '#1d2133'],
    banner: '#e7e5ee', bannerSh: '#d5d2df', bannerHi: '#f3f2f8', bannerEdge: '#b9b4cb',
    line: '#17171c', tape: '#e2cf96', tapeSh: '#c9b67d',
    wallTop: '#151a33', wallFace: '#0d1022', wallHi: '#FF3CAC',
    shadow: 'rgba(8,4,20,0.42)',
    ekf: '#00E5FF', odo: '#FF3CAC', real: 'rgba(255,255,255,0.55)', lidar: '#7CFF6B',
  };

  // mundo (cm, y arriba) -> buffer (px)
  const toB = (x, y) => [(y - Y0) * PPC, (x - X0) * PPC];
  // angulo del robot en el buffer (rotacion visual de -90 grados)
  const angB = th => Math.atan2(Math.cos(th), Math.sin(th));

  function hash(i, j) { let h = (i * 374761393 + j * 668265263) ^ 0x5bd1e995; h = Math.imul(h ^ (h >>> 13), 1274126177); return ((h ^ (h >>> 16)) >>> 0) / 4294967296; }

  function canvas(w, h) { const c = document.createElement('canvas'); c.width = w; c.height = h; const g = c.getContext('2d'); g.imageSmoothingEnabled = false; return [c, g]; }

  // ---------------------------------------------------------------------------
  //  Fondo estatico: piso, lona, linea, muros y objetos
  // ---------------------------------------------------------------------------
  function fondo(data, entorno) {
    const [c, g] = canvas(W, H);
    const img = g.createImageData(W, H);
    const px = img.data;
    const hex = h => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
    const floor = PAL.floor.map(hex);
    const madera = ['#11152A', '#141933', '#181d3b', '#0e1124', '#2b3270'].map(hex);
    const set = (i, rgb) => { px[i] = rgb[0]; px[i + 1] = rgb[1]; px[i + 2] = rgb[2]; px[i + 3] = 255; };
    const abierto = entorno.tipo === 'abierto';
    // piso
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
      const k = (j * W + i) * 4;
      if (abierto) {
        // concreto con manchas
        const n = hash(i, j), m = hash(i >> 3, j >> 3), big = hash(i >> 5, j >> 5);
        let idx = n < 0.12 ? 4 : n < 0.55 ? 1 : n < 0.85 ? 2 : 3;
        if (m < 0.18) idx = Math.max(0, idx - 1);
        if (big < 0.15) idx = Math.min(4, idx + 1) === 4 ? 0 : idx;
        set(k, floor[idx]);
      } else {
        // piso de laboratorio: baldosas oscuras con juntas de neon
        const borde = i % 40 === 0 || j % 40 === 0;
        const n = hash(i >> 1, j >> 1), t = hash(i / 40 | 0, j / 40 | 0);
        let idx = borde ? 4 : (n < 0.18 ? 3 : n < 0.65 ? (t < 0.5 ? 0 : 1) : 2);
        set(k, madera[idx]);
      }
    }
    // lona (banner) con arrugas suaves
    const b0 = toB(-8, -10), b1 = toB(107.5, 208.5);
    const bx0 = Math.round(Math.min(b0[0], b1[0])), bx1 = Math.round(Math.max(b0[0], b1[0]));
    const by0 = Math.round(Math.min(b0[1], b1[1])), by1 = Math.round(Math.max(b0[1], b1[1]));
    const ban = hex(PAL.banner), sh = hex(PAL.bannerSh), hi = hex(PAL.bannerHi), ed = hex(PAL.bannerEdge);
    // sombra de la lona
    for (let j = by0 + 2; j <= by1 + 2; j++) for (let i = bx0 + 2; i <= bx1 + 2; i++) {
      if (i >= W || j >= H) continue;
      const k = (j * W + i) * 4; px[k] *= 0.82; px[k + 1] *= 0.82; px[k + 2] *= 0.85;
    }
    for (let j = by0; j <= by1; j++) for (let i = bx0; i <= bx1; i++) {
      const k = (j * W + i) * 4;
      const w = Math.sin(i * 0.045 + Math.sin(j * 0.07) * 1.6) + 0.6 * Math.sin(j * 0.11 + i * 0.013);
      let col = ban;
      if (w > 1.05) col = hi; else if (w < -1.15) col = sh;
      if ((w < -0.95 && w > -1.15) && ((i + j) & 1)) col = sh;   // dither
      if (i === bx0 || i === bx1 || j === by0 || j === by1) col = ed;
      set(k, col);
    }
    g.putImageData(img, 0, 0);
    // ojales y cinta de enmascarar
    const ojal = (x, y) => { g.fillStyle = '#8b8d90'; g.fillRect(x - 2, y - 2, 5, 5); g.fillStyle = '#c9cbce'; g.fillRect(x - 1, y - 2, 3, 1); g.fillStyle = '#3a3c40'; g.fillRect(x - 1, y - 1, 3, 3); };
    ojal(bx0 + 5, by0 + 5); ojal(bx1 - 5, by0 + 5); ojal(bx0 + 5, by1 - 5); ojal(bx1 - 5, by1 - 5);
    const cinta = (x, y, w, h) => { g.fillStyle = PAL.tape; g.fillRect(x, y, w, h); g.fillStyle = PAL.tapeSh; g.fillRect(x, y + h - 1, w, 1); };
    for (let t = 0; t < 6; t++) cinta(bx0 + 40 + t * 90, by1 - 3, 16, 7);
    for (let t = 0; t < 3; t++) cinta(bx1 - 3, by0 + 40 + t * 90, 7, 16);
    // linea de la pista: estampado de pixeles
    const lp = data.track, n = lp.length;
    const r = (1.9 / 2) * PPC;
    g.fillStyle = PAL.line;
    for (let a = 0; a < n; a++) {
      const p = lp[a], q = lp[(a + 1) % n];
      for (let t = 0; t < 1; t += 0.34) {
        const [X, Y] = toB(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t);
        for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) if (dx * dx + dy * dy <= r * r + 0.3) g.fillRect(Math.round(X + dx), Math.round(Y + dy), 1, 1);
      }
    }
    // linea transversal (perpendicular al avance, mas angosta)
    const st = data.start;
    for (let o = -4.5; o <= 4.5; o += 0.25) {
      const [X, Y] = toB(st[0] + o, st[1]);
      g.fillRect(Math.round(X) - 1, Math.round(Y), 3, 1);
    }
    // muros y objetos (vistos desde arriba, con cara frontal y sombra)
    const caja3d = (x, y, w, h, top, face, hiC, alto) => {
      const [X, Y] = toB(x, y); // esquina superior izquierda en pantalla
      const bw = h * PPC, bh = w * PPC;
      g.fillStyle = PAL.shadow; g.fillRect(Math.round(X) + 3, Math.round(Y) + 3, Math.round(bw), Math.round(bh));
      g.fillStyle = face; g.fillRect(Math.round(X), Math.round(Y) + alto, Math.round(bw), Math.round(bh) - alto);
      g.fillStyle = top; g.fillRect(Math.round(X), Math.round(Y), Math.round(bw), Math.round(bh) - alto);
      g.fillStyle = hiC; g.fillRect(Math.round(X), Math.round(Y), Math.round(bw), 1); g.fillRect(Math.round(X), Math.round(Y), 1, Math.round(bh) - alto);
      return [Math.round(X), Math.round(Y), Math.round(bw), Math.round(bh)];
    };
    if (entorno.muros && !abierto) {
      const [mx0, my0, mx1, my1] = entorno.muros;
      // muros como bandas gruesas en el borde
      g.fillStyle = PAL.wallFace;
      const a = toB(mx0, my0), b = toB(mx1, my1);
      const t = 6;
      g.fillStyle = PAL.wallTop;
      g.fillRect(0, 0, W, Math.round(a[1]));
      g.fillRect(0, Math.round(b[1]), W, H - Math.round(b[1]));
      g.fillRect(0, 0, Math.round(a[0]), H);
      g.fillRect(Math.round(b[0]), 0, W - Math.round(b[0]), H);
      g.fillStyle = PAL.wallFace;
      g.fillRect(Math.round(a[0]), Math.round(a[1]), Math.round(b[0] - a[0]), t);
      g.fillStyle = PAL.wallHi;
      g.fillRect(Math.round(a[0]), Math.round(a[1]) - 1, Math.round(b[0] - a[0]), 1);
      g.fillRect(Math.round(a[0]), Math.round(b[1]), Math.round(b[0] - a[0]), 1);
      g.fillStyle = '#00E5FF';
      g.fillRect(Math.round(a[0]) - 1, Math.round(a[1]), 1, Math.round(b[1] - a[1]));
      g.fillRect(Math.round(b[0]), Math.round(a[1]), 1, Math.round(b[1] - a[1]));
      // ladrillo sutil en el borde superior
      g.fillStyle = 'rgba(91,95,239,0.12)';
      for (let i = 0; i < W; i += 12) for (let j = 0; j < Math.round(a[1]); j += 6) g.fillRect(i + ((j / 6) % 2) * 6, j, 11, 5);
      // puerta en el muro derecho (pantalla: abajo)
      const p0 = toB(142, 150), p1 = toB(142, 178);
      g.fillStyle = '#20232c'; g.fillRect(Math.round(p0[0]), Math.round(b[1]), Math.round(p1[0] - p0[0]), H - Math.round(b[1]));
      g.fillStyle = '#6b5233'; g.fillRect(Math.round(p0[0]), Math.round(b[1]), 3, H - Math.round(b[1]));
    }
    for (const o of entorno.objetos) {
      if (o.tipo === 'caja') {
        const [X, Y, bw, bh] = caja3d(o.x, o.y, o.w, o.h, '#d43d3d', '#9e2828', '#f07070', 5);
        g.fillStyle = '#fff'; g.fillRect(X + 6, Y + 6, bw - 12, 4); g.fillStyle = '#ffd34d'; g.fillRect(X + 6, Y + 12, 8, 3);
        g.fillStyle = '#1b2b5a'; g.fillRect(X + bw - 12, Y + 12, 6, 6);
      } else if (o.tipo === 'columna') caja3d(o.x, o.y, o.w, o.h, '#59607a', '#3a3f52', '#727a96', 8);
      else if (o.tipo === 'banco') {
        const [X, Y, bw, bh] = caja3d(o.x, o.y, o.w, o.h, '#8a6a48', '#5d4630', '#a8835b', 6);
        g.fillStyle = '#5d4630'; for (let i = 4; i < bw - 2; i += 10) g.fillRect(X + i, Y + 2, 1, bh - 10);
        g.fillStyle = '#2b2f3a'; g.fillRect(X + 8, Y + 8, 18, 12); g.fillStyle = '#3d7bd9'; g.fillRect(X + 9, Y + 9, 16, 8);
      } else if (o.tipo === 'morral') {
        const [X, Y, bw, bh] = caja3d(o.x, o.y, o.w, o.h, '#2f5d8a', '#1f3f60', '#4a7fb3', 4);
        g.fillStyle = '#1f3f60'; g.fillRect(X + 4, Y + Math.round(bh / 2) - 2, bw - 8, 2);
        g.fillStyle = '#e7c34b'; g.fillRect(X + Math.round(bw / 2), Y + 3, 2, 2);
      } else if (o.tipo === 'mueble') caja3d(o.x, o.y, o.w, o.h, '#7a7f8c', '#4d515c', '#9ba0ad', 6);
      else if (o.tipo === 'pata') caja3d(o.x, o.y, o.w, o.h, '#b8bcc6', '#7d818b', '#e2e5ec', 2);
      else if (o.tipo === 'zapato') {
        const [X, Y, bw, bh] = caja3d(o.x, o.y, o.w, o.h, '#f2f2f2', '#c9c9c9', '#ffffff', 2);
        g.fillStyle = '#d9d9d9'; for (let i = 3; i < bw - 3; i += 3) g.fillRect(X + i, Y + 3, 1, 1);
      }
    }
    return c;
  }

  // ---------------------------------------------------------------------------
  //  Sprite procedural del robot XRP (vista superior, adelante = +x)
  // ---------------------------------------------------------------------------
  const SPR = 52;
  const [sprC, sg] = canvas(SPR, SPR);
  function spriteRobot(sim, t) {
    const g = sg; g.clearRect(0, 0, SPR, SPR);
    const cx = SPR / 2, cy = SPR / 2;
    const A = sim.anim;
    const lean = Math.max(-2, Math.min(2, Math.round(A.lean * 3)));     // cabeceo por aceleracion
    const roll = Math.max(-1, Math.min(1, Math.round(sim.realW * sim.realV * 0.05)));
    const R = (x, y, w, h, c) => { g.fillStyle = c; g.fillRect(Math.round(cx + x), Math.round(cy + y), w, h); };
    // ruedas (izq arriba, der abajo) con banda de rodadura animada
    const rueda = (y0, ang) => {
      R(-8, y0, 16, 5, '#121214');
      const fase = Math.floor(((ang * 3.0 * PPC) % 4 + 4) % 4);
      for (let i = -8 + fase; i < 8; i += 4) R(i, y0, 2, 5, '#2b2b30');
      R(-8, y0, 16, 1, '#3b3b42');
      R(-2, y0 + 1, 4, 3, '#c7c9cf'); // buje
    };
    rueda(-18, sim.motorL.angulo);
    rueda(13, sim.motorR.angulo);
    // chasis
    R(-15, -13, 30, 26, '#1f2127');
    R(-14, -12, 28, 24, '#2c2f37');
    R(-14, -12, 28, 1, '#454955');
    R(-14, -12, 1, 24, '#3c404b');
    // rueda loca trasera
    R(-15, -2, 3, 4, '#8a8d95');
    // barra frontal con sensores de reflectancia (brillo IR)
    R(14, -5, 4, 10, '#0f6b3f'); R(14, -5, 4, 1, '#22a06a');
    const ir = (Math.sin(t * 40) > 0) ? '#ff4d4d' : '#c42f2f';
    R(16, -2, 2, 1, ir); R(16, 1, 2, 1, ir);
    // bateria
    R(-12 + lean, 4 + roll, 10, 7, '#2f6fd1'); R(-12 + lean, 4 + roll, 10, 1, '#6aa0f0');
    // placa XRP (roja) con chips
    R(-2 + lean, -10 + roll, 14, 12, '#c8323c'); R(-2 + lean, -10 + roll, 14, 1, '#ef6b73');
    R(2 + lean, -7 + roll, 4, 4, '#1a1a1d'); R(8 + lean, -8 + roll, 2, 2, '#e8c75a'); R(0 + lean, -1 + roll, 3, 2, '#e9e9e9');
    // LED RGB segun estado
    const led = sim.estado === 'GRABANDO' ? '#4dff7a' : sim.estado === 'FIN' ? '#ff4d4d' : '#ffd34d';
    const parp = sim.estado === 'ESPERANDO_SALIDA' ? (Math.sin(t * 6) > 0) : true;
    R(9 + lean, -3 + roll, 2, 2, parp ? led : '#5a4a1a');
    // LiDAR (disco que gira)
    const lx = -6 + lean, ly = -6 + roll;
    g.fillStyle = '#0e0e10';
    for (let dy = -5; dy <= 5; dy++) for (let dx = -5; dx <= 5; dx++) if (dx * dx + dy * dy <= 26) g.fillRect(cx + lx + dx, cy + ly + dy, 1, 1);
    g.fillStyle = '#2a2a30';
    for (let dy = -3; dy <= 3; dy++) for (let dx = -3; dx <= 3; dx++) if (dx * dx + dy * dy <= 10) g.fillRect(cx + lx + dx, cy + ly + dy, 1, 1);
    const la = sim.lidarAng;
    g.fillStyle = '#7CFF6B'; g.fillRect(Math.round(cx + lx + Math.cos(la) * 3), Math.round(cy + ly + Math.sin(la) * 3), 1, 1);
    g.fillStyle = '#9a9aa3'; g.fillRect(cx + lx, cy + ly, 1, 1);
    // cable que se mueve con la marcha (animacion procedural)
    g.fillStyle = '#e3542c';
    for (let i = 0; i < 9; i++) {
      const wob = Math.round(Math.sin(t * 9 + i * 0.9) * Math.min(1, Math.abs(sim.realV) * 0.25 + 0.15));
      g.fillRect(cx + lx + 5 + i, cy + ly + 4 + Math.round(i * 0.4) + wob, 1, 1);
    }
    return sprC;
  }

  // persona vista desde arriba (procedural)
  function dibujarPersona(g, sim) {
    if (!sim.opts.persona) return;
    const p = sim.persona();
    const [X, Y] = toB(p.cx, p.cy);
    const a = angB(Math.atan2(p.hy, p.hx));
    g.save(); g.translate(Math.round(X), Math.round(Y)); g.rotate(a);
    const paso = Math.sin(sim.t * 4.2) * 9 * PPC;
    // sombra
    g.fillStyle = 'rgba(10,12,20,0.30)'; g.fillRect(-12 + 3, -22 + 3, 24, 44);
    // zapatos
    g.fillStyle = '#f2f2f2'; g.fillRect(-6 + paso / 2, -16, 14, 9); g.fillRect(-6 - paso / 2, 7, 14, 9);
    g.fillStyle = '#9b9b9b'; g.fillRect(-6 + paso / 2, -8, 14, 1); g.fillRect(-6 - paso / 2, 15, 14, 1);
    // cuerpo (hombros) y cabeza
    g.fillStyle = '#26324a'; g.fillRect(-9, -22, 14, 44);
    g.fillStyle = '#3f557d'; g.fillRect(-9, -22, 14, 2);
    g.fillStyle = '#2b1d14'; g.fillRect(-6, -8, 13, 16);
    g.fillStyle = '#3d2a1e'; g.fillRect(-4, -6, 6, 4);
    g.restore();
  }

  // letrero vertical "로봇 실험실" (laboratorio de robots) con neon que parpadea
  let fuenteLista = false;
  if (document.fonts && document.fonts.load) document.fonts.load('22px Galmuri').then(() => { fuenteLista = true; });
  function letrero(g, t) {
    if (!fuenteLista) return;
    const txt = '로봇실험실';
    const fl = Math.sin(t * 13) + Math.sin(t * 7.3) > 1.7 ? 0.35 : 1;   // fallas del tubo
    g.save();
    g.font = '22px Galmuri'; g.textAlign = 'center'; g.textBaseline = 'top';
    for (let k = 0; k < txt.length; k++) {
      const y = 112 + k * 30;
      g.shadowColor = 'rgba(255,60,172,' + 0.9 * fl + ')'; g.shadowBlur = 10;
      g.fillStyle = 'rgba(255,120,200,' + fl + ')';
      g.fillText(txt[k], 46, y);
    }
    g.shadowBlur = 0;
    g.fillStyle = 'rgba(0,229,255,' + 0.7 * fl + ')';
    g.fillRect(30, 104, 32, 1); g.fillRect(30, 112 + txt.length * 30, 32, 1);
    g.restore();
  }

  // linea de pixeles (Bresenham)
  function linea(g, x0, y0, x1, y1, col, cada) {
    x0 = Math.round(x0); y0 = Math.round(y0); x1 = Math.round(x1); y1 = Math.round(y1);
    const dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    let err = dx + dy, k = 0;
    g.fillStyle = col;
    for (let n = 0; n < 2000; n++) {
      if (!cada || k++ % cada === 0) g.fillRect(x0, y0, 1, 1);
      if (x0 === x1 && y0 === y1) break;
      const e2 = 2 * err;
      if (e2 >= dy) { err += dy; x0 += sx; }
      if (e2 <= dx) { err += dx; y0 += sy; }
    }
  }

  // ---------------------------------------------------------------------------
  //  Renderer
  // ---------------------------------------------------------------------------
  class Render {
    constructor(canvasEl, data) {
      this.el = canvasEl;
      this.data = data;
      this.ctx = canvasEl.getContext('2d');
      [this.buf, this.g] = canvas(W, H);
      [this.trailC, this.tg] = canvas(W, H);
      this.fondoC = null; this.entornoKey = null;
      this.vis = { rayos: true, ekf: true, odo: true, real: true, seguir: false };
      this.cam = { x: W / 2, y: H / 2, zoom: 1 };
      this.trailIdx = 0;
      this.particulas = [];
    }
    preparar(sim) {
      const key = sim.entorno.tipo;
      if (key !== this.entornoKey) { this.fondoC = fondo(this.data, sim.entorno); this.entornoKey = key; }
      this.tg.clearRect(0, 0, W, H); this.trailIdx = 0; this.particulas = [];
    }
    _trails(sim) {
      const T = sim.trail, g = this.tg;
      for (let i = Math.max(1, this.trailIdx); i < T.real.length; i++) {
        const seg = (arr, col, cada) => { const a = toB(arr[i - 1][0], arr[i - 1][1]), b = toB(arr[i][0], arr[i][1]); linea(g, a[0], a[1], b[0], b[1], col, cada); };
        seg(T.real, 'rgba(255,255,255,0.35)', 0);
        seg(T.odo, PAL.odo, 0);
        seg(T.ekf, PAL.ekf, 0);
      }
      this.trailIdx = T.real.length;
    }
    dibujar(sim, tAnim) {
      const g = this.g;
      g.drawImage(this.fondoC, 0, 0);
      // trayectorias (capa incremental, se filtra por visibilidad redibujando si cambia)
      this._trails(sim);
      if (this.vis.ekf || this.vis.odo || this.vis.real) {
        if (this.vis.ekf && this.vis.odo && this.vis.real) g.drawImage(this.trailC, 0, 0);
        else this._trailsFiltradas(sim);
      }
      // letrero de neon en hangul (parpadeo procedural)
      letrero(g, tAnim);
      // persona
      dibujarPersona(g, sim);
      // LiDAR: rayos del ultimo barrido
      const [x, y, th] = sim.real;
      const [RX, RY] = toB(x, y);
      if (this.vis.rayos && sim.ultimoScan && sim.scanPose) {
        const sp = sim.scanPose;
        const [SX, SY] = toB(sp[0], sp[1]);
        for (let i = 0; i < 360; i += 3) {
          const d = sim.ultimoScan[i];
          if (!d || d < 120 || d > 6000) continue;
          const a = sp[2] + i * Math.PI / 180, dc = d / 10;
          const [HX, HY] = toB(sp[0] + dc * Math.cos(a), sp[1] + dc * Math.sin(a));
          linea(g, SX, SY, HX, HY, 'rgba(124,255,107,0.30)', 2);
          g.fillStyle = PAL.lidar; g.fillRect(Math.round(HX) - 1, Math.round(HY) - 1, 2, 2);
        }
        // barrido giratorio
        const la = sim.lidarAng;
        const [EX, EY] = toB(x + 60 * Math.cos(th + la), y + 60 * Math.sin(th + la));
        linea(g, RX, RY, EX, EY, 'rgba(190,255,180,0.5)', 2);
      }
      // polvo procedural cuando una rueda empuja fuerte
      const emit = Math.abs(sim.motorL.v - sim.motorR.v) > 5 && sim.estado === 'GRABANDO';
      if (emit && Math.random() < 0.5) {
        const back = th + Math.PI;
        this.particulas.push({ x: x + 9 * Math.cos(back) + (Math.random() - 0.5) * 12, y: y + 9 * Math.sin(back) + (Math.random() - 0.5) * 12, v: 0, life: 1 });
      }
      for (const p of this.particulas) {
        p.life -= 0.04;
        const [PX, PY] = toB(p.x, p.y);
        g.fillStyle = `rgba(170,165,150,${Math.max(0, p.life) * 0.6})`;
        g.fillRect(Math.round(PX), Math.round(PY), 1, 1);
      }
      this.particulas = this.particulas.filter(p => p.life > 0);
      // sombra del robot + sprite rotado (vecino mas cercano)
      const spr = spriteRobot(sim, tAnim);
      const a = angB(th);
      g.save(); g.translate(Math.round(RX) + 2, Math.round(RY) + 2); g.rotate(a);
      g.fillStyle = PAL.shadow; g.fillRect(-15, -18, 30, 36); g.restore();
      g.save(); g.translate(Math.round(RX), Math.round(RY)); g.rotate(a); g.drawImage(spr, -SPR / 2, -SPR / 2); g.restore();
      // halo del LED (pulso)
      if (sim.estado === 'GRABANDO') {
        const pul = 0.25 + 0.15 * Math.sin(tAnim * 5);
        g.fillStyle = `rgba(77,255,122,${pul})`;
        const lx = RX + Math.cos(a) * 9 - Math.sin(a) * -3, ly = RY + Math.sin(a) * 9 + Math.cos(a) * -3;
        g.fillRect(Math.round(lx) - 2, Math.round(ly) - 2, 5, 5);
      }
      // estimacion EKF como cruz fantasma
      if (sim.estado !== 'ESPERANDO_SALIDA' && this.vis.ekf) {
        const pe = sim.aMundo(sim.est.xHat);
        const [EX, EY] = toB(pe[0], pe[1]);
        g.fillStyle = PAL.ekf;
        g.fillRect(Math.round(EX) - 4, Math.round(EY), 9, 1); g.fillRect(Math.round(EX), Math.round(EY) - 4, 1, 9);
        const ae = angB(pe[2]);
        g.fillRect(Math.round(EX + Math.cos(ae) * 7), Math.round(EY + Math.sin(ae) * 7), 2, 2);
      }
      this._componer(RX, RY);
    }
    _trailsFiltradas(sim) {
      const T = sim.trail, g = this.g;
      const n = T.real.length;
      const dib = (arr, col) => { for (let i = 1; i < n; i++) { const a = toB(arr[i - 1][0], arr[i - 1][1]), b = toB(arr[i][0], arr[i][1]); linea(g, a[0], a[1], b[0], b[1], col, 0); } };
      if (this.vis.real) dib(T.real, 'rgba(255,255,255,0.35)');
      if (this.vis.odo) dib(T.odo, PAL.odo);
      if (this.vis.ekf) dib(T.ekf, PAL.ekf);
    }
    _componer(RX, RY) {
      const el = this.el, ctx = this.ctx;
      const cw = el.width, ch = el.height;
      ctx.imageSmoothingEnabled = false;
      ctx.fillStyle = '#0d1017'; ctx.fillRect(0, 0, cw, ch);
      const c = this.cam;
      const tz = this.vis.seguir ? 2.6 : 1;
      c.zoom += (tz - c.zoom) * 0.08;
      const tx = this.vis.seguir ? RX : W / 2, ty = this.vis.seguir ? RY : H / 2;
      c.x += (tx - c.x) * 0.1; c.y += (ty - c.y) * 0.1;
      const vw = W / c.zoom, vh = H / c.zoom;
      let sx = c.x - vw / 2, sy = c.y - vh / 2;
      sx = Math.max(0, Math.min(W - vw, sx)); sy = Math.max(0, Math.min(H - vh, sy));
      ctx.drawImage(this.buf, sx, sy, vw, vh, 0, 0, cw, ch);
    }
  }

  // Dibujo estatico para la seccion de datos reales
  function pistaEstatica(data, entorno) { return fondo(data, entorno); }

  G.XRPRender = { Render, toB, W, H, PPC, linea, pistaEstatica, angB, spriteRobot, PAL };
})(window);
