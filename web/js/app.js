/* =============================================================================
 *  Interfaz del simulador y del visor de datos reales
 * ===========================================================================*/
(function () {
  'use strict';
  const D = window.RETO_DATA;
  const { Sim, C } = window.XRP;
  const RD = window.XRPRender;
  const $ = s => document.querySelector(s);
  const $$ = s => Array.from(document.querySelectorAll(s));

  // ---------------- logo pixel ----------------
  (function logo() {
    const c = $('#logo'), g = c.getContext('2d');
    const P = (x, y, w, h, col) => { g.fillStyle = col; g.fillRect(x, y, w, h); };
    P(2, 1, 12, 3, '#121214'); P(2, 12, 12, 3, '#121214');   // ruedas
    P(3, 4, 10, 8, '#2c2f37'); P(3, 4, 10, 1, '#454955');    // chasis
    P(7, 5, 5, 4, '#c8323c'); P(10, 7, 1, 1, '#4dff7a');      // placa + LED
    P(4, 6, 3, 3, '#0e0e10'); P(5, 7, 1, 1, '#59ff8e');       // LiDAR
    P(13, 6, 2, 4, '#0f6b3f'); P(14, 7, 1, 1, '#ff4d4d'); P(14, 8, 1, 1, '#ff4d4d');
  })();

  // ---------------- video: vista previa y reproductor en la pagina ----------------
  const yt = $('#yt');
  if (yt) {
    const img = yt.querySelector('img');
    img.addEventListener('error', () => { if (!img.dataset.f) { img.dataset.f = 1; img.src = 'https://i.ytimg.com/vi/' + yt.dataset.id + '/0.jpg'; } });
    yt.addEventListener('click', () => {
      const f = document.createElement('iframe');
      f.src = 'https://www.youtube.com/embed/' + yt.dataset.id + '?autoplay=1&rel=0&playsinline=1';
      f.title = 'Robot XRP siguiendo la pista — Reto 1';
      f.allow = 'accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture';
      f.allowFullscreen = true;
      yt.replaceWith(f);
    });
  }

  // ---------------- simulador ----------------
  const opts = { ruido: 1, lidar: true, seed: 7, entorno: 'salon', persona: false };
  let sim = new Sim(D, opts);
  const render = new RD.Render($('#sim'), D);
  render.preparar(sim);
  let playing = false, speed = 4, last = performance.now();
  let errHist = [], tErr = 0;

  function nuevo(seed) {
    sim = new Sim(D, Object.assign({}, opts, seed !== undefined ? { seed } : {}));
    if (seed !== undefined) opts.seed = seed;
    render.preparar(sim);
    errHist = []; tErr = 0;
    $('#fin-card').hidden = true;
    setPlay(false);
    actualizarUI(true);
  }
  function setPlay(p) {
    playing = p;
    $('#b-play').textContent = p ? '⏸ Pausa' : (sim.estado === 'FIN' ? '↺ Otra vuelta' : '▶ Iniciar');
  }
  $('#b-play').addEventListener('click', () => {
    if (sim.estado === 'FIN') { nuevo(opts.seed + 1); setPlay(true); return; }
    setPlay(!playing);
  });
  $('#b-reset').addEventListener('click', () => nuevo(opts.seed));
  $('#b-seed').addEventListener('click', () => nuevo(Math.floor(Math.random() * 1e6)));
  $$('[data-speed]').forEach(b => b.addEventListener('click', () => {
    speed = Number(b.dataset.speed);
    $$('[data-speed]').forEach(x => x.classList.toggle('on', x === b));
  }));
  $$('[data-env]').forEach(b => b.addEventListener('click', () => {
    opts.entorno = b.dataset.env;
    $$('[data-env]').forEach(x => x.classList.toggle('on', x === b));
    nuevo(opts.seed);
  }));
  $('#r-ruido').addEventListener('input', e => { $('#v-ruido').textContent = Number(e.target.value).toFixed(1) + '×'; });
  $('#r-ruido').addEventListener('change', e => { opts.ruido = Number(e.target.value); nuevo(opts.seed); });
  $('#t-lidar').addEventListener('change', e => { opts.lidar = e.target.checked; nuevo(opts.seed); });
  $('#t-persona').addEventListener('change', e => { opts.persona = e.target.checked; sim.opts.persona = e.target.checked; });
  $('#t-rayos').addEventListener('change', e => { render.vis.rayos = e.target.checked; });
  $('#t-seguir').addEventListener('change', e => { render.vis.seguir = e.target.checked; });
  $('#t-odo').addEventListener('change', e => { render.vis.odo = e.target.checked; });
  $('#t-real').addEventListener('change', e => { render.vis.real = e.target.checked; });

  // grafica de error (pixel)
  const ch = $('#chart'), cg = ch.getContext('2d');
  const [cb, cbg] = (() => { const c = document.createElement('canvas'); c.width = 280; c.height = 50; return [c, c.getContext('2d')]; })();
  function grafica() {
    const w = cb.width, h = cb.height, g = cbg;
    g.fillStyle = '#10141e'; g.fillRect(0, 0, w, h);
    g.fillStyle = '#1d2433';
    for (let y = 0; y < h; y += 10) g.fillRect(0, y, w, 1);
    const n = errHist.length;
    const maxE = Math.max(10, ...errHist.map(e => Math.max(e[1], e[2])));
    const tMax = Math.max(60, n ? errHist[n - 1][0] : 0);
    g.fillStyle = '#5a6480'; g.font = '8px monospace';
    const dib = (k, col) => {
      let prev = null;
      for (const e of errHist) {
        const x = Math.round(e[0] / tMax * (w - 2)), y = Math.round(h - 2 - e[k] / maxE * (h - 6));
        if (prev) RD.linea(g, prev[0], prev[1], x, y, col, 0);
        prev = [x, y];
      }
    };
    dib(2, '#ffb547'); dib(1, '#35e0cf');
    cg.imageSmoothingEnabled = false;
    cg.drawImage(cb, 0, 0, ch.width, ch.height);
    cg.fillStyle = '#7a8499'; cg.font = '12px JetBrains Mono, monospace';
    cg.fillText(maxE.toFixed(0) + ' cm', 8, 16);
    cg.fillText(tMax.toFixed(0) + ' s', ch.width - 48, ch.height - 8);
  }

  const fmt = (v, d) => (Math.abs(v) < 1e-9 ? 0 : v).toFixed(d);
  let tUI = 0;
  function actualizarUI(force) {
    const now = performance.now();
    if (!force && now - tUI < 90) return;
    tUI = now;
    const E = sim.est, K = sim.ctl;
    const eb = $('#hud-estado');
    eb.textContent = sim.estado === 'GRABANDO' ? 'GRABANDO' : sim.estado === 'FIN' ? 'META · FIN' : 'ESPERANDO SALIDA';
    eb.className = 'badge ' + (sim.estado === 'GRABANDO' ? 'run' : sim.estado === 'FIN' ? 'end' : 'wait');
    $('#hud-seg').textContent = sim.estado === 'GRABANDO' ? sim.seg.estado.replace('_', ' ') : '—';
    const tr = sim.tInicio !== undefined ? sim.t - sim.tInicio : 0;
    $('#hud-t').textContent = 't = ' + fmt(Math.max(0, tr), 1) + ' s';
    $('#m-v').textContent = fmt(sim.realV, 1);
    $('#m-w').textContent = fmt(sim.realW, 2);
    $('#m-d').textContent = fmt(E.distRecorrida, 0);
    $('#m-yaw').textContent = fmt(sim.yawImu, 0);
    const pct = (v, w, b) => Math.max(0, Math.min(1, (v - w) / (b - w))) * 100;
    $('#b-izq').style.width = pct(K.izq, C.WHITE_LEFT, C.BLACK_LEFT) + '%';
    $('#b-der').style.width = pct(K.der, C.WHITE_RIGHT, C.BLACK_RIGHT) + '%';
    $('#v-izq').textContent = fmt(K.izq, 2);
    $('#v-der').textContent = fmt(K.der, 2);
    $('#b-el').style.width = Math.abs(sim.ruedaL.esfuerzo) / 0.85 * 100 + '%';
    $('#b-er').style.width = Math.abs(sim.ruedaR.esfuerzo) / 0.85 * 100 + '%';
    $('#v-el').textContent = fmt(sim.ruedaL.esfuerzo, 2);
    $('#v-er').textContent = fmt(sim.ruedaR.esfuerzo, 2);
    $('#m-x').textContent = fmt(E.xHat[0], 1);
    $('#m-y').textContent = fmt(E.xHat[1], 1);
    $('#m-th').textContent = fmt(E.xHat[2] * 180 / Math.PI, 0) + '°';
    $('#m-sx').textContent = fmt(Math.sqrt(E.P[0][0]), 2);
    $('#m-sy').textContent = fmt(Math.sqrt(E.P[1][1]), 2);
    $('#m-st').textContent = fmt(Math.sqrt(E.P[2][2]) * 180 / Math.PI, 2) + '°';
    const S = sim.stats;
    $('#m-scans').textContent = S.scans;
    $('#m-acc').textContent = S.aceptadas;
    $('#m-rej').textContent = S.rechGate + S.rechIcp;
    $('#m-n').textContent = sim.diag.n;
    $('#m-rms').textContent = fmt(sim.diag.rms || 0, 2);
    $('#m-d2').textContent = fmt(Math.max(0, sim.diag.d2 || 0), 1);
    if (errHist.length) {
      const e = errHist[errHist.length - 1];
      $('#e-ekf').textContent = fmt(e[1], 1); $('#e-odo').textContent = fmt(e[2], 1);
    }
    grafica();
    if (sim.fin && $('#fin-card').hidden) {
      const f = sim.fin;
      $('#fin-card').innerHTML = `<h4>✔ Vuelta completa</h4><table>
        <tr><td>tiempo</td><td>${fmt(f.t, 1)} s</td></tr>
        <tr><td>distancia (encoders)</td><td>${fmt(f.dist, 0)} cm</td></tr>
        <tr><td>error final EKF</td><td>${fmt(f.errEkf, 1)} cm</td></tr>
        <tr><td>error final odometría</td><td>${fmt(f.errOdo, 1)} cm</td></tr>
        <tr><td>barridos aceptados</td><td>${S.aceptadas} / ${S.scans}</td></tr></table>`;
      $('#fin-card').hidden = false;
      setPlay(false);
    }
  }

  function registrarError() {
    if (sim.estado !== 'GRABANDO' && sim.estado !== 'FIN') return;
    const tr = sim.t - sim.tInicio;
    if (tr - tErr < 0.5 && errHist.length) return;
    tErr = tr;
    const r = sim.real, pe = sim.aMundo(sim.est.xHat), po = sim.aMundo(sim.est.xOdo);
    errHist.push([tr, Math.hypot(pe[0] - r[0], pe[1] - r[1]), Math.hypot(po[0] - r[0], po[1] - r[1])]);
  }

  // pausa cuando el simulador no esta a la vista
  let visible = true;
  new IntersectionObserver(es => { visible = es[0].isIntersecting; }, { threshold: 0.05 }).observe($('#sim'));

  function frame(ts) {
    const dt = Math.min(0.1, (ts - last) / 1000); last = ts;
    if (playing && sim.estado !== 'FIN') {
      let rest = dt * speed;
      while (rest > 0) { const h = Math.min(rest, 0.1); sim.paso(h); registrarError(); rest -= h; if (sim.estado === 'FIN') break; }
    } else if (!playing && sim.estado === 'ESPERANDO_SALIDA') {
      sim.lidarAng += dt * 2 * Math.PI * 10;   // el LiDAR gira aunque el robot espere
    }
    if (visible) { render.dibujar(sim, ts / 1000); actualizarUI(); }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);

  // ---------------- datos reales ----------------
  const rc = $('#real'), rg = rc.getContext('2d');
  const [rbuf, rbg] = (() => { const c = document.createElement('canvas'); c.width = RD.W; c.height = RD.H; const g = c.getContext('2d'); g.imageSmoothingEnabled = false; return [c, g]; })();
  const fondoReal = RD.pistaEstatica(D, { tipo: 'abierto', objetos: [
    { tipo: 'caja', x: 118, y: 120, w: 20, h: 15 }, { tipo: 'zapato', x: -26, y: 70, w: 7, h: 11 }, { tipo: 'zapato', x: -26, y: 92, w: 7, h: 11 }] });
  let capa = 'raw', idx = D.raw.length - 1, replay = false, tRep = 0;
  $$('[data-layer]').forEach(b => b.addEventListener('click', () => {
    capa = b.dataset.layer; $$('[data-layer]').forEach(x => x.classList.toggle('on', x === b)); dibujarReal();
  }));
  const scrub = $('#r-scrub');
  scrub.addEventListener('input', () => { idx = Number(scrub.value); replay = false; $('#b-replay').textContent = '▶ Reproducir'; dibujarReal(); });
  $('#b-replay').addEventListener('click', () => {
    replay = !replay;
    if (replay && idx >= D.raw.length - 1) idx = 0;
    $('#b-replay').textContent = replay ? '⏸ Pausa' : '▶ Reproducir';
  });
  function marcador(g, arr, i, col) {
    const n = arr.length, p = arr[i];
    const a0 = arr[Math.max(0, Math.min(i, n - 4))], a1 = arr[Math.min(n - 1, Math.max(i, 3) + 3)];
    const q = [p[0] + (a1[0] - a0[0]), p[1] + (a1[1] - a0[1])];
    const [X, Y] = RD.toB(p[0], p[1]);
    const a = Math.atan2(q[1] - p[1], q[0] - p[0]);
    const b = RD.angB(a);
    g.save(); g.translate(Math.round(X), Math.round(Y)); g.rotate(b);
    g.fillStyle = 'rgba(10,12,20,.4)'; g.fillRect(-7, -6, 16, 14);
    g.fillStyle = '#121214'; g.fillRect(-6, -9, 12, 3); g.fillRect(-6, 6, 12, 3);
    g.fillStyle = '#2c2f37'; g.fillRect(-7, -6, 14, 12);
    g.fillStyle = '#c8323c'; g.fillRect(-1, -4, 6, 6);
    g.fillStyle = col; g.fillRect(6, -2, 3, 4);
    g.restore();
  }
  function dibujarReal() {
    const g = rbg;
    g.drawImage(fondoReal, 0, 0);
    const pts = (arr, col, hasta) => {
      for (let i = 1; i <= hasta; i++) {
        const a = RD.toB(arr[i - 1][0], arr[i - 1][1]), b = RD.toB(arr[i][0], arr[i][1]);
        RD.linea(g, a[0], a[1], b[0], b[1], col, 0);
      }
    };
    if (capa === 'raw' || capa === 'both') pts(D.raw, '#35e0cf', idx);
    if (capa === 'fixed' || capa === 'both') pts(D.fixed, '#ffb547', idx);
    if (capa === 'both') {
      const a = RD.toB(D.raw[idx][0], D.raw[idx][1]), b = RD.toB(D.fixed[idx][0], D.fixed[idx][1]);
      RD.linea(g, a[0], a[1], b[0], b[1], 'rgba(255,95,109,.9)', 2);
    }
    if (capa !== 'fixed') marcador(g, D.raw, idx, '#35e0cf');
    if (capa !== 'raw') marcador(g, D.fixed, idx, '#ffb547');
    // origen
    const [OX, OY] = RD.toB(D.start[0], D.start[1]);
    g.fillStyle = '#59ff8e'; g.fillRect(Math.round(OX) - 2, Math.round(OY) - 2, 5, 5);
    rg.imageSmoothingEnabled = false;
    rg.drawImage(rbuf, 0, 0, rc.width, rc.height);
    scrub.value = idx;
    $('#scrub-info').textContent = `punto ${idx + 1} · ${D.s[idx].toFixed(0)} cm recorridos · error ${D.err[idx].toFixed(1)} cm`;
  }
  function loopReal(ts) {
    if (replay) {
      if (ts - tRep > 16) { tRep = ts; idx = Math.min(D.raw.length - 1, idx + 3); if (idx >= D.raw.length - 1) { replay = false; $('#b-replay').textContent = '▶ Reproducir'; } dibujarReal(); }
    }
    requestAnimationFrame(loopReal);
  }
  dibujarReal();
  requestAnimationFrame(loopReal);
})();
