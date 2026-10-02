import RFB from '/novnc/core/rfb.js';

const screen = document.querySelector('#screen');
const status = document.querySelector('#connection');
const notice = document.querySelector('#notice');
const controls = ['surface', 'underwater', 'follow', 'stop'].map(id => document.getElementById(id));
const held = new Set();
const motionKeys = new Map([
  ['KeyW', 0x77], ['KeyS', 0x73], ['KeyA', 0x61], ['KeyD', 0x64],
  ['KeyQ', 0x71], ['KeyE', 0x65], ['KeyZ', 0x7a], ['KeyX', 0x78],
  ['ArrowLeft', 0xff51], ['ArrowUp', 0xff52], ['ArrowRight', 0xff53], ['ArrowDown', 0xff54],
]);
let connection;
let retry;

function pulse(key, code) {
  connection?.sendKey(key, code, true);
  connection?.sendKey(key, code, false);
}

function stopMotion() {
  held.clear();
  pulse(0x20, 'Space');
  document.querySelector('#drive-state').textContent = 'Click the viewport to drive';
}

function refreshMotion() {
  held.forEach(code => pulse(motionKeys.get(code), code));
  if (held.size) document.querySelector('#drive-state').textContent = 'Driving · ' + [...held].map(code =>
    code.replace('Key', '').replace('Arrow', '')).join(' + ');
}

// Short, paired key events avoid held VNC keys and expire in the simulator after 500 ms.
setInterval(refreshMotion, 100);
window.addEventListener('keydown', event => {
  if (!connection || status.dataset.state !== 'live' || !screen.contains(document.activeElement)) return;
  if (motionKeys.has(event.code) || event.code === 'Space') {
    event.preventDefault();
    event.stopImmediatePropagation();
    if (event.code === 'Space') stopMotion();
    else { held.add(event.code); refreshMotion(); }
  }
}, true);
window.addEventListener('keyup', event => {
  if (!held.has(event.code) && event.code !== 'Space') return;
  event.preventDefault();
  event.stopImmediatePropagation();
  held.delete(event.code);
  const remaining = [...held];
  stopMotion();
  remaining.forEach(code => held.add(code));
  refreshMotion();
}, true);
window.addEventListener('blur', stopMotion);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopMotion(); });
screen.addEventListener('focusout', event => { if (!screen.contains(event.relatedTarget)) stopMotion(); });

function connect() {
  clearTimeout(retry);
  stopMotion();
  const previous = connection;
  connection = null;
  previous?.disconnect();
  screen.replaceChildren();
  status.dataset.state = 'connecting';
  status.textContent = 'Connecting';
  notice.hidden = false;
  notice.textContent = 'Connecting to the pool…';
  controls.forEach(button => { button.disabled = true; });
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  const session = new RFB(screen, `${protocol}://${location.host}/websockify`);
  connection = session;
  session.scaleViewport = true;
  session.resizeSession = false;
  session.background = '#050b10';
  session.qualityLevel = 7;
  session.compressionLevel = 2;
  session.addEventListener('connect', () => {
    if (connection !== session) return;
    status.dataset.state = 'live';
    status.textContent = 'Live simulation';
    notice.hidden = true;
    controls.forEach(button => { button.disabled = false; });
  });
  session.addEventListener('disconnect', () => {
    if (connection !== session) return;
    held.clear();
    document.querySelector('#drive-state').textContent = 'Teleop disconnected';
    status.dataset.state = 'offline';
    status.textContent = 'Disconnected';
    notice.hidden = false;
    notice.textContent = 'Connection lost. Reconnecting…';
    controls.forEach(button => { button.disabled = true; });
    retry = setTimeout(connect, 4000);
  });
}

function sendKey(key, code) {
  pulse(key, code);
  connection?.focus();
}

document.querySelector('#surface').addEventListener('click', () => sendKey(0x72, 'KeyR'));
document.querySelector('#underwater').addEventListener('click', () => sendKey(0x75, 'KeyU'));
document.querySelector('#follow').addEventListener('click', () => sendKey(0x66, 'KeyF'));
document.querySelector('#stop').addEventListener('click', () => { stopMotion(); connection?.focus(); });
document.querySelector('#reconnect').addEventListener('click', connect);
document.querySelector('#fullscreen').addEventListener('click', async () => {
  const stage = document.querySelector('#stage');
  if (document.fullscreenElement) await document.exitFullscreen();
  else if (stage.requestFullscreen) await stage.requestFullscreen();
  connection?.focus();
});
connect();
