import RFB from '/novnc/core/rfb.js';

const screen = document.querySelector('#screen');
const status = document.querySelector('#connection');
const notice = document.querySelector('#notice');
const controls = [document.querySelector('#surface'), document.querySelector('#underwater')];
let connection;
let retry;

function connect() {
  clearTimeout(retry);
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
    status.dataset.state = 'offline';
    status.textContent = 'Disconnected';
    notice.hidden = false;
    notice.textContent = 'Connection lost. Reconnecting…';
    controls.forEach(button => { button.disabled = true; });
    retry = setTimeout(connect, 4000);
  });
}

function sendKey(key, code) {
  connection?.sendKey(key, code, true);
  connection?.sendKey(key, code, false);
  connection?.focus();
}

document.querySelector('#surface').addEventListener('click', () => sendKey(0x72, 'KeyR'));
document.querySelector('#underwater').addEventListener('click', () => sendKey(0x75, 'KeyU'));
document.querySelector('#reconnect').addEventListener('click', connect);
document.querySelector('#fullscreen').addEventListener('click', async () => {
  const stage = document.querySelector('#stage');
  if (document.fullscreenElement) await document.exitFullscreen();
  else if (stage.requestFullscreen) await stage.requestFullscreen();
  connection?.focus();
});
connect();
