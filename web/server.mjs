import { createHash } from 'node:crypto';
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { createConnection } from 'node:net';
import { dirname, extname, join, normalize } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = dirname(fileURLToPath(import.meta.url));
const webPort = Number(process.env.WEB_PORT || 8080);
const hubHost = process.env.HUB_HOST || '127.0.0.1';
const hubPort = Number(process.env.HUB_PORT || 9095);
const types = { '.css': 'text/css; charset=utf-8', '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8' };

function frame(text) {
  const body = Buffer.from(text);
  if (body.length < 126) return Buffer.concat([Buffer.from([0x81, body.length]), body]);
  return Buffer.concat([Buffer.from([0x81, 126, body.length >> 8, body.length & 255]), body]);
}
function bridge(browser) {
  let input = Buffer.alloc(0), hubInput = '', closed = false;
  const hub = createConnection({ host: hubHost, port: hubPort });
  const send = object => { if (!closed) browser.write(frame(JSON.stringify(object))); };
  const close = () => { if (!closed) { closed = true; hub.destroy(); browser.destroy(); } };
  hub.on('connect', () => send({ op: 'bridge_status', connected: true }));
  hub.on('error', error => send({ op: 'bridge_status', connected: false, message: error.message }));
  hub.on('close', () => send({ op: 'bridge_status', connected: false }));
  hub.on('data', bytes => {
    hubInput += bytes.toString();
    for (;;) {
      const end = hubInput.indexOf('\n'); if (end < 0) return;
      const line = hubInput.slice(0, end); hubInput = hubInput.slice(end + 1);
      try { if (line) send(JSON.parse(line)); } catch { send({ op: 'bridge_error', message: 'Invalid message from hub' }); }
    }
  });
  browser.on('data', bytes => {
    input = Buffer.concat([input, bytes]);
    while (input.length >= 2) {
      const opcode = input[0] & 15; let length = input[1] & 127; let offset = 2;
      if (length === 126) { if (input.length < 4) return; length = input.readUInt16BE(2); offset = 4; }
      if ((input[1] & 128) === 0 || length > 1024 * 1024 || input.length < offset + 4 + length) { if ((input[1] & 128) === 0 || length > 1024 * 1024) close(); return; }
      const mask = input.subarray(offset, offset + 4); offset += 4;
      const payload = Buffer.from(input.subarray(offset, offset + length)); input = input.subarray(offset + length);
      for (let i = 0; i < payload.length; i++) payload[i] ^= mask[i % 4];
      if (opcode === 8) return close();
      if (opcode !== 1) continue;
      try { hub.write(`${JSON.stringify(JSON.parse(payload.toString()))}\n`); }
      catch { send({ op: 'bridge_error', message: 'Invalid browser message' }); }
    }
  });
  browser.on('close', close); browser.on('error', close);
}
const server = createServer(async (request, response) => {
  const name = request.url === '/' ? 'index.html' : request.url.split('?')[0].replace(/^\/+/, '');
  const file = normalize(join(root, name));
  if (!file.startsWith(`${root}/`)) return response.writeHead(403).end();
  try { response.writeHead(200, { 'content-type': types[extname(file)] || 'application/octet-stream', 'cache-control': 'no-store' }); response.end(await readFile(file)); }
  catch { response.writeHead(404).end('Not found'); }
});
server.on('upgrade', (request, socket) => {
  if (request.url !== '/ws' || request.headers.upgrade?.toLowerCase() !== 'websocket' || !request.headers['sec-websocket-key']) return socket.destroy();
  const key = createHash('sha1').update(`${request.headers['sec-websocket-key']}258EAFA5-E914-47DA-95CA-C5AB0DC85B11`).digest('base64');
  socket.write(`HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ${key}\r\n\r\n`); bridge(socket);
});
server.listen(webPort, '127.0.0.1', () => console.log(`Open http://127.0.0.1:${webPort}; connecting to ${hubHost}:${hubPort}`));
