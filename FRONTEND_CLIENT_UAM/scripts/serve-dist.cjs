const os = require('node:os');
const { spawn } = require('node:child_process');

function getPreferredLanIp() {
  const interfaces = os.networkInterfaces();
  const ipv4 = Object.values(interfaces)
    .flat()
    .filter((network) => network && network.family === 'IPv4' && !network.internal)
    .map((network) => network.address);

  return (
    ipv4.find((address) => address.startsWith('192.')) ||
    ipv4[0] ||
    '127.0.0.1'
  );
}

const host = process.env.SERVE_HOST || getPreferredLanIp();
const port = process.env.SERVE_PORT || '3000';
const bin = process.platform === 'win32' ? 'npx.cmd' : 'npx';
const args = ['serve', '-s', 'dist', '-l', `tcp://${host}:${port}`];

console.log(`Starting static server on http://${host}:${port}`);

const child = spawn(bin, args, {
  stdio: 'inherit',
  shell: false,
});

child.on('exit', (code) => {
  process.exit(code ?? 0);
});
