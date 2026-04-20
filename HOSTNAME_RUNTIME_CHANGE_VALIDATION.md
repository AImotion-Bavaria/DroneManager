# Hostname Runtime Change and Validation

Date: 2026-03-18

## Summary
Hardcoded hostnames were removed and replaced with runtime host detection from the current browser/PC host.

Before:
- `M4001-23-01.local`
- `192.168.1.14`

After:
- `window.location.hostname` with `localhost` fallback

This makes websocket targets follow the host used to open the app (for example, `http://<this-pc-ip>:<port>`).

## Code Changes
1. `FRONTEND_UAM/src/components/drones/DroneCard.tsx`
- Replaced hardcoded hostname with:
```ts
const hostname =
  typeof window !== "undefined" && window.location.hostname
    ? window.location.hostname
    : "localhost";
```
- Stream URL now resolves to:
```ts
`ws://${hostname}:${wsPort}`
```

2. `FRONTEND_UAM/src/hooks/useWebSocket.ts`
- Added runtime hostname:
```ts
const hostname =
  typeof window !== 'undefined' && window.location.hostname
    ? window.location.hostname
    : 'localhost';
```
- Replaced hardcoded websocket endpoint with:
```ts
const websocket = new WebSocket(`ws://${hostname}:8765`);
```
- Updated callback dependencies to include `hostname`.

3. `/STREAMERMULTIPLE/index.html`
- Replaced hardcoded IP with:
```js
const hostname = window.location.hostname || 'localhost';
```
- Stream URL remains:
```js
`ws://${hostname}:${config.wsPort}`
```

## Validation Performed
### 1) Previous hardcoded `.local` host check
Command:
```powershell
ping -n 1 M4001-23-01.local; nslookup M4001-23-01.local
```
Result:
- Ping: host not found
- DNS: non-existent domain

Conclusion:
- `M4001-23-01.local` is not valid/reachable on this PC.

### 2) Previous hardcoded IP check
Command:
```powershell
ping -n 1 192.168.1.14;
Test-NetConnection 192.168.1.14 -Port 9991;
Test-NetConnection 192.168.1.14 -Port 9992;
Test-NetConnection 192.168.1.14 -Port 9993;
Test-NetConnection 192.168.1.14 -Port 8765
```
Result:
- Ping: destination host unreachable
- TCP tests: `TcpTestSucceeded: False` for tested ports

Conclusion:
- `192.168.1.14` is not valid/reachable for required websocket ports from this PC.

### 3) Source checks and diagnostics
- Searched source for hardcoded `M4001-23-01.local` and `192.168.1.14` in updated paths: no matches found.
- TypeScript diagnostics for edited frontend files: no errors found.

## Operational Note
With this change, the app connects to websocket services on the same host from which the page is served.

Examples:
- If opened as `http://localhost:3005`, websocket host becomes `localhost`.
- If opened as `http://192.168.1.200:3005`, websocket host becomes `192.168.1.200`.

If websocket servers run on a different machine than the web page host, use a dedicated environment variable approach for explicit host override.
