"""Ground-station client for the drone capture daemon.

STDLIB ONLY, on purpose: this runs on whatever machine flies the drone —
including the Windows ground station, which has no LiveGS venv — so it must
work with a bare `python`.

    python drone_capture_client.py --host 192.168.1.55 start bench01 --scans 10
    python drone_capture_client.py --host 192.168.1.55 repl      # SPACE=shot
    python drone_capture_client.py --host 192.168.1.55 status
    python drone_capture_client.py --host 192.168.1.55 stop
    python drone_capture_client.py --host 192.168.1.55 pull bench01 --dest recordings/bench01

TRANSPORT. The daemon binds loopback on the Jetson, and this client reaches it
with `ssh -W 127.0.0.1:5757 user@host`, which turns the SSH key auth you already
have into a pipe straight to the socket. No listener is exposed on the shared
Wi-Fi, there is no token to distribute or leak, and it needs nothing on either
side beyond stock OpenSSH. `--direct` connects to the TCP port instead, for a
daemon deliberately started with `--bind 0.0.0.0 --token`.
"""

import argparse
import base64
import json
import os
import shutil
import socket
import subprocess
import sys
import time


class Link:
    """One connection to the daemon, over `ssh -W` or plain TCP."""

    def __init__(self, host, user="dronetrekkers", port=5757, direct=False,
                 token=None, timeout=30.0):
        self.host, self.user, self.port = host, user, port
        self.direct, self.token, self.timeout = direct, token, timeout
        self._proc = None
        self._sock = None
        self._buf = b""

    def __enter__(self):
        if self.direct:
            self._sock = socket.create_connection((self.host, self.port),
                                                  timeout=self.timeout)
        else:
            self._proc = subprocess.Popen(
                ["ssh", "-o", "StrictHostKeyChecking=no",
                 "-o", "ConnectTimeout=10",
                 "-W", f"127.0.0.1:{self.port}", f"{self.user}@{self.host}"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        if self.token:
            self.call({"cmd": "hello", "token": self.token})
        return self

    def __exit__(self, *exc):
        try:
            if self._sock:
                self._sock.close()
            if self._proc:
                self._proc.stdin.close()
                self._proc.terminate()
                self._proc.wait(timeout=5)
        except Exception:                                      # noqa: BLE001
            pass

    def _write(self, data):
        if self._sock:
            self._sock.sendall(data)
        else:
            self._proc.stdin.write(data)
            self._proc.stdin.flush()

    def _readline(self):
        while b"\n" not in self._buf:
            chunk = (self._sock.recv(65536) if self._sock
                     else self._proc.stdout.read1(65536))
            if not chunk:
                raise ConnectionError("daemon closed the connection")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line

    def call(self, req):
        self._write((json.dumps(req) + "\n").encode())
        return json.loads(self._readline().decode())


def _print(obj, raw=False):
    if raw or not isinstance(obj, dict):
        print(json.dumps(obj, indent=2))
        return
    if obj.get("ok") is False:
        print(f"REFUSED: {obj.get('reason')}"
              + (f" ({obj.get('detail')})" if obj.get("detail") is not None else ""))
        return
    print(json.dumps(obj, indent=2))


def cmd_repl(link, args):
    """The tripod tool's operator loop, over the wire.

    capture_pairs.py was a cv2.imshow window with SPACE / u / q; a headless
    daemon has no window, so the same keys live here and `preview` fetches a
    JPEG on demand. Keeping the keys identical means the muscle memory from the
    tripod sessions carries straight over.
    """
    print("SPACE/enter = shot   u = force   p = preview   s = status   q = quit")
    last_sync = 0.0
    while True:
        try:
            k = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if time.time() - last_sync > 30:
            link.call({"cmd": "sync", "gcs_utc_ns": time.time_ns()})
            last_sync = time.time()
        if k in ("q", "quit", "exit"):
            return
        if k in ("", " ", "shot"):
            r = link.call({"cmd": "shot", "tag": args.tag})
        elif k == "u":
            r = link.call({"cmd": "shot", "force": True, "tag": args.tag})
        elif k == "p":
            r = link.call({"cmd": "preview"})
            if r.get("ok"):
                out = args.preview_out or "preview.jpg"
                with open(out, "wb") as f:
                    f.write(base64.b64decode(r["jpeg_b64"]))
                print(f"  preview -> {out}  gyro={r.get('gyro_peak')} "
                      f"sat={r.get('saturated_pct'):.2f}% "
                      f"lidar_age={r.get('lidar_age_ms')}")
                continue
        elif k in ("s", "status"):
            r = link.call({"cmd": "status"})
            s, p = r.get("session"), r.get("pose", {})
            print(f"  session={s and s['name']} captures={s and s['captures']} "
                  f"lidar={r['lidar']['frames']}f/{r['lidar']['age_ms']}ms "
                  f"cam={r['camera']['frames']}f pose_link={p.get('link_up')} "
                  f"free={r['disk']['free_gb']}GB")
            continue
        else:
            print("  ?")
            continue
        if r.get("ok"):
            rec = r.get("record") or {}
            print(f"  #{r.get('id')} pts={rec.get('points')} "
                  f"sat={rec.get('saturated_pct', 0):.2f}% "
                  f"depth={rec.get('depth_coverage_pct', 0):.2f}% "
                  f"gyro={rec.get('gyro_peak')} "
                  f"pose={(rec.get('pose') or {}).get('have')}")
        else:
            print(f"  REFUSED: {r.get('reason')} {r.get('detail') or ''}")


def cmd_pull(args):
    """rsync the session off the drone. Separate from the command protocol on
    purpose: bulk transfer belongs to a tool built for it, not to a JSON line
    protocol, and rsync resumes.

    Windows ground stations ship no rsync, and step 6 of the field procedure
    pulls a session while still on site, so a missing rsync falls back to scp
    rather than aborting. Same bytes; it just cannot resume a partial transfer.
    """
    src = f"{args.user}@{args.host}:{args.remote_root}/{args.name}"
    os.makedirs(args.dest, exist_ok=True)
    if shutil.which("rsync"):
        cmd = ["rsync", "-a", "--partial", "--info=progress2",
               src + "/", args.dest]
    elif shutil.which("scp"):
        # The trailing `/.` copies the directory CONTENTS into dest, matching
        # rsync's trailing slash. Plain `src` would nest it as dest/<name>/.
        cmd = ["scp", "-r", "-p", src + "/.", args.dest]
    else:
        raise SystemExit("neither rsync nor scp found on PATH")
    print(" ".join(cmd))
    return subprocess.call(cmd)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--host", default="192.168.1.55")
    p.add_argument("--user", default="dronetrekkers")
    p.add_argument("--port", type=int, default=5757)
    p.add_argument("--direct", action="store_true",
                   help="plain TCP instead of ssh -W (needs --bind/--token)")
    p.add_argument("--token")
    p.add_argument("--raw", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("start")
    s.add_argument("name")
    s.add_argument("--scans", type=int, default=10)
    s.add_argument("--exposure-ms", type=float, default=2.0)
    s.add_argument("--gain", type=float, default=None,
                   help="D455 colour gain 0..128. 64 is UNITY: the lowest gain "
                        "whose white point reaches 255. Below it a saturated "
                        "pixel cannot exceed e.g. 117, so nothing can detect "
                        "the overexposure. Do not lower it for 'headroom'.")
    s.add_argument("--white-balance", type=float, default=None,
                   help="colour temperature in K (2800..6500). Auto-WB is inert "
                        "outdoors and casts daylight magenta; 3700 measured "
                        "neutral on grass.")
    s.add_argument("--no-average", action="store_true")
    s.add_argument("--gate", choices=("off", "warn", "block"), default="warn")
    s.add_argument("--require-pose", action="store_true")
    s.add_argument("--require-fix", action="store_true")

    sh = sub.add_parser("shot")
    sh.add_argument("--force", action="store_true")
    sh.add_argument("--tag")

    mt = sub.add_parser("meter",
                        help="pick the longest exposure that keeps clipped "
                             "pixels under --target-sat, judged on real frames")
    mt.add_argument("--target-sat", type=float, default=0.5)
    mt.add_argument("--gain", type=float, default=64.0,
                    help="64 = unity gain; see --gain on `start`")

    sub.add_parser("stop")
    sub.add_parser("status")
    sub.add_parser("hello")
    sub.add_parser("selftest")

    pv = sub.add_parser("preview")
    pv.add_argument("--out", default="preview.jpg")
    pv.add_argument("--scale", type=float, default=0.5)

    lg = sub.add_parser("log")
    lg.add_argument("--lines", type=int, default=50)

    rp = sub.add_parser("repl")
    rp.add_argument("--tag")
    rp.add_argument("--preview-out", default="preview.jpg")

    pl = sub.add_parser("pull")
    pl.add_argument("name")
    pl.add_argument("--dest", required=True)
    pl.add_argument("--remote-root", default="/home/dronetrekkers/dronecap/sessions")

    args = p.parse_args(argv)

    if args.cmd == "pull":
        return cmd_pull(args)

    with Link(args.host, args.user, args.port, args.direct, args.token) as link:
        if args.cmd == "start":
            req = {"cmd": "start", "name": args.name, "scans": args.scans,
                   "exposure_ms": args.exposure_ms,
                   "average": not args.no_average, "gate": args.gate,
                   **({"gain": args.gain} if args.gain is not None else {}),
                   **({"white_balance": args.white_balance}
                      if args.white_balance is not None else {}),
                   "require_pose": args.require_pose,
                   "require_fix": args.require_fix,
                   # the GCS clock is the good one; the Jetson has no RTC
                   "gcs_utc_ns": time.time_ns(),
                   "gcs_utc_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
            _print(link.call(req), args.raw)
        elif args.cmd == "meter":
            _print(link.call({"cmd": "meter", "target_sat": args.target_sat,
                              "gain": args.gain}), args.raw)
        elif args.cmd == "shot":
            _print(link.call({"cmd": "shot", "force": args.force,
                              "tag": args.tag}), args.raw)
        elif args.cmd == "preview":
            r = link.call({"cmd": "preview", "scale": args.scale})
            if r.get("ok"):
                with open(args.out, "wb") as f:
                    f.write(base64.b64decode(r["jpeg_b64"]))
                r.pop("jpeg_b64")
                r["saved"] = args.out
            _print(r, args.raw)
        elif args.cmd == "log":
            r = link.call({"cmd": "log", "lines": args.lines})
            for line in r.get("lines", []):
                print(line)
        elif args.cmd == "repl":
            cmd_repl(link, args)
        else:
            _print(link.call({"cmd": args.cmd}), args.raw)
    return 0


if __name__ == "__main__":
    sys.exit(main())
