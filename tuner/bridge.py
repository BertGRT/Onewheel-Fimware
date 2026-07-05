#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Passerelle Onewheel : interface web de reglage via le ST-Link (SWD).

Fait tourner OpenOCD (attache la cible SANS reset), lit la telemetrie et ecrit
les reglages en memoire (SWD), et sert l'interface web (sliders + graphes).
=> l'interface graphique de reglage, avec le ST-Link seul. Aucun FTDI.

Lancer :  python C:\\onewheel\\tuner\\bridge.py
Puis ouvrir dans le navigateur l'URL affichee (http://127.0.0.1:8666/).
"""
import os, re, socket, subprocess, threading, json, time, sys, traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge.log")
def logerr(where):
    try:
        with open(LOG, "a", encoding="utf-8") as f:
            f.write("---- %s @ %.1f ----\n%s\n" % (where, time.time(), traceback.format_exc()))
    except Exception: pass

# ------------------------------------------------------------------ chemins
OCD_BASE = r"C:\onewheel\ocd\xpack-openocd-0.12.0-7"
OPENOCD  = os.path.join(OCD_BASE, r"bin\openocd.exe")
SCRIPTS  = os.path.join(OCD_BASE, r"openocd\scripts")
NM       = r"C:\onewheel\arm\xpack-arm-none-eabi-gcc-15.2.1-1.1\bin\arm-none-eabi-nm.exe"
ELF      = r"C:\onewheel\foc\build\hover_ow.elf"
HTML     = os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html")

HTTP_PORT = 8666
RPC_PORT  = 6666

# ---- Champs de ow_config (offset int16 depuis la base de la struct) ----
CONFIG_FIELDS = {
    'kp':4, 'ki':6, 'kd':8, 'setpoint_trim':10, 'start_angle_max':12,
    'fault_angle_max':14, 'output_max':16, 'output_ramp':18, 'current_limit':20,
    'tiltback_speed':22, 'tiltback_angle':24, 'lowbat_voltage':26, 'cutoff_voltage':28,
    'imu_alpha':30, 'pitch_invert':32, 'output_invert':34, 'footpad_active_low':36,
    'deadband':38, 'startup_beep':40, 'angle_expo':42, 'start_ramp':44,
}

# --------------------------------------------------------------- symboles ELF
def read_symbols():
    out = subprocess.check_output([NM, ELF], text=True, errors='replace')
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3:
            syms[p[2]] = int(p[0], 16)
    need = ['ow_config','ow_pitch_x100','ow_state','ow_out','ow_armed','ow_footpad',
            'ow_manual','mpu_ok_count','mpu_fail_count','ow_save_req','ow_save_ack']
    for n in need:
        if n not in syms:
            print("!! symbole manquant:", n)
    return syms

SYM = read_symbols()
CFG_BASE = SYM['ow_config']

# --------------------------------------------------------------- OpenOCD RPC
class OpenOCD:
    def __init__(self, host='127.0.0.1', port=RPC_PORT):
        self.host = host; self.port = port
        self.lock = threading.Lock()
        self.sock = None
        self._connect()
    def _connect(self):
        if self.sock:
            try: self.sock.close()
            except Exception: pass
        self.sock = socket.create_connection((self.host, self.port), timeout=8)
        self.sock.settimeout(8)
    def _raw(self, c):
        self.sock.sendall(c.encode() + b'\x1a')
        buf = b''
        while not buf.endswith(b'\x1a'):
            d = self.sock.recv(65536)
            if not d:
                raise IOError("openocd rpc closed")
            buf += d
        return buf[:-1].decode(errors='replace').strip()
    def cmd(self, c):
        with self.lock:
            try:
                return self._raw(c)
            except Exception:
                # desync/timeout -> reconnexion propre + 1 reessai
                self._connect()
                return self._raw(c)
    def read_words(self, addrs):
        """Lit plusieurs mots alignes en UNE commande. Renvoie {aligned_addr: val}."""
        al = sorted(set(a & ~3 for a in addrs))
        expr = 'format "' + ' '.join('%d' for _ in al) + '" ' + \
               ' '.join('[mrw 0x%08x]' % a for a in al)
        r = self.cmd(expr)
        parts = r.split()
        vals = {}
        for a, p in zip(al, parts):
            try: vals[a] = int(p, 0) & 0xFFFFFFFF
            except Exception: vals[a] = 0
        return vals
    def mrw(self, addr):
        return self.read_words([addr]).get(addr & ~3, 0)
    def mwh(self, addr, val): self.cmd("mwh 0x%08x %d" % (addr, val & 0xFFFF))
    def mwb(self, addr, val): self.cmd("mwb 0x%08x %d" % (addr, val & 0xFF))

def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v

def _e16(words, addr): return s16((words.get(addr & ~3, 0) >> ((addr & 2) * 8)) & 0xFFFF)
def _e8 (words, addr): return (words.get(addr & ~3, 0) >> ((addr & 3) * 8)) & 0xFF
def _e32(words, addr): return words.get(addr & ~3, 0)

OCD = None  # set after openocd is up

# --------------------------------------------------------------- lecture etat
STATE_NAMES = {0:'IDLE',1:'ARME',2:'RIDING',3:'TILTBACK',4:'FAULT',65535:'MANUEL',-1:'MANUEL'}

def get_telemetry():
    addrs = [SYM['ow_pitch_x100'], SYM['ow_state'], SYM['ow_out'], SYM['ow_armed'],
             SYM['ow_footpad'], SYM['mpu_ok_count'], SYM['mpu_fail_count']]
    w = OCD.read_words(addrs)
    state = _e16(w, SYM['ow_state'])
    return {'pitch': _e16(w, SYM['ow_pitch_x100'])/100.0,
            'state': state, 'state_name': STATE_NAMES.get(state, str(state)),
            'out': _e16(w, SYM['ow_out']),
            'armed': _e8(w, SYM['ow_armed']),
            'footpad': _e8(w, SYM['ow_footpad']),
            'i2c_ok': _e32(w, SYM['mpu_ok_count']),
            'i2c_fail': _e32(w, SYM['mpu_fail_count'])}

def get_config():
    addrs = [CFG_BASE + off for off in CONFIG_FIELDS.values()]
    w = OCD.read_words(addrs)
    return {name: _e16(w, CFG_BASE + off) for name, off in CONFIG_FIELDS.items()}

def set_value(name, value):
    value = int(value)
    if name == 'armed':   OCD.mwb(SYM['ow_armed'],   1 if value else 0); return
    if name == 'footpad': OCD.mwb(SYM['ow_footpad'], 1 if value else 0); return
    if name == 'manual':  OCD.mwh(SYM['ow_manual'],  value); return
    if name in CONFIG_FIELDS:
        OCD.mwh(CFG_BASE + CONFIG_FIELDS[name], value); return
    raise KeyError(name)

# --------------------------------------------------------------- sauvegarde flash
def save_config():
    """Demande au firmware d'ecrire ow_config en flash. L'ecriture gele le CPU
    ~40ms : on pousse ow_save_req=1 puis on attend qu'il repasse a 0 (traite)."""
    OCD.mwb(SYM['ow_save_req'], 1)
    for _ in range(30):                 # ~1.5s max
        time.sleep(0.05)
        try:
            w   = OCD.read_words([SYM['ow_save_req'], SYM['ow_save_ack']])
            req = _e8(w, SYM['ow_save_req'])
            ack = _e8(w, SYM['ow_save_ack'])
        except Exception:
            continue
        if req == 0:
            msg = {1:'sauvegarde OK', 2:'echec ecriture flash',
                   3:'refuse : coupe les moteurs (pied leve) avant de sauver'}.get(ack, 'inconnu')
            return {'ok': ack == 1, 'ack': ack, 'msg': msg}
    return {'ok': False, 'ack': 0, 'msg': 'timeout (firmware occupe ?)'}

# --------------------------------------------------------------- serveur HTTP
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code); self.send_header('Content-Type','application/json')
        self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        try:
            if self.path == '/' or self.path.startswith('/index'):
                with open(HTML,'rb') as f: b = f.read()
                self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
            elif self.path == '/api/telemetry':
                self._json(get_telemetry())
            elif self.path == '/api/config':
                self._json(get_config())
            else:
                self._json({'error':'not found'}, 404)
        except Exception as e:
            self._json({'error':str(e), 'trace':traceback.format_exc()}, 500)
    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length', 0))
            data = json.loads(self.rfile.read(n) or b'{}')
            if self.path == '/api/set':
                set_value(data['name'], data['value'])
                self._json({'ok':True})
            elif self.path == '/api/save':
                self._json(save_config())
            else:
                self._json({'error':'not found'}, 404)
        except Exception as e:
            self._json({'error':str(e)}, 500)

# --------------------------------------------------------------- demarrage
def kill_openocd():
    try: subprocess.run(["taskkill","/IM","openocd.exe","/F"],
                        capture_output=True); time.sleep(0.5)
    except Exception: pass

def start_openocd():
    # reset run : garantit que le firmware TOURNE (sinon CPU halte -> angle fige).
    args = [OPENOCD, "-s", SCRIPTS,
            "-f","interface/stlink.cfg","-f","target/stm32f1x.cfg",
            "-c","init","-c","reset run","-c","poll off"]
    p = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return p

def main():
    global OCD
    print("=== Passerelle Onewheel (ST-Link -> interface web) ===")
    print("ow_config @ 0x%08x" % CFG_BASE)
    kill_openocd()
    print("Demarrage OpenOCD...")
    ocd_proc = start_openocd()
    # attendre le RPC
    for _ in range(40):
        try:
            OCD = OpenOCD(); break
        except Exception:
            time.sleep(0.25)
    if OCD is None:
        print("!! Impossible de joindre OpenOCD (ST-Link branche ? carte alimentee ?)");
        ocd_proc.terminate(); return
    try:
        t = get_telemetry()
        print("Cible OK. pitch=%.2f state=%s" % (t['pitch'], t['state_name']))
    except Exception as e:
        print("Lecture cible echouee:", e)
    srv = ThreadingHTTPServer(('127.0.0.1', HTTP_PORT), Handler)
    print("\n>>> Ouvre dans ton navigateur :  http://127.0.0.1:%d/\n" % HTTP_PORT)
    print("(Ctrl+C pour arreter)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        ocd_proc.terminate()

if __name__ == '__main__':
    main()
