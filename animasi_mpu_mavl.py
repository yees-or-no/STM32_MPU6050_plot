"""
Animasi data MPU6050 dari STM32 (USB CDC / Virtual COM Port):
kotak 3D + grafik Kalman vs Complementary.

Dua format input dari STM32:

 1) Teks CSV (bawaan), satu baris per sampel (derajat):
        kalmanX,kalmanY,compX,compY\r\n
    SYARAT di firmware (main.c):  #define PUTTY_DASHBOARD 0

 2) MAVLink (opsi --mavlink), dikirim oleh mavlink_out.c:
        ATTITUDE    -> sudut Kalman (roll, pitch, radian)
        DEBUG_VECT  -> name "COMP", x = roll, y = pitch Complementary (derajat)
    Di firmware, jangan kirim baris CSV bersamaan dengan MAVLink (satu aliran saja).

Tutup PuTTY dulu: satu port COM hanya bisa dibuka satu program.

Pakai:
    pip install pyserial matplotlib numpy pymavlink
    python animasi_mpu.py                       # CSV, port STM32 dicari otomatis
    python animasi_mpu.py COM4                  # CSV, port ditentukan sendiri
    python animasi_mpu.py --mavlink             # MAVLink, port STM32 dicari otomatis
    python animasi_mpu.py --mavlink COM4        # MAVLink, port ditentukan sendiri
    python animasi_mpu.py --mavlink udpin:0.0.0.0:14550   # MAVLink lewat UDP (MAVProxy, SITL, mavlink_fake_sender.py)
    python animasi_mpu.py --demo                # data simulasi, tanpa hardware
"""
import sys, threading, time, collections, math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

BAUD = 115200                   # diabaikan oleh USB CDC, tapi pyserial minta nilainya
HIST = 400                      # jumlah titik di grafik (400 titik = 4 detik @100 Hz, 8 detik @50 Hz)
DEMO = "--demo" in sys.argv
MAV = "--mavlink" in sys.argv
PORT = next((a for a in sys.argv[1:] if not a.startswith("--")), None)   # None = cari STM32 otomatis
NET = PORT is not None and PORT.lower().startswith(("udp", "tcp"))       # koneksi jaringan (hanya untuk MAVLink)

KEYS = ("kx", "ky", "cx", "cy")
buf = {k: collections.deque([0.0] * HIST, maxlen=HIST) for k in KEYS}
lock = threading.Lock()
status = {"rx": 0, "msg": ""}


def push(kx, ky, cx, cy):
    with lock:
        for k, v in zip(KEYS, (kx, ky, cx, cy)):
            buf[k].append(v)
        status["rx"] += 1


def reader_serial(ser):
    ser.reset_input_buffer()
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            parts = line.split(",")
            if len(parts) == 4:
                push(*map(float, parts))
            elif line:
                status["msg"] = line[:60]      # mis. pesan "MPU6050 tidak terdeteksi..."
        except ValueError:
            pass                                # baris rusak/terpotong, lewati
        except Exception as e:                  # port dicabut, dsb.
            status["msg"] = f"serial error: {e}"
            time.sleep(0.5)


def reader_mavlink(conn):
    """ATTITUDE (Kalman) + DEBUG_VECT 'COMP' (Complementary). Satu titik grafik dibuat
    setiap ATTITUDE datang, memakai nilai COMP terbaru."""
    comp = None
    while True:
        try:
            msg = conn.recv_match(type=["ATTITUDE", "DEBUG_VECT"], blocking=True, timeout=1)
            if msg is None:
                status["msg"] = "tidak ada pesan MAVLink (cek baud/kabel/firmware)"
                continue
            typ = msg.get_type()
            if typ == "DEBUG_VECT":
                name = msg.name.decode(errors="ignore") if isinstance(msg.name, (bytes, bytearray)) else str(msg.name)
                if name.strip("\x00 ").upper().startswith("COMP"):
                    comp = (msg.x, msg.y)
            elif typ == "ATTITUDE":
                if comp is None:
                    status["msg"] = "menunggu DEBUG_VECT 'COMP' (Complementary)"
                else:
                    status["msg"] = ""
                    push(math.degrees(msg.roll), math.degrees(msg.pitch), comp[0], comp[1])
        except Exception as e:
            status["msg"] = f"mavlink error: {e}"
            time.sleep(0.5)


def reader_demo():
    t0 = time.time()
    while True:
        t = time.time() - t0
        kx, ky = 30 * np.sin(0.8 * t), 45 * np.sin(0.5 * t)
        push(kx, ky, kx + np.random.normal(0, 1.5), ky + np.random.normal(0, 1.5))
        time.sleep(0.01)


if DEMO:
    target, args = reader_demo, ()
else:
    if NET and not MAV:
        sys.exit("Alamat udp:/tcp: hanya bisa dipakai bersama --mavlink.")
    if not NET:
        from serial.tools import list_ports
        if PORT is None:    # nomor COM tidak diberikan: cari STM32 Virtual ComPort (VID 0483, PID 5740)
            PORT = next((p.device for p in list_ports.comports()
                         if p.vid == 0x0483 and p.pid == 0x5740), None)
            if PORT is None:
                ada = ", ".join(p.device for p in list_ports.comports()) or "(tidak ada)"
                sys.exit(f"STM32 tidak ditemukan. Port yang ada: {ada}\n"
                         "Cek kabel USB (harus kabel data) dan pastikan firmware sudah di-flash.")
            print(f"STM32 ditemukan di {PORT}")
    if MAV:
        try:
            from pymavlink import mavutil
        except ImportError:
            sys.exit("Install dulu: pip install pymavlink")
        try:    # dibuka di thread utama: error langsung kelihatan
            conn = mavutil.mavlink_connection(PORT, baud=BAUD)
        except Exception as e:
            sys.exit(f"Gagal membuka {PORT}: {e}\n"
                     "Cek: nomor COM benar? PuTTY / Serial Monitor lain sudah ditutup?")
        target, args = reader_mavlink, (conn,)
    else:
        import serial
        try:
            ser = serial.Serial(PORT, BAUD, timeout=1)   # buka di thread utama: error langsung kelihatan
        except serial.SerialException as e:
            sys.exit(f"Gagal membuka {PORT}: {e}\n"
                     "Cek: nomor COM benar? PuTTY / Serial Monitor lain sudah ditutup?")
        target, args = reader_serial, (ser,)

threading.Thread(target=target, args=args, daemon=True).start()

# ---------- kotak 3D ----------
V = np.array([[x, y, z] for x in (-1, 1) for y in (-.6, .6) for z in (-.15, .15)])
F = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]


def rot(roll, pitch):
    """Matriks rotasi sensor -> dunia.
    Rumus firmware: roll = atan2(Ay, sqrt(Ax^2+Az^2)), pitch = atan2(-Ax, Az)
    hanya konsisten dengan R = Rx(roll) @ Ry(pitch)  (BUKAN Ry @ Rx)."""
    r, p = np.radians(roll), np.radians(pitch)
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    return Rx @ Ry


fig = plt.figure(figsize=(12, 5))
ax3 = fig.add_subplot(121, projection="3d")
ax3.set_xlim(-1.5, 1.5); ax3.set_ylim(-1.5, 1.5); ax3.set_zlim(-1.5, 1.5)
ax3.set_xlabel("X"); ax3.set_ylabel("Y"); ax3.set_zlabel("Z")
box = Poly3DCollection([], facecolor="tab:blue", edgecolor="k", alpha=0.7)
ax3.add_collection3d(box)
title3d = ax3.set_title("")

# ---------- grafik ----------
axg = fig.add_subplot(122)
x = np.arange(HIST)
lkx, = axg.plot(x, [0] * HIST, color="tab:green",  label="Kalman X (roll)")
lky, = axg.plot(x, [0] * HIST, color="tab:olive",  label="Kalman Y (pitch)")
lcx, = axg.plot(x, [0] * HIST, color="tab:blue",   lw=.8, ls="--", label="Comp X")
lcy, = axg.plot(x, [0] * HIST, color="tab:purple", lw=.8, ls="--", label="Comp Y")
axg.set_ylim(-190, 190)         # pitch bisa -180..+180
axg.set_yticks(range(-180, 181, 45))
axg.set_ylabel("derajat"); axg.grid(alpha=.3)
axg.legend(loc="upper left", fontsize=8)
info = axg.text(0.99, 0.02, "", transform=axg.transAxes, ha="right", fontsize=8, color="gray")


def update(_):
    with lock:
        d = {k: list(v) for k, v in buf.items()}
        rx = status["rx"]
    kx, ky = d["kx"][-1], d["ky"][-1]
    P = V @ rot(kx, ky).T
    box.set_verts([P[f] for f in F])
    title3d.set_text(f"Roll {kx:6.1f}°   Pitch {ky:6.1f}°")
    lkx.set_ydata(d["kx"]); lky.set_ydata(d["ky"])
    lcx.set_ydata(d["cx"]); lcy.set_ydata(d["cy"])
    info.set_text(f"sampel diterima: {rx}   {status['msg']}")
    return box, lkx, lky, lcx, lcy


plt.tight_layout()
ani = FuncAnimation(fig, update, interval=40, cache_frame_data=False)
plt.show()