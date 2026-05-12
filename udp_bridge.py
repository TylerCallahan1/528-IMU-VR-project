
"""
  python udp_bridge.py --port COM5 --verbose
  python udp_bridge.py --port COM5 --beta 0.05
  python udp_bridge.py --port COM5 --accel-threshold 0.04
"""

import argparse
import math
import re
import socket
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial not installed.")
    sys.exit(1)

# ── Constants ──────────────────────────────────────────────────────────────────
BAUD_RATE    = 115200
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
FS           = 100.0
G_MS2        = 9.81   # 1 g in m/s²

LINE_RE = re.compile(
    r"AX:(?P<ax>[-\d.]+)\s+AY:(?P<ay>[-\d.]+)\s+AZ:(?P<az>[-\d.]+)"
    r"\s*\|\s*"
    r"GX:(?P<gx>[-\d.]+)\s+GY:(?P<gy>[-\d.]+)\s+GZ:(?P<gz>[-\d.]+)"
)


# ── Madgwick Filter ────────────────────────────────────────────────────────────

class MadgwickFilter:
    """
    Fuses accel + gyro into a unit quaternion representing orientation.
    Exposes the quaternion directly so PositionEstimator can use it
    to rotate acceleration into the world frame.
    """
    def __init__(self, beta=0.033):
        self.beta = beta
        self.q = [1.0, 0.0, 0.0, 0.0]   # [w, x, y, z]

    def reset(self):
        self.q = [1.0, 0.0, 0.0, 0.0]

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """Update and return (roll, pitch, yaw) in degrees."""
        q1, q2, q3, q4 = self.q
        gx = math.radians(gx); gy = math.radians(gy); gz = math.radians(gz)
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-10:
            return self._euler()
        ax /= norm; ay /= norm; az /= norm

        _2q1=2*q1;_2q2=2*q2;_2q3=2*q3;_2q4=2*q4
        _4q1=4*q1;_4q2=4*q2;_4q3=4*q3
        _8q2=8*q2;_8q3=8*q3
        q1q1=q1*q1;q2q2=q2*q2;q3q3=q3*q3;q4q4=q4*q4

        s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay
        s2 = _4q2*q4q4-_2q4*ax+4*q1q1*q2-_2q1*ay-_4q2+_8q2*q2q2+_8q2*q3q3+_4q2*az
        s3 = 4*q1q1*q3+_2q1*ax+_4q3*q4q4-_2q4*ay-_4q3+_8q3*q2q2+_8q3*q3q3+_4q3*az
        s4 = 4*q2q2*q4-_2q2*ax+4*q3q3*q4-_2q3*ay
        ns = math.sqrt(s1*s1+s2*s2+s3*s3+s4*s4)
        if ns > 1e-10:
            s1/=ns; s2/=ns; s3/=ns; s4/=ns

        qd1=0.5*(-q2*gx-q3*gy-q4*gz)-self.beta*s1
        qd2=0.5*( q1*gx+q3*gz-q4*gy)-self.beta*s2
        qd3=0.5*( q1*gy-q2*gz+q4*gx)-self.beta*s3
        qd4=0.5*( q1*gz+q2*gy-q3*gx)-self.beta*s4

        q1+=qd1*dt; q2+=qd2*dt; q3+=qd3*dt; q4+=qd4*dt
        nq = math.sqrt(q1*q1+q2*q2+q3*q3+q4*q4)
        self.q = [q1/nq, q2/nq, q3/nq, q4/nq]
        return self._euler()

    def rotate_to_world(self, sx, sy, sz):
        """
        Rotate a sensor-frame vector [sx, sy, sz] into the world frame
        """
        q1, q2, q3, q4 = self.q
        # Equivalent matrix rotation (faster than full quaternion product)
        wx = (1 - 2*(q3*q3 + q4*q4))*sx + 2*(q2*q3 - q1*q4)*sy + 2*(q2*q4 + q1*q3)*sz
        wy = 2*(q2*q3 + q1*q4)*sx + (1 - 2*(q2*q2 + q4*q4))*sy + 2*(q3*q4 - q1*q2)*sz
        wz = 2*(q2*q4 - q1*q3)*sx + 2*(q3*q4 + q1*q2)*sy + (1 - 2*(q2*q2 + q3*q3))*sz
        return wx, wy, wz

    def _euler(self):
        q1, q2, q3, q4 = self.q
        roll  = math.degrees(math.atan2(2*(q1*q2+q3*q4), 1-2*(q2*q2+q3*q3)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(q1*q3-q4*q2)))))
        yaw   = math.degrees(math.atan2(2*(q1*q4+q2*q3), 1-2*(q3*q3+q4*q4)))
        return roll, pitch, yaw



class PositionEstimator:
    """
    accel_thresh : float
        Linear acceleration threshold in m/s² below which we treat a
        sample as "no movement" for ZUPT purposes. Default: 0.3 m/s².
    gyro_thresh : float
        Gyro magnitude threshold in deg/s for ZUPT. Default: 8.0 deg/s.
    decay : float
        Per-sample velocity decay factor. 0.98-0.995 is a good range.
    """

    def __init__(self, accel_thresh=0.3, gyro_thresh=8.0, decay=0.98):
        self.accel_thresh = accel_thresh   # m/s²
        self.gyro_thresh  = gyro_thresh    # deg/s
        self.decay        = decay
        self.vx = self.vy = self.vz = 0.0
        self.px = self.py = self.pz = 0.0

    def reset(self):
        self.vx = self.vy = self.vz = 0.0
        self.px = self.py = self.pz = 0.0

    def update(self, madgwick: MadgwickFilter,
               ax_g, ay_g, az_g,
               gx_ds, gy_ds, gz_ds,
               dt) -> tuple:
        """
        madgwick   : MadgwickFilter instance (must be updated first this step)
        ax_g ay_g az_g : raw accelerometer in g
        gx_ds gy_ds gz_ds : raw gyroscope in deg/s
        dt         : timestep in seconds

        Returns (x, y, z) in meters.
        """
        # 1. Convert accel to m/s²
        ax_ms2 = ax_g * G_MS2
        ay_ms2 = ay_g * G_MS2
        az_ms2 = az_g * G_MS2

        # 2. Rotate sensor-frame accel into world frame
        wx, wy, wz = madgwick.rotate_to_world(ax_ms2, ay_ms2, az_ms2)

        # 3. Subtract gravity (world Z is up → gravity is -9.81 on Z)
        #    MPU6050 at rest reads ~+1g on whichever axis is "up"
        #    After Madgwick, world Z always aligns with gravity direction
        lx = wx
        ly = wy
        lz = wz - G_MS2   # remove gravity

        # 4. Deadband — kill quantization noise below threshold
        def deadband(v, t):
            return v if abs(v) > t else 0.0
        db = self.accel_thresh
        lx = deadband(lx, db)
        ly = deadband(ly, db)
        lz = deadband(lz, db)

        # 5. ZUPT — if sensor is stationary, snap velocity to zero
        accel_mag = math.sqrt(lx*lx + ly*ly + lz*lz)
        gyro_mag  = math.sqrt(gx_ds*gx_ds + gy_ds*gy_ds + gz_ds*gz_ds)
        is_still  = (accel_mag < self.accel_thresh) and (gyro_mag < self.gyro_thresh)

        if is_still:
            self.vx = self.vy = self.vz = 0.0
        else:
            # 6. Integrate velocity
            self.vx += lx * dt
            self.vy += ly * dt
            self.vz += lz * dt

            # 7. Velocity decay (soft drift suppression between movements)
            self.vx *= self.decay
            self.vy *= self.decay
            self.vz *= self.decay

        # 8. Integrate position
        self.px += self.vx * dt
        self.py += self.vy * dt
        self.pz += self.vz * dt

        return self.px, self.py, self.pz



def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        d = (p.description or "").lower()
        if any(k in d for k in ("cp210", "ch340", "usb serial", "uart", "ftdi")):
            return p.device
        if "usb" in p.device.lower():
            return p.device
    if ports:
        print(f"[WARN] Using first port: {ports[0].device}")
        return ports[0].device
    print("[ERROR] No serial port found. Use --port.", file=sys.stderr)
    sys.exit(1)


def parse_line(line):
    m = LINE_RE.search(line)
    if m:
        return tuple(float(m.group(k)) for k in ("ax","ay","az","gx","gy","gz"))
    return None


def make_packet(roll, pitch, yaw, x, y, z, ax, ay, az, gx, gy, gz):
    """
    UDP packet — comma-separated floats, newline-terminated.

    Format: roll,pitch,yaw,x,y,z,ax,ay,az,gx,gy,gz
    C++ side: sscanf(buf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", ...)
    """
    return (f"{roll:.4f},{pitch:.4f},{yaw:.4f},"
            f"{x:.5f},{y:.5f},{z:.5f},"
            f"{ax:.4f},{ay:.4f},{az:.4f},"
            f"{gx:.4f},{gy:.4f},{gz:.4f}\n").encode("ascii")



def run(port, baud, host, udp_port, beta, accel_thresh, decay, verbose):
    madgwick = MadgwickFilter(beta=beta)
    estimator = PositionEstimator(accel_thresh=accel_thresh, decay=decay)

    sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest   = (host, udp_port)
    t_prev = None
    count  = 0

    print(f"[BRIDGE] Serial   : {port} @ {baud} baud")
    print(f"[BRIDGE] UDP out  : {host}:{udp_port}")
    print(f"[BRIDGE] Filter   : Madgwick beta={beta}")
    print(f"[BRIDGE] Position : accel_thresh={accel_thresh} m/s²  decay={decay}")
    print(f"[BRIDGE] Packet   : roll,pitch,yaw,x,y,z,ax,ay,az,gx,gy,gz")
    print(f"[BRIDGE] Ctrl-C to stop.\n")

    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            ser.reset_input_buffer()
            while True:
                raw   = ser.readline()
                t_now = time.perf_counter()
                line  = raw.decode("utf-8", errors="replace").strip()
                data  = parse_line(line)
                if data is None:
                    continue

                ax, ay, az, gx, gy, gz = data
                dt = (1.0/FS) if t_prev is None else min(max(t_now-t_prev, 0.001), 0.1)
                t_prev = t_now

                # Orientation first (Madgwick must run before rotate_to_world)
                roll, pitch, yaw = madgwick.update(ax, ay, az, gx, gy, gz, dt)

                # Position from same sample
                x, y, z = estimator.update(madgwick, ax, ay, az, gx, gy, gz, dt)

                sock.sendto(make_packet(roll, pitch, yaw, x, y, z,
                                        ax, ay, az, gx, gy, gz), dest)
                count += 1

                if verbose:
                    print(f"  R:{roll:6.1f} P:{pitch:6.1f} Y:{yaw:6.1f} | "
                          f"X:{x:6.3f} Y:{y:6.3f} Z:{z:6.3f} m  [{count}]",
                          end="\r")

    except KeyboardInterrupt:
        print(f"\n[BRIDGE] Stopped. {count} packets sent.")
    except serial.SerialException as e:
        print(f"\n[ERROR] Serial: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        sock.close()


def main():
    p = argparse.ArgumentParser(description="IMU-to-OpenGL UDP bridge with position")
    p.add_argument("--port",           default=None,         help="Serial port")
    p.add_argument("--baud",           default=BAUD_RATE,    type=int)
    p.add_argument("--host",           default=DEFAULT_HOST)
    p.add_argument("--udp-port",       default=DEFAULT_PORT, type=int)
    p.add_argument("--beta",           default=0.033,        type=float,
                   help="Madgwick gain (default 0.033)")
    p.add_argument("--accel-threshold",default=0.3,          type=float,
                   help="ZUPT accel threshold m/s² (default 0.3)")
    p.add_argument("--decay",          default=0.98,         type=float,
                   help="Velocity decay per sample (default 0.98)")
    p.add_argument("--verbose",        action="store_true")
    args = p.parse_args()

    run(args.port or find_port(), args.baud, args.host, args.udp_port,
        args.beta, args.accel_threshold, args.decay, args.verbose)


if __name__ == "__main__":
    main()
