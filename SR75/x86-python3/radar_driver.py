from ctypes import *
import time
import csv
from datetime import datetime
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ----------------- Constants -----------------
VCI_USBCAN2 = 41
STATUS_OK = 1
INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0
TYPE_CANFD = 1
TYPE_CAN = 0

# ----------------- Load Shared Library -----------------
canDLL = cdll.LoadLibrary('./libcontrolcanfd.so')

# ----------------- Structures -----------------
class ZCAN_CAN_FRAME(Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", c_uint, 29),
        ("err", c_uint, 1),
        ("rtr", c_uint, 1),
        ("eff", c_uint, 1),
        ("len", c_ubyte),
        ("__res0", c_ubyte * 3),
        ("data", c_ubyte * 8)
    ]

class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]

class CANFDConfig(Structure):
    _fields_ = [
        ("acc_code",     c_uint),
        ("acc_mask",     c_uint),
        ("abit_timing",  c_uint),
        ("dbit_timing",  c_uint),
        ("brp",          c_uint),
        ("filter",       c_ubyte),
        ("mode",         c_ubyte),
        ("pad",          c_ushort),
        ("reserved",     c_uint)
    ]

class CANUnion(Union):
    _fields_ = [("canfd", CANFDConfig)]

class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [
        ("can_type", c_uint),
        ("config", CANUnion)
    ]

# ----------------- Function Prototypes -----------------
canDLL.ZCAN_OpenDevice.restype = c_void_p
canDLL.ZCAN_SetAbitBaud.argtypes = (c_void_p, c_ulong, c_ulong)
canDLL.ZCAN_SetDbitBaud.argtypes = (c_void_p, c_ulong, c_ulong)
canDLL.ZCAN_SetCANFDStandard.argtypes = (c_void_p, c_ulong, c_ulong)
canDLL.ZCAN_InitCAN.argtypes = (c_void_p, c_ulong, c_void_p)
canDLL.ZCAN_InitCAN.restype = c_void_p
canDLL.ZCAN_StartCAN.argtypes = (c_void_p,)
canDLL.ZCAN_GetReceiveNum.argtypes = (c_void_p, c_ulong)
canDLL.ZCAN_Receive.argtypes = (c_void_p, c_void_p, c_ulong, c_long)

# ----------------- Helper Functions -----------------
def open_device():
    dev = canDLL.ZCAN_OpenDevice(VCI_USBCAN2, 0, 0)
    if dev == INVALID_DEVICE_HANDLE:
        raise RuntimeError("Failed to open CAN device.")
    print("Device opened.")
    return dev

def set_baud(device_handle):
    for ch in range(2):
        if canDLL.ZCAN_SetAbitBaud(device_handle, ch, 1000000) != STATUS_OK:
            raise RuntimeError(f"Failed to set Abit baud for channel {ch}")
        if canDLL.ZCAN_SetDbitBaud(device_handle, ch, 5000000) != STATUS_OK:
            raise RuntimeError(f"Failed to set Dbit baud for channel {ch}")
        '''
        if canDLL.ZCAN_SetCANFDStandard(device_handle, ch, 0) != STATUS_OK:
            raise RuntimeError(f"Failed to set CAN-FD ISO mode on channel {ch}")'''
    print("Baud rate configured.")

def init_channel(device_handle, channel):
    config = ZCAN_CHANNEL_INIT_CONFIG()
    config.can_type = TYPE_CANFD
    cfg = config.config.canfd
    cfg.acc_code = 0
    cfg.acc_mask = 0xFFFFFFFF
    cfg.abit_timing = 0x06000303
    cfg.dbit_timing = 0x06000303
    cfg.brp = 0
    cfg.filter = 0
    cfg.mode = 0
    cfg.pad = 0
    cfg.reserved = 0

    handle = canDLL.ZCAN_InitCAN(device_handle, channel, byref(config))
    if handle == INVALID_CHANNEL_HANDLE:
        raise RuntimeError(f"Failed to init CAN channel {channel}")
    if canDLL.ZCAN_StartCAN(handle) != STATUS_OK:
        raise RuntimeError(f"Failed to start CAN channel {channel}")
    print(f"CAN channel {channel} initialized and started.")
    return handle

# ----------------- CSV Logger Setup -----------------
def setup_csv_logger():
    filename = f"radar_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    file = open(filename, mode='w', newline='')
    writer = csv.writer(file)
    header = [
        "timestamp", "message_type", "can_id", "data_length", "data",
        "Cluster_ID", "DistLong_m", "DistLat_m", "rad_vel",
        "Height_m", "DynProp", "SNR",
        "RadialDistance_m", "Angle_deg"
    ]
    writer.writerow(header)
    return file, writer

# ----------------- Decoding CAN 0x701 -----------------
def parse_0x701_hex(data):
    if len(data) != 8:
        raise ValueError("Data must be 8 bytes")

    B = list(data)
    # 1. cluster ID
    cluster_id = B[0]
    # 2. Longitudinal Distance
    dist_long_raw = B[1] * 32 + (B[2] >> 3)
    dist_long = dist_long_raw * 0.05 - 100
    # 3. Lateral Distance
    dist_lat_raw = ((B[2] & 0x07) * 256) + B[3]
    dist_lat = dist_lat_raw * 0.05 - 50
    # 4. Radial Velocity
    rad_vel_raw = B[4] * 4 + (B[5] >> 6)
    rad_vel = rad_vel_raw * 0.25 - 128
    # 5. Height - Altitude
    height_raw = (B[5] & 0x3F) * 8 + (B[6] >> 5)
    height = height_raw * 0.25 - 64
    # 6. Dynamic attribute
    dyn_prop = B[6] & 0x07
    # 7. Signal to Noise Ratio
    snr = B[7]
    # 8. Radial Distance
    radial_distance = math.hypot(dist_long, dist_lat)
    # 9. Target Angle
    angle_rad = math.atan2(dist_lat, dist_long)
    angle_deg = math.degrees(angle_rad)

    print("Distanece : ", dist_lat, " , " , dist_long)

    return {
        "Cluster_ID": cluster_id,
        "DistLong_m": dist_long,
        "DistLat_m": dist_lat,
        "Rad_vel": rad_vel,
        "Height_m": height,
        "DynProp": dyn_prop,
        "SNR": snr,
        "RadialDistance_m": radial_distance,
        "Angle_deg": angle_deg
    }

# ----------------- Listening and Visualization -----------------
def listen_to_radar(channel_handle):
    print("Listening to radar data (press Ctrl+C to stop)...")
    file, writer = setup_csv_logger()

    buffer = []
    last_plot_time = time.time()

    # Setup plot
    plt.ion()
    fig = plt.figure(figsize=(200, 20))
    ax = fig.add_subplot(111, projection='3d')

    try:
        while True:
            can_count = canDLL.ZCAN_GetReceiveNum(channel_handle, TYPE_CAN)
            if can_count > 0:
                rcv_data = (ZCAN_Receive_Data * can_count)()
                num = canDLL.ZCAN_Receive(channel_handle, byref(rcv_data), can_count, -1)

                for i in range(num):
                    frame = rcv_data[i].frame
                    data_len = frame.len
                    can_id_int = frame.can_id
                    timestamp = rcv_data[i].timestamp
                    data_bytes = list(frame.data[:data_len])

                    parsed_fields = [""] * 10

                    if can_id_int == 0x701 and data_len == 8:
                        try:
                            info = parse_0x701_hex(data_bytes)
                            parsed_fields = [
                                info["Cluster_ID"],
                                info["DistLong_m"],
                                info["DistLat_m"],
                                info["Rad_vel"],
                                info["Height_m"],
                                info["DynProp"],
                                info["SNR"],
                                info["RadialDistance_m"],
                                info["Angle_deg"]
                            ]
                            buffer.append(info)
                        except Exception as e:
                            print(f"Error decoding CAN 0x701 cluster: {e}")

                    writer.writerow([
                        timestamp,
                        "CAN",
                        f"0x{can_id_int:X}",
                        data_len,
                        ",".join(map(str, data_bytes))
                    ] + parsed_fields)

            # Every 3 seconds, update point cloud
            if time.time() - last_plot_time > 3:
                if buffer:
                    ax.clear()
                    x = [p["DistLong_m"] for p in buffer]
                    y = [p["DistLat_m"] for p in buffer]
                    z = [p["Height_m"] for p in buffer]
                    c = [p["Rad_vel"] for p in buffer]

                    scatter = ax.scatter(x, y, z, c=c, cmap='jet', s=10)
                    ax.set_xlabel("DistLong (m)")
                    ax.set_ylabel("DistLat (m)")
                    ax.set_zlabel("Height (m)")
                    ax.set_title("Live Radar Point Cloud")
                    #fig.colorbar(scatter, ax=ax, label="VrelLong (m/s)")
                    plt.draw()
                    plt.pause(0.01)
                    #buffer.clear()
                last_plot_time = time.time()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Radar listener stopped.")
    finally:
        file.close()

# ----------------- Main Entry -----------------
def main():
    device = open_device()
    set_baud(device)
    can_channel = init_channel(device, 0)
    listen_to_radar(can_channel)

if __name__ == "__main__":
    main()

