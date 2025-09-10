import os
import time
import can
import threading

# --------------------- CAN1 RECORDER ---------------------
def record_can1_task(bus: can.BusABC, stop_ev: threading.Event, trc_path="can1_log.trc"):
    """
    Reads raw CAN frames from `bus` (expect can1) until stop_ev is set.
    Prints each frame and appends to .trc (Vector ASCII Trace v1.1).
    """

    # Open file for write
    need_header = not os.path.exists(trc_path)
    f = open(trc_path, "w")

    if need_header:
        start_iso = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        f.write("; Version = 1.1\n")
        f.write("; Creator = python-can recorder\n")
        f.write(f"; StartTime = {start_iso}\n")
        f.write("; Columns = TimeOffset Channel ID Dir DLC Data\n")
        f.write("Begin Triggerblock\n")
        f.flush()

    t0 = time.time()

    try:
        while not stop_ev.is_set():
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue

            ts_epoch = float(msg.timestamp) if hasattr(msg, "timestamp") else time.time()
            offset = ts_epoch - t0

            arb_hex = f"{msg.arbitration_id:03X}" if not msg.is_extended_id else f"{msg.arbitration_id:08X}"
            dlc = msg.dlc if hasattr(msg, "dlc") else len(msg.data)
            data_bytes = " ".join(f"{b:02X}" for b in msg.data)

            # Print to console
            print(f"{offset:9.6f}  can1  {arb_hex}  dlc={dlc}  data=[{data_bytes}]")

            # Write TRC line
            f.write(f"{offset:9.6f} can1 {arb_hex} Rx d {dlc}  {data_bytes}\n")
            f.flush()

    finally:
        f.close()


# -------------------- Minimal MAIN ----------------------
BITRATE = 500000
BUSTYPE = "socketcan"

def make_can1_bus():
    return can.interface.Bus(bustype=BUSTYPE, channel="can1", bitrate=BITRATE)

def main():
    bus_can1 = make_can1_bus()
    stop_ev = threading.Event()

    rec_thr = threading.Thread(target=record_can1_task, args=(bus_can1, stop_ev, "can1_log.trc"), daemon=True)
    rec_thr.start()
    print("Recording raw CAN from can1 → can1_log.trc. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_ev.set()
        rec_thr.join(timeout=1.0)
        bus_can1.shutdown()

if __name__ == "__main__":
    main()
