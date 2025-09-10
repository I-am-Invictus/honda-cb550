import csv, os, time
import can
import threading

# --------------------- CAN1 RECORDER ---------------------
def record_can1_task(bus: can.BusABC, stop_ev: threading.Event, csv_path="can1_log.csv"):
    """
    Reads raw CAN frames from `bus` (expect can1) until stop_ev is set.
    Prints each frame and appends to CSV.

    CSV columns:
      ts_iso, ts_epoch, channel, arbitration_id_hex, is_extended, dlc, data_hex
    """
    # Prepare CSV (append; create header if new)
    need_header = not os.path.exists(csv_path)
    f = open(csv_path, "w", newline="")
    writer = csv.writer(f)
    if need_header:
        writer.writerow(["ts_iso","ts_epoch","channel","arbitration_id_hex","is_extended","dlc","data_hex"])
        f.flush()

    try:
        while not stop_ev.is_set():
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue

            # Timestamps
            ts_epoch = float(msg.timestamp) if hasattr(msg, "timestamp") else time.time()
            ts_struct = time.localtime(ts_epoch)
            ts_iso = time.strftime("%Y-%m-%d %H:%M:%S", ts_struct) + f".{int((ts_epoch%1)*1_000_000):06d}"

            arb_hex = f"0x{msg.arbitration_id:08X}" if msg.is_extended_id else f"0x{msg.arbitration_id:03X}"
            dlc = msg.dlc if hasattr(msg, "dlc") else len(msg.data)
            data_hex = msg.data.hex()

            # Print to console
            print(f"{ts_iso}  can1  {arb_hex}  dlc={dlc}  ext={int(msg.is_extended_id)}  data=[{data_hex}]")

            # Write CSV
            writer.writerow([ts_iso, f"{ts_epoch:.6f}", "can1", arb_hex, int(msg.is_extended_id), dlc, data_hex])
            f.flush()
    finally:
        f.close()


# -------------------- Minimal MAIN ----------------------
# Uses the same style as your existing code (thread + stop event)
BITRATE = 500000
BUSTYPE = "socketcan"

def make_can1_bus():
    return can.interface.Bus(bustype=BUSTYPE, channel="can1", bitrate=BITRATE)

def main():
    bus_can1 = make_can1_bus()
    stop_ev = threading.Event()

    rec_thr = threading.Thread(target=record_can1_task, args=(bus_can1, stop_ev, "can1_log.csv"), daemon=True)
    rec_thr.start()
    print("Recording raw CAN from can1 → can1_log.csv. Press Ctrl+C to stop.")

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
