import canopen

# Connect to CANopen
network = canopen.Network()
network.connect(bustype='socketcan', channel='can0', bitrate=125000)

# Add your charger node (Node ID = 10 by default)
charger = canopen.RemoteNode(10, None)
network.add_node(charger)

# Read Installed Algo Count (0x2242h)
raw_count = charger.sdo.upload(0x2242, 0)
algo_count = int.from_bytes(raw_count, byteorder="little")  # 8-bit little endian
print("Installed algo count:", algo_count)

# Loop over each available algorithm
for idx in range(algo_count):
    charger.sdo.download(0x2243, 0, idx.to_bytes(1, "little"))

    raw_info = charger.sdo.upload(0x2244, 0)
    val = int.from_bytes(raw_info, "little")

    algo_id       = val & 0xFFFF          # bits 0–15
    algo_revision = (val >> 16) & 0xFF    # bits 16–23
    algo_variant  = (val >> 24) & 0xFF    # bits 24–31

    print(f"Algorithm ID: {algo_id}")
    print(f"Revision: {algo_revision}")
    print(f"Variant/flags: {algo_variant}")


network.disconnect()