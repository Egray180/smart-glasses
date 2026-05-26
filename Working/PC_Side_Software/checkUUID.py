import asyncio
from bleak import BleakScanner, BleakClient

UART_SERVICE = "49535343-FE7D-4AE5-8FA9-9FAFD205E455"

async def main():
    print("Scanning...")

    devices = await BleakScanner.discover(timeout=5.0)

    for d in devices:
        # RSSI isn't always available depending on backend/version
        rssi = getattr(d, "rssi", None)
        if rssi is None:
            # Some backends put it in metadata
            md = getattr(d, "metadata", {}) or {}
            rssi = md.get("rssi", "N/A")

        print(f"{d.address}  {d.name}  RSSI={rssi}")

    # Pick your device by name substring, or hardcode address
    target = None
    for d in devices:
        if d.name and ("RN" in d.name or "RN4871" in d.name or "RN4870" in d.name):
            target = d
            break

    if not target:
        print("\nDidn't auto-find it.")
        print("Copy the address from the scan list and set TARGET_ADDRESS manually.")
        return

    print(f"\nConnecting to {target.address} ({target.name})...")
    async with BleakClient(target.address) as client:
        print("Connected:", client.is_connected)

        svcs = client.services
        for s in svcs:
            print(f"\nService {s.uuid}: {s.description}")
            for c in s.characteristics:
                print(f"  Char {c.uuid}  props={c.properties}")

        print("\nUART service present:",
              any(s.uuid.lower() == UART_SERVICE.lower() for s in svcs))

asyncio.run(main())
