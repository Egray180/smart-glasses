import asyncio
import threading
from dataclasses import dataclass
from typing import Callable, Optional

from bleak import BleakClient


@dataclass
class RN4871UUIDs:
    # You already discovered these on your machine.
    tx_char: str = "49535343-8841-43f4-a8d4-ecbe34729bb3"  # PC -> RN4871 write
    notify_char: str = "49535343-1e4d-4bd9-ba61-23c647249616"  # RN4871 -> PC notify (working for you)
    # Optional: if you want to also enable notify on the other char, you can add it.
    notify_char_2: Optional[str] = "49535343-4c8a-39b3-2f49-511cff073b7e"


class RN4871BLE:
    """
    A small BLE driver:
      - Runs Bleak/asyncio on a dedicated background thread
      - Exposes thread-safe methods you can call from normal (non-async) code
      - Lets you register a callback for received bytes
    """

    def __init__(
        self,
        address: str,
        uuids: RN4871UUIDs = RN4871UUIDs(),
        on_rx: Optional[Callable[[bytes], None]] = None,
        on_status: Optional[Callable[[str], None]] = None,
    ):
        self.address = address
        self.uuids = uuids
        self.on_rx = on_rx
        self.on_status = on_status

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._client: Optional[BleakClient] = None

        self._connected_evt = threading.Event()
        self._stop_evt = threading.Event()

    # ---------- public API (call from normal code) ----------

    def start(self) -> None:
        """Start BLE background thread (does not connect yet)."""
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Disconnect and stop background loop."""
        self._stop_evt.set()
        if self._loop:
            asyncio.run_coroutine_threadsafe(self._async_disconnect(), self._loop)
            self._loop.call_soon_threadsafe(self._loop.stop)

    def connect(self, timeout_s: float = 10.0) -> bool:
        """Connect (blocking). Returns True on success."""
        self._connected_evt.clear()
        if not self._loop:
            raise RuntimeError("Call start() before connect().")

        fut = asyncio.run_coroutine_threadsafe(self._async_connect(), self._loop)
        try:
            ok = fut.result(timeout=timeout_s)
        except Exception as e:
            self._status(f"Connect error: {e}")
            return False

        if ok:
            self._connected_evt.set()
        return ok

    def disconnect(self, timeout_s: float = 5.0) -> None:
        """Disconnect (blocking)."""
        if not self._loop:
            return
        fut = asyncio.run_coroutine_threadsafe(self._async_disconnect(), self._loop)
        try:
            fut.result(timeout=timeout_s)
        except Exception as e:
            self._status(f"Disconnect error: {e}")
        self._connected_evt.clear()

    def is_connected(self) -> bool:
        return self._client is not None and bool(getattr(self._client, "is_connected", False))

    def send_packet(self, payload: bytes, timeout_s: float = 3.0) -> bool:
        """Thread-safe send. Returns True if write call completed."""
        if not self._loop:
            self._status("send_packet(): BLE loop not running (did you call start()?)")
            return False
        if not self.is_connected():
            self._status("send_packet(): not connected")
            return False

        fut = asyncio.run_coroutine_threadsafe(self._async_write(payload), self._loop)
        try:
            fut.result(timeout=timeout_s)
            return True
        except Exception as e:
            self._status(f"send_packet(): write failed: {e}")
            return False

    # ---------- internal helpers ----------

    def _status(self, msg: str) -> None:
        if self.on_status:
            self.on_status(msg)
        else:
            print(msg)

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.create_task(self._async_keepalive())
        self._loop.run_forever()

    async def _async_keepalive(self) -> None:
        # Keep the loop alive until stop() is called.
        while not self._stop_evt.is_set():
            await asyncio.sleep(0.2)

    async def _async_connect(self) -> bool:
        # If already connected, do nothing
        if self._client and self._client.is_connected:
            return True

        self._client = BleakClient(self.address)

        try:
            await self._client.connect()
        except Exception as e:
            self._status(f"BLE connect failed: {e}")
            self._client = None
            return False

        self._status(f"Connected: {self._client.is_connected}")

        # Notifications
        try:
            await self._client.start_notify(self.uuids.notify_char, self._on_notify)
            self._status(f"Notify enabled on {self.uuids.notify_char}")
        except Exception as e:
            self._status(f"Notify enable failed on {self.uuids.notify_char}: {e}")

        if self.uuids.notify_char_2:
            try:
                await self._client.start_notify(self.uuids.notify_char_2, self._on_notify)
                self._status(f"Notify enabled on {self.uuids.notify_char_2}")
            except Exception as e:
                self._status(f"Notify enable failed on {self.uuids.notify_char_2}: {e}")

        return True

    async def _async_disconnect(self) -> None:
        if not self._client:
            return
        try:
            if self._client.is_connected:
                # stop notify best-effort
                try:
                    await self._client.stop_notify(self.uuids.notify_char)
                except Exception:
                    pass
                if self.uuids.notify_char_2:
                    try:
                        await self._client.stop_notify(self.uuids.notify_char_2)
                    except Exception:
                        pass
                await self._client.disconnect()
        finally:
            self._status("Disconnected")
            self._client = None

    async def _async_write(self, payload: bytes) -> None:
        # response=False is usually correct for "write-without-response"
        await self._client.write_gatt_char(self.uuids.tx_char, payload, response=False)

    def _on_notify(self, sender, data: bytearray) -> None:
        # sender is characteristic handle/uuid depending on platform; data is bytes
        b = bytes(data)
        if self.on_rx:
            self.on_rx(b)
        else:
            print("RX:", b.hex(" "))

