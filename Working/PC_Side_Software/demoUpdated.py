# gui_send_route.py
import tkinter as tk
from tkinter import ttk
import requests

import time
import threading
import queue

from rn4871_ble import RN4871BLE

RN_ADDR = "40:84:32:56:57:AB"  # your device
VALHALLA_URL = "http://localhost:8002/route"

# ----------------- Valhalla helpers -----------------

def encode_distance_bytes(length_km: float) -> tuple[int, int, int]:
    if length_km < 1.0:
        meters = int(round(length_km * 1000.0))
        meters = max(0, min(9999, meters))
        b3 = meters // 100
        b4 = meters % 100
        unit = 0  # m
        return b3, b4, unit

    km = max(0.0, float(length_km))
    km_int = int(km)
    if km_int > 254:
        km_int = 254

    frac = km - int(km)
    # one decimal digit (tenths of km): 0..9
    dec1 = int(round(frac * 10.0))
    if dec1 >= 10:
        km_int = min(254, km_int + 1)
        dec1 = 0
    dec1 = max(0, min(9, dec1))

    unit = 1  # km
    return km_int, dec1, unit

def maneuver_type_to_arrow_byte(t: int) -> int:
    left_types = {14, 15, 16, 21, 24}
    right_types = {9, 10, 11, 20, 23}
    if t in left_types:
        return 1
    if t in right_types:
        return 2
    return 2

def is_destination_maneuver(m: dict) -> bool:
    """
    Valhalla maneuver types:
      4 = arrive at destination
      5 = arrive at destination on left
      6 = arrive at destination on right
    """
    try:
        return int(m.get("type", -1)) in (4, 5, 6)
    except Exception:
        return False

def extract_street_like(m: dict) -> str:
    streets = m.get("street_names")
    if isinstance(streets, list) and streets and streets[0]:
        return streets[0]

    sign = m.get("sign", {})
    if isinstance(sign, dict):
        exit_nums = sign.get("exit_number_elements")
        if isinstance(exit_nums, list) and exit_nums:
            txt = exit_nums[0].get("text")
            if txt:
                return txt if "exit" in txt.lower() else f"Exit {txt}"

        toward = sign.get("exit_toward_elements")
        if isinstance(toward, list) and toward:
            txt = toward[0].get("text")
            if txt:
                return txt

    instr = m.get("instruction")
    return instr if instr else "(unknown)"

def find_first_actionable_index(maneuvers: list[dict]) -> int:
    for i, m in enumerate(maneuvers):
        t = m.get("type", -1)
        if t not in (1, 2, 3):
            return i
    return 0

def street8(s: str) -> bytes:
    s = (s or "")[:8].ljust(8, " ")
    out = []
    for ch in s:
        c = ord(ch)
        if c < 0x20 or c > 0x7E:
            c = 0x20
        out.append(c)
    return bytes(out)

def build_packet_14(direction_byte: int, length_km: float, street: str, special: int = 0) -> bytes:
    b3, b4, unit = encode_distance_bytes(length_km)
    pkt = bytearray(14)
    pkt[0] = 255
    pkt[1] = direction_byte & 0xFF
    pkt[2] = b3 & 0xFF
    pkt[3] = b4 & 0xFF
    pkt[4] = unit & 0xFF
    pkt[5:13] = street8(street)  # BLE payload still uses 8 chars (unchanged)
    pkt[13] = special & 0xFF
    return bytes(pkt)

def pretty_distance(length_km: float) -> str:
    if length_km < 1.0:
        return f"{int(round(length_km * 1000.0))} m"
    return f"{length_km:.1f} km"

def sanitize_printable_full(s: str) -> str:
    s = s or ""
    out = []
    for ch in s:
        c = ord(ch)
        out.append(ch if (0x20 <= c <= 0x7E) else " ")
    return "".join(out).strip()

# ----------------- Tk app -----------------

def main():
    root = tk.Tk()
    root.title("RN4871 Valhalla Sender (Dev)")

    # ============ DEV WINDOW ============
    log = tk.Text(root, width=90, height=20)
    log.grid(row=0, column=0, columnspan=4, padx=8, pady=8)

    def add_log(msg: str):
        log.insert("end", msg + "\n")
        log.see("end")

    def on_rx(data: bytes):
        root.after(0, lambda: add_log(f"RX: {data.hex(' ')}"))

    def on_status(msg: str):
        root.after(0, lambda: add_log(msg))

    ble = RN4871BLE(RN_ADDR, on_rx=on_rx, on_status=on_status)
    ble.start()

    # ---- connection state ----
    ble_connected_state = False

    def _bool_from_attr(obj, attr_name: str):
        if obj is None:
            return None
        if not hasattr(obj, attr_name):
            return None
        v = getattr(obj, attr_name)
        try:
            if callable(v):
                v = v()
        except Exception:
            return None
        return v if isinstance(v, bool) else None

    def _find_client_obj():
        for name in ("client", "_client", "bleak_client", "_bleak_client", "ble_client", "_ble_client"):
            if hasattr(ble, name):
                c = getattr(ble, name)
                if c is not None:
                    return c
        return None

    def ble_connected() -> bool:
        v = _bool_from_attr(ble, "is_connected")
        if isinstance(v, bool):
            return v
        client = _find_client_obj()
        v2 = _bool_from_attr(client, "is_connected")
        if isinstance(v2, bool):
            return v2
        v3 = _bool_from_attr(ble, "connected")
        if isinstance(v3, bool):
            return v3
        v4 = _bool_from_attr(client, "connected")
        if isinstance(v4, bool):
            return v4
        return ble_connected_state

    # ============================================================
    # BLE TX WORKER: serialize all writes off the Tk thread
    # ============================================================
    tx_q: "queue.Queue[tuple[bytes, str]]" = queue.Queue()
    tx_stop = threading.Event()

    TX_GAP_MS = 80
    KEEPALIVE_BYTES = b"KA\n"
    KEEPALIVE_MS = 5000

    def tx_worker():
        last_send_t = 0.0
        while not tx_stop.is_set():
            try:
                pkt, tag = tx_q.get(timeout=0.2)
            except queue.Empty:
                continue

            if pkt is None:
                tx_q.task_done()
                continue

            # throttle
            now = time.time()
            dt = now - last_send_t
            gap_s = TX_GAP_MS / 1000.0
            if dt < gap_s:
                time.sleep(gap_s - dt)

            t0 = time.time()
            ok = False
            try:
                if ble_connected():
                    ok = bool(ble.send_packet(pkt))
                else:
                    ok = False
            except Exception as e:
                ok = False
                root.after(0, lambda e=e: add_log(f"TX ERROR ({tag}): {e}"))

            t_ms = (time.time() - t0) * 1000.0
            last_send_t = time.time()

            root.after(0, lambda ok=ok, tag=tag, t_ms=t_ms:
                       add_log(f"TX DONE ({tag}) -> {ok}  ({t_ms:.1f} ms)"))

            tx_q.task_done()

    tx_thread = threading.Thread(target=tx_worker, daemon=True)
    tx_thread.start()

    def enqueue_send(pkt: bytes, tag: str):
        try:
            tx_q.put_nowait((pkt, tag))
            add_log(f"ENQ ({tag}): {pkt.hex(' ')}")
        except Exception as e:
            add_log(f"ENQ FAILED ({tag}): {e}")

    # ---- KEEP-ALIVE (via queue) ----
    keepalive_job = None

    def keep_alive_tick():
        nonlocal keepalive_job
        try:
            if ble_connected():
                enqueue_send(KEEPALIVE_BYTES, "KEEPALIVE")
        except Exception:
            pass
        keepalive_job = root.after(KEEPALIVE_MS, keep_alive_tick)

    keep_alive_tick()

    # ---- route state ----
    maneuvers: list[dict] = []
    cur_idx: int = 0  # next-to-send index into maneuvers

    # NEW: currently displayed instruction (what's on the screen)
    current_display_text: str = "Welcome Screen"

    # ---- UI fields for route (shared vars) ----
    frm = ttk.Frame(root)
    frm.grid(row=1, column=0, columnspan=4, padx=8, pady=4, sticky="ew")
    frm.columnconfigure(1, weight=1)
    frm.columnconfigure(3, weight=1)

    ttk.Label(frm, text="From (lat, lon):").grid(row=0, column=0, sticky="w")
    from_var = tk.StringVar(value="49.277684, -123.108586")
    ttk.Entry(frm, textvariable=from_var, width=28).grid(row=0, column=1, sticky="ew", padx=6)

    ttk.Label(frm, text="To (lat, lon):").grid(row=0, column=2, sticky="w")
    to_var = tk.StringVar(value="49.272365, -123.155302")
    ttk.Entry(frm, textvariable=to_var, width=28).grid(row=0, column=3, sticky="ew", padx=6)

    # ---- status line (dev only) ----
    status_var = tk.StringVar(value="No route loaded.")
    ttk.Label(root, textvariable=status_var).grid(row=2, column=0, columnspan=4, padx=8, pady=2, sticky="w")

    # ============ DEMO WINDOW ============
    DEMO_N = 4  # total rectangles including "current display" (slot 0)

    demo = tk.Toplevel(root)
    demo.title("BLE Route Sender")
    demo.geometry("560x700")

    HIGHLIGHT_BG = "#2F6FE0"
    NORMAL_BG = "#D6D6D6"
    EMPTY_BG = "#FFFFFF"
    CARD_HEIGHT_PX = 110

    demo_header = ttk.Label(demo, text="Navigation Queue", font=("Segoe UI", 18, "bold"), anchor="center")
    demo_header.pack(fill="x", padx=12, pady=(14, 10))

    cards_frame = ttk.Frame(demo)
    cards_frame.pack(fill="both", expand=True, padx=12, pady=6)

    card_frames: list[tk.Frame] = []
    card_labels: list[tk.Label] = []

    def _make_card(parent) -> tuple[tk.Frame, tk.Label]:
        f = tk.Frame(parent, bd=2, relief="ridge", bg=EMPTY_BG, height=CARD_HEIGHT_PX)
        f.pack_propagate(False)
        lbl = tk.Label(
            f,
            text="",
            anchor="center",
            justify="center",
            font=("Segoe UI", 14),
            bg=EMPTY_BG,
            fg="black",
            wraplength=520,
        )
        lbl.pack(fill="both", expand=True)
        return f, lbl

    for _ in range(DEMO_N):
        f, lbl = _make_card(cards_frame)
        f.pack(fill="x", pady=7)
        card_frames.append(f)
        card_labels.append(lbl)

    # Connected indicator row
    conn_row = ttk.Frame(demo)
    conn_row.pack(fill="x", padx=12, pady=(6, 4))

    conn_label = ttk.Label(conn_row, text="BLE Status:", font=("Segoe UI", 12))
    conn_label.grid(row=0, column=0, sticky="w")

    IND_SIZE = 18
    conn_canvas = tk.Canvas(conn_row, width=IND_SIZE + 2, height=IND_SIZE + 2, highlightthickness=0)
    conn_canvas.grid(row=0, column=1, padx=(10, 0), sticky="w")
    circle_id = conn_canvas.create_oval(2, 2, IND_SIZE, IND_SIZE, outline="#333333", width=1, fill="red")

    def set_connected_indicator(is_on: bool):
        conn_canvas.itemconfigure(circle_id, fill=("green" if is_on else "red"))

    def refresh_connection_indicator_now():
        try:
            set_connected_indicator(ble_connected())
        except Exception:
            set_connected_indicator(False)

    def poll_connection_indicator():
        refresh_connection_indicator_now()
        demo.after(250, poll_connection_indicator)

    poll_connection_indicator()

    # Controls
    demo_controls = ttk.Frame(demo)
    demo_controls.pack(fill="x", padx=12, pady=(6, 10))
    demo_controls.columnconfigure(1, weight=1)
    demo_controls.columnconfigure(3, weight=1)

    ttk.Label(demo_controls, text="From (lat, lon):").grid(row=0, column=0, sticky="w")
    ttk.Entry(demo_controls, textvariable=from_var, width=28).grid(row=0, column=1, sticky="ew", padx=6)

    ttk.Label(demo_controls, text="To (lat, lon):").grid(row=0, column=2, sticky="w")
    ttk.Entry(demo_controls, textvariable=to_var, width=28).grid(row=0, column=3, sticky="ew", padx=6)

    btn_row = ttk.Frame(demo)
    btn_row.pack(fill="x", padx=12, pady=(0, 14))
    for c in range(4):
        btn_row.columnconfigure(c, weight=1)

    # -------- text helpers --------
    def maneuver_to_text(m: dict) -> str:
        
        if is_destination_maneuver(m):
            return "You have arrived!"
        
        t = int(m.get("type", -1))
        length_km = float(m.get("length", 0.0))
        street = sanitize_printable_full(extract_street_like(m))
        dir_byte = maneuver_type_to_arrow_byte(t)
        dir_txt = "LEFT" if dir_byte == 1 else "RIGHT"
        dist = pretty_distance(length_km)
        return f"{dir_txt}\n{dist}\n{street}"

    def refresh_demo_cards():
        """
        Slot 0 (blue): CURRENT DISPLAY (what's on the screen)
        Slots 1..: UPCOMING queue starting at cur_idx
        Fetching a route should NOT overwrite slot 0.
        """
        # Slot 0: current display
        card_frames[0].configure(bg=HIGHLIGHT_BG)
        card_labels[0].configure(bg=HIGHLIGHT_BG, fg="white", text=current_display_text)

        # Slots 1..: upcoming maneuvers
        for i in range(1, DEMO_N):
            idx = cur_idx + (i - 1)  # queue starts at cur_idx
            if maneuvers and 0 <= idx < len(maneuvers):
                text = maneuver_to_text(maneuvers[idx])
                bg, fg = NORMAL_BG, "black"
            else:
                text = ""
                bg, fg = EMPTY_BG, "black"

            card_frames[i].configure(bg=bg)
            card_labels[i].configure(bg=bg, fg=fg, text=text)

    refresh_demo_cards()

    # ---- shared actions ----
    def do_connect():
        nonlocal ble_connected_state
        t0 = time.time()
        ok = ble.connect(timeout_s=10)
        ble_connected_state = bool(ok)
        dt = (time.time() - t0) * 1000.0
        add_log(f"connect() -> {ok}  ({dt:.1f} ms)")
        refresh_connection_indicator_now()
        refresh_demo_cards()

    def do_disconnect():
        nonlocal ble_connected_state
        try:
            ble.disconnect()
        finally:
            ble_connected_state = False
        add_log("disconnect() called")
        refresh_connection_indicator_now()
        refresh_demo_cards()

    def do_fetch_route():
        nonlocal maneuvers, cur_idx
        try:
            lat1, lon1 = [float(x.strip()) for x in from_var.get().split(",")]
            lat2, lon2 = [float(x.strip()) for x in to_var.get().split(",")]
        except Exception:
            add_log("Parse error: enter coords like '49.261286, -123.150797'")
            return

        req = {
            "locations": [
                {"lat": lat1, "lon": lon1, "type": "break"},
                {"lat": lat2, "lon": lon2, "type": "break"},
            ],
            "costing": "auto",
            "units": "kilometers",
        }

        try:
            r = requests.post(VALHALLA_URL, json=req, timeout=10)
            r.raise_for_status()
            data = r.json()
            maneuvers = data["trip"]["legs"][0]["maneuvers"]
            cur_idx = find_first_actionable_index(maneuvers)
            status_var.set(f"Route loaded: {len(maneuvers)} maneuvers. Starting at index {cur_idx}.")
            add_log("Route fetched OK.")
        except Exception as e:
            add_log(f"Route fetch failed: {e}")
            maneuvers = []
            cur_idx = 0
            status_var.set("No route loaded.")

        # IMPORTANT: do NOT overwrite current_display_text here
        refresh_demo_cards()

    def do_send_next():
        nonlocal cur_idx, current_display_text

        if not maneuvers:
            add_log("No maneuvers loaded. Click Fetch Route first.")
            refresh_demo_cards()
            return

        if cur_idx >= len(maneuvers):
            add_log("Reached end of maneuver list.")
            refresh_demo_cards()
            return

        m = maneuvers[cur_idx]
        street = extract_street_like(m)

        dir_byte = maneuver_type_to_arrow_byte(int(m.get("type", -1)))

        # --- SPECIAL BYTE: destination reached maneuver only ---
        special = 1 if is_destination_maneuver(m) else 0

        pkt = build_packet_14(
            dir_byte,
            float(m.get("length", 0.0)),
            street,
            special=special
        )

        # Update "current display" immediately in UI (blue box)
        current_display_text = maneuver_to_text(m)

        add_log(f"[{cur_idx}] ENQ CMD (type={m.get('type')} special={special})")
        enqueue_send(pkt, f"CMD idx={cur_idx} special={special}")

        cur_idx += 1
        status_var.set(f"Next index: {cur_idx} / {len(maneuvers)}")
        refresh_demo_cards()

    # ---- DEV window buttons ----
    btn_connect = ttk.Button(root, text="Connect BLE", command=do_connect)
    btn_fetch   = ttk.Button(root, text="Fetch Route", command=do_fetch_route)
    btn_next    = ttk.Button(root, text="Send Next Command", command=do_send_next)
    btn_disc    = ttk.Button(root, text="Disconnect", command=do_disconnect)

    btn_connect.grid(row=3, column=0, padx=8, pady=6, sticky="ew")
    btn_fetch.grid(row=3, column=1, padx=8, pady=6, sticky="ew")
    btn_next.grid(row=3, column=2, padx=8, pady=6, sticky="ew")
    btn_disc.grid(row=3, column=3, padx=8, pady=6, sticky="ew")

    # ---- DEMO window buttons ----
    demo_btn_connect = ttk.Button(btn_row, text="Connect BLE", command=do_connect)
    demo_btn_fetch   = ttk.Button(btn_row, text="Fetch Route", command=do_fetch_route)
    demo_btn_next    = ttk.Button(btn_row, text="Send Next Command", command=do_send_next)
    demo_btn_disc    = ttk.Button(btn_row, text="Disconnect", command=do_disconnect)

    demo_btn_connect.grid(row=0, column=0, padx=6, pady=8, sticky="ew")
    demo_btn_fetch.grid(row=0, column=1, padx=6, pady=8, sticky="ew")
    demo_btn_next.grid(row=0, column=2, padx=6, pady=8, sticky="ew")
    demo_btn_disc.grid(row=0, column=3, padx=6, pady=8, sticky="ew")

    # Window close behavior
    def on_demo_close():
        try:
            demo.destroy()
        except Exception:
            pass

    demo.protocol("WM_DELETE_WINDOW", on_demo_close)

    def on_close():
        nonlocal keepalive_job
        try:
            if keepalive_job is not None:
                root.after_cancel(keepalive_job)
        except Exception:
            pass

        tx_stop.set()
        try:
            tx_q.put_nowait((b"", "STOP"))
        except Exception:
            pass

        try:
            ble.stop()
        except Exception:
            pass

        try:
            demo.destroy()
        except Exception:
            pass

        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
