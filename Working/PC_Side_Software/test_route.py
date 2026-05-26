import requests

def format_distance_km(length_km: float) -> str:
    if length_km < 1.0:
        return f"{round(length_km * 1000):d} m"
    return f"{length_km:.1f} km"

def maneuver_type_to_direction(t: int) -> str:
    mapping = {
        8:  "continue",
        9:  "bear right",
        10: "right",
        11: "sharp right",
        14: "sharp left",
        15: "left",
        16: "bear left",
        20: "exit right",
        21: "exit left",
        23: "keep right",
        24: "keep left",
        26: "enter roundabout",
        27: "exit roundabout",
        6:  "arrive",
    }
    return mapping.get(t, f"unknown({t})")

def extract_street_like(m: dict) -> str:
    streets = m.get("street_names")
    if isinstance(streets, list) and streets and streets[0]:
        return streets[0]

    sign = m.get("sign", {})
    if isinstance(sign, dict):
        # Exit number (treat as valid "street")
        exit_nums = sign.get("exit_number_elements")
        if isinstance(exit_nums, list) and exit_nums:
            txt = exit_nums[0].get("text")
            if txt:
                return txt if "exit" in txt.lower() else f"Exit {txt}"

        # "Toward" text (common for ramps/exits)
        toward = sign.get("exit_toward_elements")
        if isinstance(toward, list) and toward:
            txt = toward[0].get("text")
            if txt:
                return txt

    instr = m.get("instruction")
    return instr if instr else "(unknown)"

def find_first_actionable_index(maneuvers: list[dict]) -> int:
    # Skip “start” types if present (varies by route; common values 1/2/3)
    for i, m in enumerate(maneuvers):
        t = m.get("type", -1)
        if t not in (1, 2, 3):
            return i
    return 0
    
def main():
    req = {
        "locations": [
            {"lat": 49.261286, "lon": -123.150797, "type": "break"},
            {"lat": 49.268978, "lon": -123.162058, "type": "break"}
        ],
        "costing": "auto",
        "units": "kilometers"
    }

    
    r = requests.post("http://localhost:8002/route", json=req, timeout=10)
    r.raise_for_status()
    data = r.json()

    maneuvers = data["trip"]["legs"][0]["maneuvers"]
    start_i = find_first_actionable_index(maneuvers)

    # Take next 3 maneuvers starting from start_i
    n = 20
    nextn = maneuvers[start_i:start_i + n]

    for k, m in enumerate(nextn, start=1):
        direction = maneuver_type_to_direction(m.get("type", -1))
        distance_str = format_distance_km(float(m.get("length", 0.0)))
        street = extract_street_like(m)

        print(f"--- Next {k} ---")
        print(f"Direction: {direction}")
        print(f"Distance:  {distance_str}")
        print(f"Street:    {street}")

if __name__ == "__main__":
    main()
