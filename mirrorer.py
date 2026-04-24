import json
import sys
from pathlib import Path

# =========================
# CONFIG
# =========================
FIELD_WIDTH = 8.0692  # meters (Choreo uses 8.0692)

CHOREO_DIR = Path("./src/main/deploy/choreo")

# =========================
# MIRROR HELPERS
# =========================
def mirror_y(y):
    return FIELD_WIDTH - y

def mirror_heading(h):
    return -h

def mirror_exp(exp, val, axis):
    """Update expression string and value for a mirrored field."""
    new_val = mirror_y(val) if axis == "y" else mirror_heading(val)
    # Always use the literal mirrored value as the expression,
    # even if the original was a variable reference (e.g. "d1e.y"),
    # so Choreo doesn't re-resolve the original un-mirrored variable.
    if axis == "y":
        new_exp = f"{new_val} m"
    else:
        new_exp = f"{new_val} rad"
    return new_exp, new_val


# =========================
# MAIN TRAJ MIRROR
# =========================
def mirror_choreo_traj(data):
    mirrored = json.loads(json.dumps(data))  # deep copy

    # --- snapshot.waypoints ---
    if "snapshot" in mirrored:
        snap = mirrored["snapshot"]
        if "waypoints" in snap:
            for wp in snap["waypoints"]:
                wp["y"] = mirror_y(wp["y"])
                wp["heading"] = mirror_heading(wp["heading"])

        # --- snapshot.constraints (e.g. KeepInRectangle y) ---
        if "constraints" in snap:
            for c in snap["constraints"]:
                props = c.get("data", {}).get("props", {})
                if "y" in props and isinstance(props["y"], (int, float)):
                    # For KeepInRectangle: mirror the y origin
                    if "h" in props:
                        props["y"] = mirror_y(props["y"] + props["h"])

    # --- params.waypoints ---
    if "params" in mirrored:
        params = mirrored["params"]
        if "waypoints" in params:
            for wp in params["waypoints"]:
                if "y" in wp and isinstance(wp["y"], dict):
                    new_exp, new_val = mirror_exp(wp["y"]["exp"], wp["y"]["val"], "y")
                    wp["y"]["exp"] = new_exp
                    wp["y"]["val"] = new_val
                if "heading" in wp and isinstance(wp["heading"], dict):
                    new_exp, new_val = mirror_exp(wp["heading"]["exp"], wp["heading"]["val"], "heading")
                    wp["heading"]["exp"] = new_exp
                    wp["heading"]["val"] = new_val

        # --- params.constraints ---
        if "constraints" in params:
            for c in params["constraints"]:
                props = c.get("data", {}).get("props", {})
                if "y" in props and isinstance(props["y"], dict):
                    if "h" in props:
                        # KeepInRectangle: mirror y origin
                        h_val = props["h"]["val"] if isinstance(props["h"], dict) else props["h"]
                        old_y = props["y"]["val"]
                        new_y = mirror_y(old_y + h_val)
                        new_exp, _ = mirror_exp(props["y"]["exp"], old_y, "y")
                        props["y"]["val"] = new_y
                        props["y"]["exp"] = f"{new_y} m"

    # --- trajectory.samples ---
    if "trajectory" in mirrored and "samples" in mirrored["trajectory"]:
        for s in mirrored["trajectory"]["samples"]:
            s["y"] = mirror_y(s["y"])
            s["heading"] = mirror_heading(s["heading"])
            s["vy"] = -s["vy"]
            s["omega"] = -s["omega"]
            s["ay"] = -s["ay"]
            s["alpha"] = -s["alpha"]
            # fy: negate each force-y component
            if "fy" in s:
                s["fy"] = [-f for f in s["fy"]]

    return mirrored


# =========================
# FILE IO
# =========================
def main():
    if len(sys.argv) < 2:
        # Default: mirror all udl*.traj -> udr*.traj
        inputs = sorted(CHOREO_DIR.glob("rtml*.traj"))
        if not inputs:
            print("No udl*.traj files found in", CHOREO_DIR)
            return
        pairs = [(p, p.parent / p.name.replace("rtml", "rtmr")) for p in inputs]
    else:
        input_path = Path(sys.argv[1])
        if len(sys.argv) >= 3:
            output_path = Path(sys.argv[2])
        else:
            output_path = input_path.parent / input_path.name.replace("udl", "udr")
        pairs = [(input_path, output_path)]

    for input_path, output_path in pairs:
        with input_path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        mirrored = mirror_choreo_traj(data)
        # Update the trajectory name
        if "name" in mirrored:
            mirrored["name"] = mirrored["name"].replace("udl", "udr")

        with output_path.open("w", encoding="utf-8") as f:
            json.dump(mirrored, f, indent=1)
            f.write("\n")

        print(f"Mirrored: {input_path.name} -> {output_path.name}")

if __name__ == "__main__":
    main()
