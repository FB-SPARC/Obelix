import json
from pathlib import Path

# =========================
# CONFIG
# =========================
FIELD_LENGTH = 16.541  # meters
FIELD_WIDTH = 8.069    # meters

INPUT_PATH_FILE = "./src/main/deploy/pathplanner/paths/UDSL3.path"
OUTPUT_PATH_FILE = "UDSR3.path"

# =========================
# MIRROR HELPERS
# =========================
def mirror_y(y):
    return FIELD_WIDTH - y

def mirror_deg(deg):
    return -deg

def mirror_point(pt):
    if pt is None:
        return None
    if isinstance(pt, dict) and "x" in pt and "y" in pt:
        return {
            **pt,
            "y": mirror_y(pt["y"])
        }
    return pt

# =========================
# MAIN PATH MIRROR
# =========================
def mirror_pathplanner_path(data):
    mirrored = json.loads(json.dumps(data))  # deep copy

    # Mirror waypoint geometry
    if "waypoints" in mirrored:
        for wp in mirrored["waypoints"]:
            if "anchor" in wp:
                wp["anchor"] = mirror_point(wp["anchor"])
            if "prevControl" in wp:
                wp["prevControl"] = mirror_point(wp["prevControl"])
            if "nextControl" in wp:
                wp["nextControl"] = mirror_point(wp["nextControl"])

    # Mirror rotation targets
    if "rotationTargets" in mirrored:
        for target in mirrored["rotationTargets"]:
            if "rotationDegrees" in target:
                target["rotationDegrees"] = mirror_deg(target["rotationDegrees"])

    # Mirror end/start state rotations
    if "goalEndState" in mirrored and "rotation" in mirrored["goalEndState"]:
        mirrored["goalEndState"]["rotation"] = mirror_deg(mirrored["goalEndState"]["rotation"])

    if "idealStartingState" in mirrored and "rotation" in mirrored["idealStartingState"]:
        mirrored["idealStartingState"]["rotation"] = mirror_deg(mirrored["idealStartingState"]["rotation"])

    return mirrored

# =========================
# FILE IO
# =========================
def main():
    input_path = Path(INPUT_PATH_FILE)
    output_path = Path(OUTPUT_PATH_FILE)

    with input_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    mirrored = mirror_pathplanner_path(data)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(mirrored, f, indent=2)

    print(f"Mirrored path written to: {output_path}")

if __name__ == "__main__":
    main()