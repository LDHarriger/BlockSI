"""Temporary script to wrap calibration sections in ui.expansion() elements."""
import os

UI_PATH = os.path.join("Interfaces", "PC", "dashboard", "ui_main.py")

with open(UI_PATH, "r", encoding="utf-8") as f:
    lines = f.readlines()

print(f"Total lines before: {len(lines)}")

# === SECTION 1: Power-O3 ===
# The expansion header (lines 557-564) is already in place.
# Content at lines 566-844 (0-idx 565-843) needs +4 spaces.
for i in range(565, 844):
    if lines[i].strip():  # don't indent blank lines
        lines[i] = "    " + lines[i]

# === SECTION 2: k_d ===
# Current structure (0-indexed):
#   844: comment line
#   845: ui.separator()
#   846: ui.label("k_d Calibration")
#   847-854: ui.markdown(...)
#   855: blank
#   856-1000: content
# Replace lines 845-854 with expansion header + indented markdown,
# then indent 856-1000 by +4 spaces.

# Replace separator with expansion opener
lines[845] = '                with ui.expansion(\n'
lines[846] = '                    "k_d Calibration", icon="science",\n'
lines[847] = '                ).classes("w-full"):\n'
lines[848] = '                    ui.markdown(\n'
lines[849] = '                        "Fill the vessel at 100% power until steady-state, "\n'
lines[850] = '                        "then evacuate at 0% power until O3 clears. "\n'
lines[851] = '                        "Fits a decay-aware CSTR model to extract system volume, "\n'
lines[852] = '                        "O3 decay rate, and dead volume. Air compressor is OFF "\n'
lines[853] = '                        "during calibration for best decay sensitivity. "\n'
lines[854] = '                        "Parameters generalise to any flow rate and air config."\n'
# Line 855 was ).classes(...), keep it but indent
# Actually let me check what 855 is:
# Original line 855 = ).classes("text-caption text-grey q-mb-sm")
# It needs to close the markdown and be at 20+4=24 spaces
lines[855] = '                    ).classes("text-caption text-grey q-mb-sm")\n'

# Now indent content lines 857-1000 (0-idx 856-1000) by +4 spaces
for i in range(856, 1001):
    if lines[i].strip():
        lines[i] = "    " + lines[i]

# === SECTION 3: k_abs ===
# Current structure (0-indexed):
#   1001: comment line
#   1002: ui.separator()
#   1003-1005: ui.label(...) multi-line
#   1006-1011: ui.markdown(...)
#   1012: blank
#   1013+: content until line 1117

# Replace header
lines[1002] = '                with ui.expansion(\n'
lines[1003] = '                    "k_abs Calibration (Loaded Vessel)", icon="biotech",\n'
lines[1004] = '                ).classes("w-full"):\n'
lines[1005] = '                    ui.markdown(\n'
lines[1006] = '                        "Fill a substrate-loaded vessel at 100% power for 30 min, "\n'
lines[1007] = '                        "then evacuate. Fits k_abs (substrate absorption rate) and "\n'
lines[1008] = '                        "V_residual (free gas volume) from the transient response. "\n'
lines[1009] = '                        "Requires a valid k_d model and power-O3 calibration."\n'
lines[1010] = '                    ).classes("text-caption text-grey q-mb-sm")\n'
lines[1011] = '\n'

# Indent content lines 1013-1117 (0-idx 1012-1116) by +4 spaces
for i in range(1012, 1117):
    if lines[i].strip():
        lines[i] = "    " + lines[i]

with open(UI_PATH, "w", encoding="utf-8") as f:
    f.writelines(lines)

print(f"Total lines after: {len(lines)}")
print("Done! All three calibration sections wrapped in ui.expansion().")

