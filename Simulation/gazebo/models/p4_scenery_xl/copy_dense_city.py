#!/usr/bin/env python3
"""Copy the dense p4_scenery model.sdf into p4_scenery_xl, adjusting the model name."""
import os, shutil

src = r"d:\Intern_Data\P4 Transwing Clone Drone\Simulation\gazebo\models\p4_scenery\model.sdf"
dst = r"d:\Intern_Data\P4 Transwing Clone Drone\Simulation\gazebo\models\p4_scenery_xl\model.sdf"
bak = dst + ".bak"

# Backup original
shutil.copy2(dst, bak)
print("Backup saved:", bak)

# Read source (dense city with Landmark 81)
with open(src, "r", encoding="utf-8") as f:
    content = f.read()

# Replace model name to match p4_scenery_xl
content = content.replace('name="p4_scenery"', 'name="p4_scenery_xl"', 1)

# Write to destination
with open(dst, "w", encoding="utf-8") as f:
    f.write(content)

# Validate
import xml.etree.ElementTree as ET
tree = ET.parse(dst)
visuals = tree.findall(".//visual")
print("Done! p4_scenery_xl updated.")
print("Total visuals:", len(visuals))
print("File size:", os.path.getsize(dst) // 1024, "KB")
print("XML valid!")
