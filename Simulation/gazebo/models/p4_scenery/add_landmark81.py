#!/usr/bin/env python3
"""Add Landmark 81 (461 m supertall) to the existing model.sdf."""
import xml.etree.ElementTree as ET
import os

path = r'd:\Intern_Data\P4 Transwing Clone Drone\Simulation\gazebo\models\p4_scenery\model.sdf'

with open(path, 'r', encoding='utf-8') as f:
    content = f.read()

lm81_block = """
      <!-- =====================================================
           LANDMARK 81 - 461 m supertall skyscraper
           Position: (350, 600) - visible from launch pad.
           Three tapered sections + spire + observation deck.
      ===================================================== -->
      <visual name="lm81_base"><pose>350 600 80 0 0 0.05</pose><geometry><box><size>56 56 160</size></box></geometry><material><ambient>0.10 0.16 0.22 1</ambient><diffuse>0.16 0.32 0.48 1</diffuse></material></visual>
      <visual name="lm81_mid"><pose>350 600 240 0 0 0.05</pose><geometry><box><size>44 44 160</size></box></geometry><material><ambient>0.11 0.18 0.24 1</ambient><diffuse>0.18 0.36 0.52 1</diffuse></material></visual>
      <visual name="lm81_crown"><pose>350 600 370 0 0 0.05</pose><geometry><box><size>32 32 100</size></box></geometry><material><ambient>0.12 0.20 0.26 1</ambient><diffuse>0.20 0.40 0.56 1</diffuse></material></visual>
      <visual name="lm81_roof"><pose>350 600 420.6 0 0 0.05</pose><geometry><box><size>34 34 1.2</size></box></geometry><material><ambient>0.06 0.08 0.10 1</ambient><diffuse>0.12 0.18 0.24 1</diffuse></material></visual>
      <visual name="lm81_obs"><pose>350 600 405 0 0 0.05</pose><geometry><box><size>36 36 10</size></box></geometry><material><ambient>0.08 0.14 0.22 1</ambient><diffuse>0.14 0.30 0.50 1</diffuse><specular>0.30 0.40 0.50 1</specular></material></visual>
      <visual name="lm81_spire"><pose>350 600 440.5 0 0 0</pose><geometry><cylinder><radius>1.2</radius><length>41</length></cylinder></geometry><material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.60 0.60 0.62 1</diffuse><specular>0.40 0.40 0.42 1</specular></material></visual>
      <visual name="lm81_tip"><pose>350 600 461 0 0 0</pose><geometry><sphere><radius>1.8</radius></sphere></geometry><material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.60 0.60 0.62 1</diffuse></material></visual>
      <visual name="lm81_b01"><pose>350 600 40 0 0 0.05</pose><geometry><box><size>56.3 56.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b02"><pose>350 600 80 0 0 0.05</pose><geometry><box><size>56.3 56.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b03"><pose>350 600 120 0 0 0.05</pose><geometry><box><size>56.3 56.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b04"><pose>350 600 160 0 0 0.05</pose><geometry><box><size>56.3 56.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b05"><pose>350 600 200 0 0 0.05</pose><geometry><box><size>44.3 44.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b06"><pose>350 600 240 0 0 0.05</pose><geometry><box><size>44.3 44.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b07"><pose>350 600 280 0 0 0.05</pose><geometry><box><size>44.3 44.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b08"><pose>350 600 320 0 0 0.05</pose><geometry><box><size>44.3 44.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b09"><pose>350 600 355 0 0 0.05</pose><geometry><box><size>32.3 32.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_b10"><pose>350 600 390 0 0 0.05</pose><geometry><box><size>32.3 32.3 0.6</size></box></geometry><material><ambient>0.04 0.08 0.14 1</ambient><diffuse>0.06 0.18 0.36 1</diffuse><emissive>0.01 0.04 0.08 1</emissive></material></visual>
      <visual name="lm81_p1"><pose>377 627 80 0 0 0.05</pose><geometry><box><size>1.8 1.8 160</size></box></geometry><material><ambient>0.06 0.10 0.16 1</ambient><diffuse>0.10 0.22 0.38 1</diffuse></material></visual>
      <visual name="lm81_p2"><pose>323 627 80 0 0 0.05</pose><geometry><box><size>1.8 1.8 160</size></box></geometry><material><ambient>0.06 0.10 0.16 1</ambient><diffuse>0.10 0.22 0.38 1</diffuse></material></visual>
      <visual name="lm81_p3"><pose>377 573 80 0 0 0.05</pose><geometry><box><size>1.8 1.8 160</size></box></geometry><material><ambient>0.06 0.10 0.16 1</ambient><diffuse>0.10 0.22 0.38 1</diffuse></material></visual>
      <visual name="lm81_p4"><pose>323 573 80 0 0 0.05</pose><geometry><box><size>1.8 1.8 160</size></box></geometry><material><ambient>0.06 0.10 0.16 1</ambient><diffuse>0.10 0.22 0.38 1</diffuse></material></visual>
      <visual name="lm81_w1"><pose>350 600 461.5 0 0 0</pose><geometry><sphere><radius>0.6</radius></sphere></geometry><material><ambient>0.80 0.05 0.02 1</ambient><diffuse>0.95 0.08 0.03 1</diffuse><emissive>0.90 0.10 0.05 1</emissive></material></visual>
      <visual name="lm81_w2"><pose>350 600 320 0 0 0</pose><geometry><sphere><radius>0.5</radius></sphere></geometry><material><ambient>0.80 0.05 0.02 1</ambient><diffuse>0.95 0.08 0.03 1</diffuse><emissive>0.70 0.08 0.04 1</emissive></material></visual>
      <visual name="lm81_w3"><pose>350 600 160 0 0 0</pose><geometry><sphere><radius>0.5</radius></sphere></geometry><material><ambient>0.80 0.05 0.02 1</ambient><diffuse>0.95 0.08 0.03 1</diffuse><emissive>0.70 0.08 0.04 1</emissive></material></visual>
      <visual name="lm81_hp"><pose>350 600 421.3 0 0 0</pose><geometry><cylinder><radius>8</radius><length>0.08</length></cylinder></geometry><material><ambient>0.15 0.20 0.22 1</ambient><diffuse>0.28 0.36 0.40 1</diffuse></material></visual>
      <visual name="lm81_hph"><pose>350 600 421.4 0 0 0</pose><geometry><box><size>1.2 7 0.06</size></box></geometry><material><ambient>0.90 0.90 0.90 1</ambient><diffuse>0.95 0.95 0.92 1</diffuse></material></visual>
      <visual name="lm81_pod"><pose>350 600 5 0 0 0.05</pose><geometry><box><size>80 80 10</size></box></geometry><material><ambient>0.20 0.20 0.22 1</ambient><diffuse>0.36 0.36 0.40 1</diffuse></material></visual>
      <visual name="lm81_podr"><pose>350 600 10.4 0 0 0.05</pose><geometry><box><size>82 82 0.8</size></box></geometry><material><ambient>0.12 0.12 0.14 1</ambient><diffuse>0.24 0.24 0.28 1</diffuse></material></visual>
"""

# Insert before closing </link>
content = content.replace('    </link>', lm81_block + '\n    </link>', 1)

with open(path, 'w', encoding='utf-8') as f:
    f.write(content)

# Validate
ET.parse(path)
visuals = ET.parse(path).findall('.//visual')
lm81 = sum(1 for v in visuals if v.get('name','').startswith('lm81'))
print(f"Landmark 81 added! ({lm81} visuals)")
print(f"Total visuals: {len(visuals)}")
print(f"File size: {os.path.getsize(path)/1024:.1f} KB")
print("XML valid!")
