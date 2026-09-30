#!/usr/bin/env python3
"""
Generate a dense 10 km × 10 km city scenery for Gazebo Sim.
Outputs model.sdf with thousands of visual elements.
"""

import math
import random
import textwrap

random.seed(42)  # reproducible layout

# ── helpers ─────────────────────────────────────────────────────────
_id = 0
def uid(prefix="v"):
    global _id; _id += 1
    return f"{prefix}_{_id:04d}"

def vis(name, x, y, z, geom, mat, rot=0.0, cast_shadows=False):
    """Return one <visual> XML block."""
    cs = "false" if not cast_shadows else "true"
    return (
        f'      <visual name="{name}">'
        f'<pose>{x:.1f} {y:.1f} {z:.3f} 0 0 {rot:.4f}</pose>'
        f'<cast_shadows>{cs}</cast_shadows>'
        f'<geometry>{geom}</geometry>'
        f'<material>{mat}</material>'
        f'</visual>\n'
    )

def box(sx, sy, sz):
    return f'<box><size>{sx} {sy} {sz}</size></box>'

def cyl(r, l):
    return f'<cylinder><radius>{r}</radius><length>{l}</length></cylinder>'

def sphere(r):
    return f'<sphere><radius>{r}</radius></sphere>'

def cone(r, l):
    return f'<cone><radius>{r}</radius><length>{l}</length></cone>'

def mat(ar, ag, ab, dr, dg, db):
    return (f'<ambient>{ar:.2f} {ag:.2f} {ab:.2f} 1</ambient>'
            f'<diffuse>{dr:.2f} {dg:.2f} {db:.2f} 1</diffuse>')

def mat_e(ar, ag, ab, dr, dg, db, er, eg, eb):
    return (f'<ambient>{ar:.2f} {ag:.2f} {ab:.2f} 1</ambient>'
            f'<diffuse>{dr:.2f} {dg:.2f} {db:.2f} 1</diffuse>'
            f'<emissive>{er:.2f} {eg:.2f} {eb:.2f} 1</emissive>')

def mat_water():
    return (f'<ambient>0.03 0.14 0.24 1</ambient>'
            f'<diffuse>0.05 0.28 0.46 1</diffuse>'
            f'<specular>0.20 0.32 0.45 1</specular>')

# ── colour palettes ──────────────────────────────────────────────
WALL_COLORS = [
    (0.46, 0.38, 0.28,  0.68, 0.54, 0.38),   # warm tan
    (0.44, 0.34, 0.26,  0.65, 0.50, 0.36),   # sand
    (0.42, 0.34, 0.46,  0.62, 0.50, 0.68),   # lavender grey
    (0.30, 0.40, 0.34,  0.46, 0.62, 0.50),   # sage green
    (0.34, 0.38, 0.43,  0.52, 0.58, 0.66),   # steel blue
    (0.40, 0.32, 0.24,  0.60, 0.48, 0.36),   # brown
    (0.48, 0.33, 0.25,  0.70, 0.47, 0.34),   # terracotta
    (0.36, 0.28, 0.42,  0.54, 0.42, 0.64),   # mauve
    (0.14, 0.22, 0.30,  0.22, 0.42, 0.58),   # glass blue (towers)
    (0.24, 0.20, 0.30,  0.44, 0.34, 0.56),   # deep purple (towers)
    (0.22, 0.26, 0.22,  0.40, 0.52, 0.40),   # olive (towers)
    (0.30, 0.22, 0.18,  0.56, 0.40, 0.28),   # rust (towers)
    (0.28, 0.28, 0.30,  0.42, 0.44, 0.46),   # concrete grey
    (0.32, 0.26, 0.20,  0.58, 0.46, 0.34),   # khaki
    (0.20, 0.30, 0.24,  0.36, 0.54, 0.42),   # teal
    (0.42, 0.43, 0.45,  0.62, 0.64, 0.68),   # silver
]

ROOF_COLORS = [
    (0.25, 0.17, 0.12,  0.44, 0.28, 0.18),   # red-brown
    (0.18, 0.16, 0.18,  0.30, 0.27, 0.30),   # dark slate
    (0.14, 0.10, 0.08,  0.28, 0.20, 0.16),   # dark brown
    (0.08, 0.12, 0.16,  0.16, 0.24, 0.30),   # dark blue
    (0.10, 0.14, 0.11,  0.18, 0.26, 0.20),   # dark green
    (0.20, 0.14, 0.10,  0.38, 0.24, 0.16),   # copper
]

# ── exclusion zones (parks, water, airport runway, helipad) ──────
EXCLUSION_ZONES = []  # list of (cx, cy, half_w, half_h)

def in_exclusion(x, y, margin=30):
    for (cx, cy, hw, hh) in EXCLUSION_ZONES:
        if abs(x - cx) < hw + margin and abs(y - cy) < hh + margin:
            return True
    return False

# ── MAIN ────────────────────────────────────────────────────────
lines = []
W = lines.append  # shortcut

W('<?xml version="1.0"?>')
W('<sdf version="1.9">')
W('  <!--')
W('    Dense 10 km × 10 km urban scenery — procedurally generated.')
W('    Visual-only, no collision elements.')
W('  -->')
W('  <model name="p4_scenery">')
W('    <static>true</static>')
W('    <link name="scenery_link">')
W('')

# ════════════════════ GROUND ════════════════════
W(vis(uid("gnd"), 0, 0, 0,
      '<plane><normal>0 0 1</normal><size>10000 10000</size></plane>',
      mat(0.30, 0.33, 0.28, 0.42, 0.46, 0.38)))

# ════════════════════ HELIPAD ════════════════════
EXCLUSION_ZONES.append((0, 0, 18, 18))
W(vis(uid("hpad"), 0, 0, 0.030, cyl(15, 0.014), mat(0.13, 0.18, 0.20, 0.24, 0.34, 0.38)))
W(vis(uid("hpad"), 0, 0, 0.045, box(2.4, 14, 0.016), mat(0.90, 0.92, 0.90, 0.95, 0.96, 0.92)))
W(vis(uid("hpad"), -4.2, 0, 0.046, box(6.5, 2.4, 0.016), mat(0.90, 0.92, 0.90, 0.95, 0.96, 0.92)))
W(vis(uid("hpad"), 4.2, 0, 0.046, box(6.5, 2.4, 0.016), mat(0.90, 0.92, 0.90, 0.95, 0.96, 0.92)))

# ════════════════════ ROADS ════════════════════
# Major roads every 500 m, minor every 250 m
ROAD_H = []
ROAD_V = []

for offset in range(0, 5001, 500):
    for sign in ([1, -1] if offset else [1]):
        y = offset * sign
        ROAD_H.append(y)
        name = uid("rdH")
        W(vis(name, 0, y, 0.016, box(10000, 12, 0.032), mat(0.07, 0.08, 0.09, 0.12, 0.13, 0.15)))
        # centre marking
        W(vis(uid("mkH"), 0, y, 0.035, box(10000, 0.3, 0.014), mat(0.75, 0.68, 0.34, 0.90, 0.82, 0.42)))

for offset in range(0, 5001, 500):
    for sign in ([1, -1] if offset else [1]):
        x = offset * sign
        ROAD_V.append(x)
        W(vis(uid("rdV"), x, 0, 0.017, box(10000, 12, 0.032), mat(0.07, 0.08, 0.09, 0.12, 0.13, 0.15), rot=1.5708))
        W(vis(uid("mkV"), x, 0, 0.036, box(10000, 0.3, 0.014), mat(0.75, 0.68, 0.34, 0.90, 0.82, 0.42), rot=1.5708))

# Minor roads every 250 m (only odd multiples of 250 not already covered)
for offset in range(250, 5001, 500):
    for sign in [1, -1]:
        y = offset * sign
        W(vis(uid("rdHs"), 0, y, 0.015, box(10000, 7, 0.024), mat(0.07, 0.08, 0.09, 0.11, 0.12, 0.13)))
        x = offset * sign
        W(vis(uid("rdVs"), x, 0, 0.016, box(10000, 7, 0.024), mat(0.07, 0.08, 0.09, 0.11, 0.12, 0.13), rot=1.5708))

# ════════════════════ WATER ════════════════════
# Main river
W(vis(uid("riv"), 0, -3200, 0.010, box(10200, 80, 0.020), mat_water(), rot=0.15))
EXCLUSION_ZONES.append((0, -3200, 5100, 50))

# Canal
W(vis(uid("can"), 3800, 0, 0.010, box(40, 5100, 0.020), mat_water()))
EXCLUSION_ZONES.append((3800, 0, 30, 2550))

# Lakes
for lx, ly, lr in [(2800, 2600, 350), (-3200, -2400, 280), (380, -280, 55),
                     (-1200, 3600, 200), (4200, -1000, 180)]:
    W(vis(uid("lake"), lx, ly, 0.010, cyl(lr, 0.020), mat_water()))
    EXCLUSION_ZONES.append((lx, ly, lr, lr))

# ════════════════════ PARKS ════════════════════
PARK_DEFS = [
    (0, 800, 400, 300),
    (1800, 1800, 500, 350),
    (-2200, 1600, 450, 380),
    (2100, -2100, 550, 420),
    (-1800, -1900, 480, 360),
    (600, -800, 120, 80),
    (-800, 2800, 350, 300),
    (3200, 800, 300, 250),
    (-3800, -800, 400, 320),
    (1200, 3800, 320, 280),
]
for px, py, pw, ph in PARK_DEFS:
    r = random.uniform(-0.2, 0.2)
    W(vis(uid("park"), px, py, 0.014, box(pw, ph, 0.028),
          mat(0.07+random.uniform(0,0.03), 0.20+random.uniform(0,0.06), 0.06+random.uniform(0,0.02),
              0.14+random.uniform(0,0.04), 0.38+random.uniform(0,0.08), 0.10+random.uniform(0,0.04)),
          rot=r))
    EXCLUSION_ZONES.append((px, py, pw//2, ph//2))

# ════════════════════ AIRPORT ════════════════════
EXCLUSION_ZONES.append((-3500, 0, 250, 600))
W(vis(uid("apt"), -3500, 0, 10, box(400, 100, 20), mat(0.36, 0.36, 0.38, 0.56, 0.56, 0.60)))
W(vis(uid("apt"), -3500, 0, 20.5, box(402, 102, 1), mat(0.18, 0.18, 0.20, 0.30, 0.30, 0.34)))
W(vis(uid("apt"), -3500, 500, 0.013, box(50, 900, 0.026), mat(0.08, 0.09, 0.10, 0.13, 0.14, 0.16)))
W(vis(uid("apt"), -3500, -500, 0.013, box(50, 900, 0.026), mat(0.08, 0.09, 0.10, 0.13, 0.14, 0.16)))
# control tower
W(vis(uid("apt"), -3600, 200, 40, box(20, 20, 80), mat(0.14, 0.22, 0.28, 0.22, 0.42, 0.56)))
W(vis(uid("apt"), -3600, 200, 82, cyl(14, 4), mat(0.10, 0.18, 0.24, 0.18, 0.36, 0.50)))
W(vis(uid("apt"), -3600, 200, 90, cyl(0.4, 14), mat(0.18, 0.18, 0.18, 0.50, 0.50, 0.52)))

# ════════════════════ BUILDINGS ════════════════════
# Strategy: iterate every city block (250m grid), place 1-4 buildings per block
# Density varies by distance from centre:
#   0-500m   : CBD, tall towers 30-90m, 3-5 per block
#   500-1500m: inner residential, mid-rise 12-30m, 3-4 per block
#   1500-3000m: suburban, low-rise 6-18m, 2-4 per block
#   3000-5000m: rural/industrial, 1-3 per block

building_count = 0

for bx_idx in range(-19, 20):
    for by_idx in range(-19, 20):
        # block centre
        bcx = bx_idx * 250 + 125
        bcy = by_idx * 250 + 125
        
        dist = math.sqrt(bcx**2 + bcy**2)
        
        # determine zone
        if dist < 600:
            zone = "cbd"
            n_buildings = random.randint(3, 5)
            h_range = (30, 90)
            w_range = (25, 55)
        elif dist < 1500:
            zone = "inner"
            n_buildings = random.randint(3, 4)
            h_range = (12, 30)
            w_range = (20, 50)
        elif dist < 3000:
            zone = "suburb"
            n_buildings = random.randint(2, 4)
            h_range = (6, 18)
            w_range = (15, 42)
        else:
            zone = "rural"
            n_buildings = random.randint(1, 3)
            h_range = (5, 14)
            w_range = (12, 38)
        
        for _ in range(n_buildings):
            # random position within block (avoid edges = roads)
            ox = random.uniform(-90, 90)
            oy = random.uniform(-90, 90)
            bx = bcx + ox
            by = bcy + oy
            
            if abs(bx) > 4950 or abs(by) > 4950:
                continue
            if in_exclusion(bx, by):
                continue
            
            h = random.uniform(*h_range)
            w = random.uniform(*w_range)
            d = random.uniform(w * 0.5, w * 1.2)
            rot = random.uniform(-0.18, 0.18)
            
            wc = random.choice(WALL_COLORS)
            rc = random.choice(ROOF_COLORS)
            
            # building body
            W(vis(uid("bld"), bx, by, h/2, box(w, d, h), mat(*wc), rot=rot))
            # roof slab
            W(vis(uid("rof"), bx, by, h + 0.4, box(w+1, d+1, 0.8), mat(*rc), rot=rot))
            building_count += 1
            
            # CBD: add glazing bands + antenna
            if zone == "cbd" and h > 45:
                for band_z in [h*0.25, h*0.5, h*0.75]:
                    W(vis(uid("bnd"), bx, by, band_z, box(w+0.2, d+0.2, 0.5),
                          mat_e(0.05, 0.10, 0.16, 0.08, 0.20, 0.34, 0.01, 0.04, 0.06), rot=rot))
                # antenna
                W(vis(uid("ant"), bx, by, h + 6, cyl(0.3, 11), mat(0.18, 0.18, 0.18, 0.50, 0.50, 0.52)))

# ════════════════════ TREES ════════════════════
# Place trees: along roads, in parks, scattered in suburbs
tree_count = 0

# Trees along major roads (every 80m)
for y in ROAD_H:
    for x_pos in range(-4800, 4801, 80):
        if in_exclusion(x_pos, y + 18) or abs(x_pos) > 4950:
            continue
        if random.random() < 0.5:  # 50% chance to skip for variation
            continue
        # deciduous tree (trunk + sphere crown)
        trunk_h = random.uniform(3, 5)
        crown_r = random.uniform(1.8, 3.5)
        W(vis(uid("trt"), x_pos + random.uniform(-3, 3), y + 18, trunk_h/2, 
              cyl(random.uniform(0.3, 0.5), trunk_h),
              mat(0.18, 0.09, 0.03, 0.32, 0.17, 0.06)))
        W(vis(uid("trc"), x_pos + random.uniform(-3, 3), y + 18, trunk_h + crown_r * 0.7,
              sphere(crown_r),
              mat(0.06+random.uniform(0,0.04), 0.20+random.uniform(0,0.08), 0.05+random.uniform(0,0.03),
                  0.12+random.uniform(0,0.06), 0.38+random.uniform(0,0.14), 0.08+random.uniform(0,0.06))))
        tree_count += 1

# Trees along vertical roads (every 100m, sparser)
for x in ROAD_V:
    for y_pos in range(-4800, 4801, 100):
        if in_exclusion(x + 18, y_pos) or abs(y_pos) > 4950:
            continue
        if random.random() < 0.6:
            continue
        trunk_h = random.uniform(3, 5)
        crown_r = random.uniform(1.8, 3.5)
        W(vis(uid("trt"), x + 18, y_pos + random.uniform(-3, 3), trunk_h/2,
              cyl(random.uniform(0.3, 0.5), trunk_h),
              mat(0.18, 0.09, 0.03, 0.32, 0.17, 0.06)))
        W(vis(uid("trc"), x + 18, y_pos + random.uniform(-3, 3), trunk_h + crown_r * 0.7,
              sphere(crown_r),
              mat(0.06+random.uniform(0,0.04), 0.20+random.uniform(0,0.08), 0.05+random.uniform(0,0.03),
                  0.12+random.uniform(0,0.06), 0.38+random.uniform(0,0.14), 0.08+random.uniform(0,0.06))))
        tree_count += 1

# Park trees (dense clusters)
for px, py, pw, ph in PARK_DEFS:
    n_park_trees = int((pw * ph) / 2000)  # ~1 tree per 2000 m²
    for _ in range(n_park_trees):
        tx = px + random.uniform(-pw/2 + 10, pw/2 - 10)
        ty = py + random.uniform(-ph/2 + 10, ph/2 - 10)
        style = random.choice(["deciduous", "conifer"])
        if style == "deciduous":
            trunk_h = random.uniform(3, 6)
            crown_r = random.uniform(2.0, 4.0)
            W(vis(uid("ptt"), tx, ty, trunk_h/2, cyl(random.uniform(0.3, 0.5), trunk_h),
                  mat(0.18, 0.09, 0.03, 0.32, 0.17, 0.06)))
            W(vis(uid("ptc"), tx, ty, trunk_h + crown_r * 0.7, sphere(crown_r),
                  mat(0.06+random.uniform(0,0.04), 0.22+random.uniform(0,0.06), 0.05+random.uniform(0,0.03),
                      0.12+random.uniform(0,0.06), 0.42+random.uniform(0,0.10), 0.08+random.uniform(0,0.06))))
        else:
            trunk_h = random.uniform(4, 8)
            W(vis(uid("ptt"), tx, ty, trunk_h/2, cyl(random.uniform(0.3, 0.5), trunk_h),
                  mat(0.14, 0.07, 0.02, 0.30, 0.15, 0.04)))
            W(vis(uid("ptl"), tx, ty, trunk_h + 2, cone(random.uniform(2.5, 4.0), random.uniform(4, 6)),
                  mat(0.04, 0.18, 0.04, 0.10, 0.38, 0.08)))
            W(vis(uid("pth"), tx, ty, trunk_h + 4.5, cone(random.uniform(1.8, 2.8), random.uniform(3.5, 5.5)),
                  mat(0.03, 0.15, 0.03, 0.08, 0.31, 0.06)))
        tree_count += 1

# Outer forest edge (ring at 4000-4800m)
for angle_deg in range(0, 360, 3):
    angle = math.radians(angle_deg)
    r_dist = random.uniform(4000, 4800)
    fx = r_dist * math.cos(angle)
    fy = r_dist * math.sin(angle)
    if abs(fx) > 4950 or abs(fy) > 4950 or in_exclusion(fx, fy):
        continue
    trunk_h = random.uniform(5, 9)
    W(vis(uid("ftt"), fx, fy, trunk_h/2, cyl(random.uniform(0.4, 0.6), trunk_h),
          mat(0.14, 0.07, 0.02, 0.30, 0.15, 0.04)))
    W(vis(uid("ftl"), fx, fy, trunk_h + 2.5, cone(random.uniform(3.0, 4.5), random.uniform(5, 7)),
          mat(0.04, 0.18, 0.04, 0.10, 0.38, 0.08)))
    W(vis(uid("fth"), fx, fy, trunk_h + 5.5, cone(random.uniform(2.0, 3.2), random.uniform(4, 6)),
          mat(0.03, 0.15, 0.03, 0.08, 0.31, 0.06)))
    tree_count += 1

# ════════════════════ STREET LAMPS ════════════════════
lamp_count = 0
# Along major roads, every 60m
for y in ROAD_H:
    for x_pos in range(-4800, 4801, 60):
        if random.random() < 0.3:
            continue
        if in_exclusion(x_pos, y - 8):
            continue
        W(vis(uid("lpp"), x_pos, y - 8, 3, cyl(0.10, 6), mat(0.12, 0.13, 0.14, 0.30, 0.32, 0.34)))
        W(vis(uid("lph"), x_pos, y - 8, 6.2, sphere(0.28),
              mat_e(0.75, 0.66, 0.32, 0.95, 0.82, 0.42, 0.25, 0.18, 0.05)))
        lamp_count += 1

# ════════════════════ LANDMARK TOWERS ════════════════════
LANDMARKS = [
    (4200, 4200, 120, 60, 60, 0.20),
    (-4300, -4100, 100, 55, 55, -0.15),
    (4100, -4200, 90, 50, 50, 0.10),
    (-4000, 4300, 110, 58, 52, -0.08),
    (-176, -162, 54, 22, 22, -0.12),
    (178, 152, 46, 26, 20, 0.15),
    (-174, 42, 38, 18, 24, -0.12),
    (172, -150, 62, 24, 24, 0.15),
]
for lx, ly, lh, lw, ld, lr in LANDMARKS:
    wc = random.choice(WALL_COLORS[8:12])  # tower colours
    W(vis(uid("lmt"), lx, ly, lh/2, box(lw, ld, lh), mat(*wc), rot=lr))
    # bands
    for bz in [lh*0.25, lh*0.5, lh*0.75]:
        W(vis(uid("lmb"), lx, ly, bz, box(lw+0.2, ld+0.2, 0.5),
              mat_e(0.05, 0.10, 0.14, 0.10, 0.28, 0.38, 0.01, 0.03, 0.04), rot=lr))
    # roof
    W(vis(uid("lmr"), lx, ly, lh + 0.6, box(lw+1.2, ld+1.2, 1.2), mat(0.09, 0.11, 0.13, 0.18, 0.22, 0.25), rot=lr))
    # antenna
    W(vis(uid("lma"), lx, ly, lh + 8, cyl(0.35, 14), mat(0.18, 0.18, 0.18, 0.50, 0.50, 0.52)))

# ════════════════════ BRIDGES over river ════════════════════
for bx in range(-4500, 4501, 500):
    river_y = -3200 + bx * math.tan(0.15)
    W(vis(uid("brg"), bx, river_y, 3, box(14, 100, 1.5), mat(0.20, 0.20, 0.22, 0.38, 0.38, 0.42)))
    # bridge railing
    W(vis(uid("brl"), bx, river_y - 48, 4, box(14, 1, 2.5), mat(0.16, 0.16, 0.18, 0.30, 0.30, 0.34)))
    W(vis(uid("brl"), bx, river_y + 48, 4, box(14, 1, 2.5), mat(0.16, 0.16, 0.18, 0.30, 0.30, 0.34)))

# ════════════════════ CLOSE ════════════════════
W('    </link>')
W('  </model>')
W('</sdf>')

# ── Write output ────────────────────────────────────────────────
output = "\n".join(lines)
out_path = __file__.replace("generate_dense_city.py", "model.sdf")
with open(out_path, "w", encoding="utf-8") as f:
    f.write(output)

total_visuals = output.count("<visual ")
print(f"=== Dense City Generator ===")
print(f"Buildings:      {building_count}")
print(f"Trees:          {tree_count}")
print(f"Street lamps:   {lamp_count}")
print(f"Total <visual>: {total_visuals}")
print(f"File size:      {len(output)/1024:.1f} KB")
print(f"Written to:     {out_path}")
