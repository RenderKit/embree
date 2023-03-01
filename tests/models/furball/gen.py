import random
import os

script_dir = os.path.dirname(os.path.abspath(__file__))

wiggle = 4

with open(script_dir + "/spheres_triangle.ecs", 'w') as f:
    for y in [-50, -25, 0, 25, 50]:
        for z in [0, 30, 60, 90, 120, 150, 180]:
            for x in [-40, 0, 40, 80, 120]:
                f.write(f"--triangle-sphere {x+random.randint(-wiggle, wiggle)} {y+random.randint(-wiggle, wiggle)} {z+random.randint(-wiggle, wiggle)} {random.randint(5, 13)} 100\n")
    f.write("--vp 8.961043358 58.70862198 -94.01120758 --vi 9.361638069 57.54679489 -91.27469635 --vu 0 1 0 --fov 90 --righthanded\n")

with open(script_dir + "/spheres_quads.ecs", 'w') as f:
    for y in [-50, -25, 0, 25, 50]:
        for z in [0, 30, 60, 90, 120, 150, 180]:
            for x in [-40, 0, 40, 80, 120]:
                f.write(f"--quad-sphere {x+random.randint(-wiggle, wiggle)} {y+random.randint(-wiggle, wiggle)} {z+random.randint(-wiggle, wiggle)} {random.randint(5, 13)} 100\n")
    f.write("--vp 8.961043358 58.70862198 -94.01120758 --vi 9.361638069 57.54679489 -91.27469635 --vu 0 1 0 --fov 90 --righthanded\n")

with open(script_dir + "/spheres_grids.ecs", 'w') as f:
    for y in [-50, -25, 0, 25, 50]:
        for z in [0, 30, 60, 90, 120, 150, 180]:
            for x in [-40, 0, 40, 80, 120]:
                f.write(f"--grid-sphere {x+random.randint(-wiggle, wiggle)} {y+random.randint(-wiggle, wiggle)} {z+random.randint(-wiggle, wiggle)} {random.randint(5, 13)} 100\n")
    f.write("--vp 8.961043358 58.70862198 -94.01120758 --vi 9.361638069 57.54679489 -91.27469635 --vu 0 1 0 --fov 90 --righthanded\n")

with open(script_dir + "/spheres_points.ecs", 'w') as f:
    for y in [-50, -25, 0, 25, 50]:
        for z in [0, 30, 60, 90, 120, 150, 180]:
            for x in [-40, 0, 40, 80, 120]:
                f.write(f"--point-sphere {x+random.randint(-wiggle, wiggle)} {y+random.randint(-wiggle, wiggle)} {z+random.randint(-wiggle, wiggle)} {random.randint(1, 100) / 10} {random.randint(5, 13)} 100\n")
    f.write("--vp 8.961043358 58.70862198 -94.01120758 --vi 9.361638069 57.54679489 -91.27469635 --vu 0 1 0 --fov 90 --righthanded\n")

types = [
    "linear_flat",
    "linear_round",
    "bezier_flat",
    "bezier_round",
    "bezier_normaloriented",
    "bspline_flat",
    "bspline_round",
    "bspline_normaloriented",
    "hermite_flat",
    "hermite_round",
    "hermite_normaloriented",
    "catmulrom_flat",
    "catmulrom_round",
    "catmulrom_normaloriented",
    ]


for t in types:
    id = 1
    with open(script_dir + "/furball_" + t + ".xml", 'w') as f:
        f.write("<?xml version=\"1.0\"?>\n")
        f.write("<scene>")
        for y in [-50, -25, 0, 25, 50]:
            for z in [0, 30, 60, 90, 120, 150, 180]:
                for x in [-40, 0, 40, 80, 120]:
                    f.write(f"  <Group id=\"{id}\">\n")
                    f.write(f"    <Transform id=\"{id+1}\">\n")
                    f.write(f"      <AffineSpace>\n")
                    f.write(f"        1.000000 0.000000 0.000000 {x+random.randint(-wiggle, wiggle)}\n")
                    f.write(f"        0.000000 1.000000 0.000000 {y+random.randint(-wiggle, wiggle)}\n")
                    f.write(f"        0.000000 0.000000 1.000000 {z+random.randint(-wiggle, wiggle)}\n")
                    f.write(f"      </AffineSpace>\n")
                    f.write(f"      <FurBall id=\"{id+2}\" radius=\"{random.randint(3,11)}\" hairtype=\"{t}\">\n")
                    f.write(f"        <material>\n")
                    f.write(f"          <code>\"OBJ\"</code>\n")
                    f.write(f"          <parameters>\n")
                    f.write(f"            <float3 name=\"Kd\">{random.randint(5,100)/100} {random.randint(5,100)/100} {random.randint(5,100)/100}</float3>\n")
                    f.write(f"          </parameters>\n")
                    f.write(f"        </material>\n")
                    f.write(f"      </FurBall>\n")
                    f.write(f"    </Transform>\n")
                    f.write(f"  </Group>\n")
                    id = id + 3

        f.write("</scene>\n")