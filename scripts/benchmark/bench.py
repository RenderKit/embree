import os
import json
import argparse
import subprocess

# Parse arguments
parser = argparse.ArgumentParser(description="Benchmark script")
parser.add_argument('--presets', nargs='*', default=[], help="Whitelist of presets, defaults to all presets if not specified")
parser.add_argument('--models', default="", help="Path to embree models repository, accepts 'git clone' to clone into current path")
parser.add_argument('--size', nargs=2, type=int, default=[2048, 2048], help="Width and height (default: 2048 2048)")
parser.add_argument('--out', default="./benchmarks_out", help="Output directory for benchmark results (default: benchmarks_out)")
args = parser.parse_args()


# Ensure the current working directory is the root directory of the repository
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
os.chdir(repo_root)
print(f"Changed working directory to repository root: {repo_root}")


# PRESETS #
###########
with open("./scripts/benchmark/presets.json", 'r') as file:
    presets_data = json.load(file)

presets = []
for p in presets_data["configurePresets"]:
    if not args.presets or p["name"] in args.presets:
        presets.append(p["name"])

for p in args.presets:
    if p not in presets:
        print(f"WARNING: Unknown preset '{p}' ignored.")



# BUILD EMBREE #
################
for preset in presets:
    print(f"Building embree with preset: {preset}")
    try:
        subprocess.run(["cmake", "-S", ".", "-B", f"build-{preset}", "--preset", preset], check=True)
        subprocess.run(["cmake", "--build", f"build-{preset}", "-j8"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error while building embree with preset {preset}:\n{e.stderr}")
        continue


# MODELS #
##########
if args.models == "git clone":
    print("Cloning embree models repository...")
    os.system(f"git lfs install")
    os.system(f"git clone https://github.com/intel-innersource/libraries.graphics.renderkit.embree.models.git ./embree-models")
    models_repo = os.path.abspath("./embree-models")
else:
    models_repo = os.path.abspath(args.models)

ecs_files = []
for root, _, files in os.walk(models_repo):
    for file in files:
        if file.endswith(".ecs"):
            ecs_files.append(os.path.join(root, file))

# Check for duplicate filenames in ecs_files
filenames = [os.path.basename(ecs) for ecs in ecs_files]
duplicates = set([name for name in filenames if filenames.count(name) > 1])

if duplicates:
    for duplicate in duplicates:
        print(f"WARNING: Duplicate filename detected: {duplicate}")


# RUN BENCHMARKS #
##################

def runbench(preset, exe, ecs):
    print(os.path.abspath(f"./{args.out}/viewer_{preset}_{os.path.basename(ecs)[:-4]}.json"))
    cmd = [
        f"./build-{preset}/{exe}",
        "-c", ecs,
        "--size", f"{args.size[0]}", f"{args.size[1]}",
        "--benchmark", "1", "8", f"--benchmark_out={os.path.abspath(f"./{args.out}/viewer_{preset}_{os.path.basename(ecs)[:-4]}.json")}"
    ]
    try:
        print(f"embree viewer {preset}: {ecs} ...")
        result = subprocess.run(cmd, check=True, text=True, capture_output=True)
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print(f"Error while processing {ecs} with preset {preset}:\n{e.stderr}")
    print("====================================================================================")


# Ensure the output directory exists
if not os.path.exists(args.out):
    os.makedirs(args.out)
    print(f"Created output directory: {args.out}")


for preset in presets:
    for ecs in ecs_files:
        runbench(preset, "embree_viewer", ecs)
        runbench(preset, "embree_pathtracer", ecs)



