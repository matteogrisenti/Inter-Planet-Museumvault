#!/usr/bin/env python3
import sys
import argparse
from pathlib import Path
from parser import PDDLParser
from world import WorldState
from renderer import Visualizer

def run_visualization(input_dir: Path, output_dir: Path, mode: str):
    """
    Orchestrates the parsing and rendering based on the selected mode.
    """
    # 1. File Path Setup
    domain_file = input_dir / "domain.pddl"
    problem_file = input_dir / "problem.pddl"
    plan_file = input_dir / "sas_plan"

    for f in [domain_file, problem_file, plan_file]:
        if not f.exists():
            print(f"❌ Error: Required file not found: {f}")
            sys.exit(1)

    # 2. Parsing Phase
    print(f"--- Parsing Files in {input_dir.name} ---")
    try:
        domain_data = PDDLParser.parse_domain(str(domain_file))
        problem_data = PDDLParser.parse_problem(str(problem_file))
        plan = PDDLParser.parse_plan(str(plan_file))
    except Exception as e:
        print(f"❌ Parsing failed: {e}")
        sys.exit(1)

    print(f"✔ Domain: {domain_data['name']}")
    print(f"✔ Plan Length: {len(plan)} actions")

    # 3. Visualization Phase
    viz = Visualizer(problem_data, plan)
    
    # Create the output directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)

    if mode in ['1']:  # Full Animation
        out = output_dir / "pddl_full_simulation.gif"
        print(f"🎬 Creating full animation: {out.name}...")
        viz.create_animation(str(out), interval=800)

    if mode == '2':  # Preview Animation
        out = output_dir / "pddl_preview_50.gif"
        print(f"🎬 Creating preview (50 steps): {out.name}...")
        viz.create_animation(str(out), interval=600, max_steps=50)

    if mode in ['3']:  # Static Comparison
        out = output_dir / "pddl_static_comparison.png"
        print(f"🖼 Creating static comparison: {out.name}...")
        viz.create_static_visualization(str(out))

def main():
    parser = argparse.ArgumentParser(description="PDDL Planning Visualizer")
    parser.add_argument("directory", type=str, help="Directory containing PDDL files")
    # Changed default to "viz" (relative path) so it works correctly with path joining
    parser.add_argument("--output", type=str, default="viz", help="Output directory name (subdirectory of input)")
    args = parser.parse_args()

    input_path = Path(args.directory).resolve()
    
    # Join input_path with the output argument to ensure it is a subdirectory
    # We strip any leading '/' from args.output to prevent it from being treated as an absolute path
    output_path = input_path / args.output.lstrip("/")

    if not input_path.exists():
        print(f"❌ Error: Input directory does not exist: {input_path}")
        return

    print("="*60)
    print(" PDDL VISUALIZER ENGINE ".center(60, "="))
    print("="*60)

    print("\nSelect visualization mode:")
    print("  1. Full Animated GIF")
    print("  2. Preview Animated GIF (First 50 steps)")
    print("  3. Static Comparison (Initial vs Final)")
    
    choice = input("\nChoice (1-3): ").strip()
    
    if choice not in ['1', '2', '3']:
        print("Invalid choice. Exiting.")
        return

    run_visualization(input_path, output_path, choice)

    print("\n" + "="*60)
    print(" Visualization Complete! ".center(60, " "))
    print(f" 📂 Output saved to: {output_path}".center(60, " "))
    print("="*60)

if __name__ == "__main__":
    main()