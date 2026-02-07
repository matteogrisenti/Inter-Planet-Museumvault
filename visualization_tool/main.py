#!/usr/bin/env python3
import sys
import argparse
import json
from pathlib import Path
from parser import PDDLParser
from world import WorldState
from renderer import Visualizer
from comic_book import ComicBookGenerator, ComicBookViewer

def save_debug_json(problem_data, plan, output_path):
    """Simulates the plan and saves initial/final states to JSON."""
    
    # 1. Capture Initial State
    sim_state = WorldState(problem_data)
    initial_dump = sim_state.to_dict()
    
    # 2. Run Simulation
    log = []
    for action_name, parameters in plan:
        desc = sim_state.apply_action(action_name, parameters)
        log.append({
            "action": action_name,
            "parameters": parameters,
            "description": desc
        })
    
    # 3. Capture Final State
    final_dump = sim_state.to_dict()
    
    # 4. Compare Artifacts
    initial_artifacts = set(initial_dump['artifact_locations'].keys())
    final_artifacts = set(final_dump['artifact_locations'].keys())
    
    new_artifacts = list(final_artifacts - initial_artifacts)
    
    debug_data = {
        "analysis": {
            "initial_artifact_count": initial_dump['artifact_count'],
            "final_artifact_count": final_dump['artifact_count'],
            "new_artifacts_detected": new_artifacts
        },
        "initial_state": initial_dump,
        "final_state": final_dump,
        "action_log": log
    }
    
    json_path = output_path / "debug_world_states.json"
    with open(json_path, 'w') as f:
        json.dump(debug_data, f, indent=4)
        
    print(f"\n🐛 Debug log saved to: {json_path}")
    if new_artifacts:
        print(f"⚠️  WARNING: Found {len(new_artifacts)} new artifacts in final state that were not in initial state!")
        print(f"   Names: {', '.join(new_artifacts)}")

def run_visualization(input_dir: Path, output_dir: Path, mode: str):
    """Orchestrates parsing and rendering."""
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
        # Print full traceback for debugging
        import traceback
        traceback.print_exc()
        sys.exit(1)

    print(f"✔ Domain: {domain_data['name']}")
    print(f"✔ Plan Length: {len(plan)} actions")

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    # 3. Debug Phase (Generate JSON)
    # print("--- Generating Debug Log ---")
    # save_debug_json(problem_data, plan, output_dir)

    # 4. Visualization Phase
    viz = Visualizer(problem_data, plan)
    
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
    
    elif mode == '4': # Generate Comic Book
        gen = ComicBookGenerator(problem_data, plan, output_dir)
        gen.generate()
        
    elif mode == '5': # View Comic Book
        viewer = ComicBookViewer(output_dir)
        print("📖 Opening Comic Book Viewer... (Use Left/Right Arrow Keys)")
        viewer.show()

def main():
    parser = argparse.ArgumentParser(description="PDDL Planning Visualizer")
    parser.add_argument("directory", type=str, help="Directory containing PDDL files")
    parser.add_argument("--output", type=str, default="viz", help="Output directory name")
    
    # We allow mode to be passed as argument to skip the menu
    parser.add_argument("--mode", type=str, choices=['1', '2', '3', '4', '5'], help="Directly select mode")
    args = parser.parse_args()

    input_path = Path(args.directory).resolve()
    output_path = input_path / args.output.lstrip("/")

    if not input_path.exists():
        print(f"❌ Error: Input directory does not exist: {input_path}")
        return

    print("="*60)
    print(" PDDL VISUALIZER ENGINE ".center(60, "="))
    print("="*60)

    if args.mode:
        choice = args.mode
    else:
        print("\nSelect visualization mode:")
        print("  1. Full Animated GIF")
        print("  2. Preview Animated GIF (First 50 steps)")
        print("  3. Static Comparison (Initial vs Final)")
        print("  4. Generate Comic Book (Plots & Text)")
        print("  5. View Comic Book (Interactive)")
        
        choice = input("\nChoice (1-5): ").strip()
    
    if choice not in ['1', '2', '3', '4', '5']:
        print("Invalid choice. Exiting.")
        return

    run_visualization(input_path, output_path, choice)

    print("\n" + "="*60)
    print(" Visualization Complete! ".center(60, " "))
    print(f" 📂 Output saved to: {output_path}".center(60, " "))
    print("="*60)

if __name__ == "__main__":
    main()