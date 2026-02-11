#!/usr/bin/env python3
import sys
import argparse
import subprocess
from pathlib import Path

# Import external modules
try:
    from parser import World
    from renderer import Renderer
    from comic_book import ComicBookGenerator
    from policy_visualization import run_policy_viz
except ImportError as e:
    print(f"❌ Error importing modules: {e}")
    print("Ensure parser.py, renderer.py, comic_book.py, and policy_visualization.py are in the same directory.")
    sys.exit(1)

def run_visualization(input_dir: Path, output_dir: Path, mode: str, max_steps_limit: int = None):
    output_dir.mkdir(parents=True, exist_ok=True)

    # --- MODES 1-5: STANDARD VISUALIZATION ---
    domain_file = input_dir / "domain.pddl"
    problem_file = input_dir / "problem.pddl"
    plan_file = input_dir / "sas_plan"

    print(f"--- Parsing Files in {input_dir.name} ---")
    try:
        world = World(domain_file, problem_file, plan_file)
        world.save_trace_to_json(output_dir)
        trace_path = output_dir / "trace.json"

        renderer = Renderer(trace_path)

        if mode == '1':
            output_file = output_dir / "full_animation.gif"
            print(f"Generating Full Animated GIF...")
            renderer.save_gif(output_file, max_frames=max_steps_limit, interval=500)
            
        elif mode == '2':
            output_file = output_dir / "preview_animation.gif"
            print(f"Generating Preview GIF...")
            renderer.save_gif(output_file, max_frames=50, interval=500)
            
        elif mode == '3':
            output_file = output_dir / "summary.png"
            print(f"Generating Static Summary...")
            renderer.save_static(output_file)
            
        elif mode == '4':
            print("--- Generating Comic Book ---")
            gen = ComicBookGenerator(trace_path, output_dir)
            gen.generate()
            print("Run mode '5' to view it!")
            
        elif mode == '5':
            comic_dir = output_dir / "comic_book"
            if not comic_dir.exists():
                print("❌ Comic book not found. Run mode 4 first.")
                return
            
            print("Opening Viewer...")
            script_path = Path("comic_viewer.py").resolve()
            subprocess.run([sys.executable, str(script_path), str(comic_dir)])
        
        # --- MODE 6: POLICY VISUALIZATION ---
        elif mode == '6':
            # Delegate completely to the external script
            run_policy_viz(input_dir, output_dir)
            return

    except Exception as e:
        print(f"❌ Execution Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description="PDDL Planning Visualizer")
    parser.add_argument("directory", type=str, help="Directory containing PDDL files")
    parser.add_argument("--output", type=str, default="viz", help="Output directory name")
    parser.add_argument("--mode", type=str, choices=['1', '2', '3', '4', '5', '6'], help="Directly select mode")
    parser.add_argument("--steps", type=int, default=None, help="Limit number of steps (for Mode 1)")
    
    args = parser.parse_args()

    input_path = Path(args.directory).resolve()
    # Output path is relative to the input folder
    output_path = input_path / args.output

    if not input_path.exists():
        print(f"❌ Error: Input directory does not exist: {input_path}")
        return

    print("="*60)
    print(" PDDL VISUALIZER ENGINE ".center(60, "="))
    print("="*60)

    # Menu Logic
    if args.mode:
        choice = args.mode
    else:
        print("\nSelect visualization mode:")
        print("  1. Full Animated GIF")
        print("  2. Preview Animated GIF (First 50 steps)")
        print("  3. Static Comparison (Initial vs Final)")
        print("  4. Generate Comic Book (Plots & Text)")
        print("  5. View Comic Book (Interactive)")
        print("  6. Policy Graph Visualization (Cyclic)")
        choice = input("\nChoice (1-6): ").strip()
    
    if choice not in ['1', '2', '3', '4', '5', '6']:
        print("Invalid choice. Exiting.")
        return

    run_visualization(input_path, output_path, choice, max_steps_limit=args.steps)

    print("\n" + "="*60)
    print(" Process Complete! ".center(60, " "))
    print(f" 📂 Data saved to: {output_path}".center(60, " "))
    print("="*60)

if __name__ == "__main__":
    main()