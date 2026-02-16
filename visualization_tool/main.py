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
    from optic_parser import OpticParser
    from temporal_diagram import TemporalDiagram
    from combined_optic_viz import CombinedOpticVisualization
except ImportError as e:
    print(f"❌ Error importing modules: {e}")
    print("Ensure all required modules are in the same directory.")
    sys.exit(1)

def run_visualization(input_dir: Path, output_dir: Path, mode: str, max_steps_limit: int = None):
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        # --- MODE 6: POLICY VISUALIZATION ---
        if mode == '6':
            # Delegate completely to the external script
            run_policy_viz(input_dir, output_dir)
            return
        
        # --- MODE 7: OPTIC TEMPORAL VISUALIZATION (STATIC) ---
        elif mode == '7':
            output_file = input_dir / "output.txt"
            if not output_file.exists():
                print(f"❌ Error: output.txt not found in {input_dir}")
                print("   Mode 7 requires an OPTIC output file named 'output.txt'")
                return
            
            print(f"--- Parsing OPTIC Output from {input_dir.name} ---")
            parser = OpticParser(output_file)
            timeline_data = parser.parse()
            
            # Get problem.pddl path for earthquake data
            problem_file = input_dir / "problem.pddl" if (input_dir / "problem.pddl").exists() else None
            
            # Generate temporal diagram
            diagram = TemporalDiagram(timeline_data, problem_file)
            diagram.print_summary()
            
            diagram_path = output_dir / "temporal_diagram.png"
            diagram.render(diagram_path)
            
            print(f"\n✅ OPTIC Temporal Visualization Complete!")
            print(f"   📊 Diagram: {diagram_path}")
            return
        
        # --- MODE 8: OPTIC COMBINED ANIMATION (MAP + TIMELINE) ---
        elif mode == '8':
            output_file = input_dir / "output.txt"
            if not output_file.exists():
                print(f"❌ Error: output.txt not found in {input_dir}")
                print("   Mode 8 requires an OPTIC output file named 'output.txt'")
                return
            
            print(f"--- Parsing OPTIC Output from {input_dir.name} ---")
            parser = OpticParser(output_file)
            timeline_data = parser.parse()
            
            # Optional: include problem.pddl for better location layout
            problem_file = input_dir / "problem.pddl" if (input_dir / "problem.pddl").exists() else None
            
            # Look for trace.json for full map rendering
            trace_file = output_dir / "trace.json"
            if not trace_file.exists():
                # Try to generate it first using the standard parser
                domain_file = input_dir / "domain.pddl"
                problem_file_pddl = input_dir / "problem.pddl"
                plan_file = input_dir / "sas_plan"
                
                if domain_file.exists() and problem_file_pddl.exists() and plan_file.exists():
                    print(f"Generating trace.json for full map rendering...")
                    try:
                        world = World(domain_file, problem_file_pddl, plan_file)
                        world.save_trace_to_json(output_dir)
                        print(f"✅ trace.json generated")
                    except Exception as e:
                        print(f"⚠️  Could not generate trace.json: {e}")
                        trace_file = None
                else:
                    trace_file = None
            
            # Generate combined animation
            viz = CombinedOpticVisualization(timeline_data, problem_file)
            gif_path = output_dir / "combined_animation.gif"
            viz.render_combined_gif(gif_path, fps=2, max_frames=100, trace_file=trace_file)
            
            print(f"\n✅ OPTIC Combined Animation Complete!")
            print(f"   🎬 Animation: {gif_path}")
            print(f"   🎥 Video: {gif_path.with_suffix('.mp4')}")
            return

        # --- MODES 1-5: STANDARD VISUALIZATION ---
        domain_file = input_dir / "domain.pddl"
        problem_file = input_dir / "problem.pddl"
        plan_file = input_dir / "sas_plan"

        print(f"--- Parsing Files in {input_dir.name} ---")
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

    except Exception as e:
        print(f"❌ Execution Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description="PDDL Planning Visualizer")
    parser.add_argument("directory", type=str, help="Directory containing PDDL files")
    parser.add_argument("--output", type=str, default="viz", help="Output directory name")
    parser.add_argument("--mode", type=str, choices=['1', '2', '3', '4', '5', '6', '7', '8'], help="Directly select mode")
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
        print("  7. OPTIC Temporal Visualization (Static Timeline)")
        print("  8. OPTIC Combined Animation (Map + Timeline GIF)")
        choice = input("\nChoice (1-8): ").strip()
    
    if choice not in ['1', '2', '3', '4', '5', '6', '7', '8']:
        print("Invalid choice. Exiting.")
        return

    run_visualization(input_path, output_path, choice, max_steps_limit=args.steps)

    print("\n" + "="*60)
    print(" Process Complete! ".center(60, " "))
    print(f" 📂 Data saved to: {output_path}".center(60, " "))
    print("="*60)

if __name__ == "__main__":
    main()
