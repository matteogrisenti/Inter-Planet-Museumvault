import json
import os
import shutil
from pathlib import Path
from renderer import Renderer, StateParser

class ComicBookGenerator:
    def __init__(self, trace_path: Path, output_dir: Path):
        self.trace_path = trace_path
        self.output_dir = output_dir / "comic_book"
        self.renderer = Renderer(trace_path)
        
        # Load the full history
        with open(trace_path, 'r') as f:
            self.history = json.load(f)

    def _identify_fixed_atoms(self):
        """Identifies atoms that never change throughout the entire plan."""
        if not self.history:
            return []
            
        # Start with the initial state atoms as candidates
        # Convert lists to tuples for set intersection
        candidates = set(tuple(a) for a in self.history[0]['state'])
        
        for step in self.history[1:]:
            current_state = set(tuple(a) for a in step['state'])
            candidates.intersection_update(current_state)
            
        # Convert back to list of lists
        return sorted([list(a) for a in candidates])

    def generate(self):
        """Creates the comic book assets."""
        if self.output_dir.exists():
            shutil.rmtree(self.output_dir)
        self.output_dir.mkdir(parents=True)

        print(f"   - Identifying Fixed Atoms (Map/Topology)...")
        fixed_atoms = self._identify_fixed_atoms()
        
        # We process step by step
        # Note: History index 0 is Initial State (no action). 
        # Plan steps start at index 1.
        
        total_steps = len(self.history)
        print(f"   - Generating {total_steps} pages...")

        for i, step_data in enumerate(self.history):
            # 1. Generate Image (The terminal state of this step)
            img_filename = f"step_{i:03d}.png"
            img_path = self.output_dir / img_filename
            
            # Use the renderer to draw this specific frame
            # We create a temporary figure to save
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(10, 6)) # Slightly smaller for comic book
            fig.patch.set_facecolor('#FDF5E6')
            
            self.renderer.render_frame(ax, i)
            plt.savefig(img_path, dpi=100, bbox_inches='tight')
            plt.close(fig)

            # 2. Generate JSON Data
            # For the viewer, we need: Action info, Initial Atoms, Terminal Atoms, Fixed Atoms
            
            # Initial state is the state of the PREVIOUS step (or empty if i=0)
            if i == 0:
                initial_atoms = [] # Before creation
                action_info = {
                    "name": "INITIALIZATION",
                    "parameters": [],
                    "add": [],
                    "del": []
                }
            else:
                prev_step = self.history[i-1]
                initial_atoms = [a for a in prev_step['state'] if a not in fixed_atoms]
                
                action_info = {
                    "name": step_data['action'],
                    "parameters": step_data.get('parameters', []),
                    "add": step_data.get('effects', {}).get('add', []),
                    "del": step_data.get('effects', {}).get('del', [])
                }

            # Terminal atoms (excluding fixed ones to reduce noise)
            terminal_atoms = [a for a in step_data['state'] if a not in fixed_atoms]

            page_data = {
                "step_id": i,
                "image_file": img_filename,
                "action": action_info,
                "fixed_atoms": fixed_atoms,
                "initial_atoms": initial_atoms,
                "terminal_atoms": terminal_atoms
            }

            json_filename = f"step_{i:03d}.json"
            with open(self.output_dir / json_filename, 'w') as f:
                json.dump(page_data, f, indent=2)

        print(f"✅ Comic Book generated in: {self.output_dir}")
