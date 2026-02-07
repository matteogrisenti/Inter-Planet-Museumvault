import os
import json
import sys
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from pathlib import Path
from typing import List, Dict, Any
from world import WorldState
from renderer import Visualizer

class ComicBookGenerator:
    def __init__(self, problem_data: Dict[str, Any], plan: List, output_dir: Path):
        self.problem_data = problem_data
        self.plan = plan
        self.output_dir = output_dir / "comic_book"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.visualizer = Visualizer(problem_data, plan)
        
    def generate(self):
        """Generates the comic book pages (images and structured text logs)."""
        print(f"🎨 Generating Comic Book in: {self.output_dir}")
        
        sim = WorldState(self.problem_data)
        
        # --- PROLOGUE ---
        self._save_page(
            page_num=0,
            title="Prologue: The Vault Opens",
            state=sim,
            text_lines=[
                "INITIAL STATE:",
                f"  • Robot Location: {sim.robot_location}",
                f"  • Artifacts to move: {len(sim.artifact_locations)}",
                "",
                "PLAN OVERVIEW:",
                f"  • Total Steps: {len(self.plan)}",
                "  • Mission Start..."
            ]
        )
        
        episode_count = 1
        current_episode_actions = []
        transit_actions = [] 
        
        in_mission = False
        
        # We need to capture the state *before* the pickup starts
        start_state_desc = []
        
        for action_name, params in self.plan:
            # 1. CAPTURE START OF EPISODE (Pick Up)
            if 'pick-up' in action_name and not in_mission:
                in_mission = True
                
                # Snapshot the "Initial State" of this episode
                # (The state after transit but before the pickup)
                start_state_desc = [
                    f"  • Robot Location: {sim.robot_location}",
                    f"  • Hand Status: {'Empty' if sim.hand_empty else 'Occupied'}"
                ]
                if transit_actions:
                    start_state_desc.append(f"  • Context: Arrived after {len(transit_actions)} transit steps.")
                
                # Apply the pick-up action
                desc = sim.apply_action(action_name, params)
                current_episode_actions.append(desc)
                
            # 2. CAPTURE END OF EPISODE (Drop)
            elif 'drop' in action_name and in_mission:
                # Apply the drop action
                desc = sim.apply_action(action_name, params)
                current_episode_actions.append(desc)
                in_mission = False
                
                # Snapshot the "Terminal State"
                term_state_desc = [
                    f"  • Robot Location: {sim.robot_location}",
                    f"  • Action Result: {desc}"
                ]
                
                # Build the structured text
                lines = [
                    f"EPISODE {episode_count}",
                    "=" * 30,
                    "",
                    "INITIAL STATE:",
                    *start_state_desc,
                    "",
                    "ACTIONS:",
                    *[f"  {i+1}. {act}" for i, act in enumerate(current_episode_actions)],
                    "",
                    "TERMINAL STATE:",
                    *term_state_desc
                ]
                
                self._save_page(
                    page_num=episode_count,
                    title=f"Episode {episode_count}: Delivery Complete",
                    state=sim,
                    text_lines=lines
                )
                
                episode_count += 1
                current_episode_actions = []
                transit_actions = []
                
            else:
                # Apply intermediate actions
                desc = sim.apply_action(action_name, params)
                
                if in_mission:
                    current_episode_actions.append(desc)
                else:
                    transit_actions.append(desc)
        
        # --- EPILOGUE ---
        if transit_actions or current_episode_actions:
             final_log = transit_actions + current_episode_actions
             self._save_page(
                page_num=episode_count,
                title="Epilogue: Mission Debrief",
                state=sim,
                text_lines=["FINAL STATUS", "-"*20, "Remaining Actions:"] + final_log
            )
            
        print(f"✨ Comic Book Generation Complete! ({episode_count} pages)")

    def _save_page(self, page_num: int, title: str, state: WorldState, text_lines: List[str]):
        """Saves a page as PNG and JSON."""
        # 1. Save Image
        filename_base = f"page_{page_num:03d}"
        img_path = self.output_dir / f"{filename_base}.png"
        
        # We create a larger figure to ensure high quality map
        fig, ax = plt.subplots(figsize=(12, 10))
        self.visualizer._draw_state(ax, state, title)
        plt.savefig(img_path, bbox_inches='tight', dpi=100)
        plt.close(fig)
        
        # 2. Save Text Data
        json_path = self.output_dir / f"{filename_base}.json"
        page_data = {
            "title": title,
            "page_num": page_num,
            "text": text_lines
        }
        with open(json_path, 'w') as f:
            json.dump(page_data, f, indent=4)

class ComicBookViewer:
    def __init__(self, output_dir: Path):
        self.data_dir = output_dir / "comic_book"
        if not self.data_dir.exists():
            print(f"❌ No comic book data found at {self.data_dir}.")
            print("   Please run option 4 (Generate Comic) first.")
            sys.exit(1)
            
        self.pages = sorted(self.data_dir.glob("*.json"))
        if not self.pages:
            print("❌ No pages found.")
            sys.exit(1)
            
        self.current_idx = 0
        self.fig = None
        self.ax_img = None
        self.ax_text = None

    def show(self):
        """Launches the interactive viewer."""
        # Setup Figure: 16x9 aspect ratio works well for screens
        self.fig = plt.figure(figsize=(18, 9)) 
        self.fig.canvas.manager.set_window_title("PDDL Comic Book Viewer")
        self.fig.patch.set_facecolor('#2C3E50') # Dark slate frame
        
        # GridSpec: 1 Row, 2 Columns. 
        # width_ratios=[1.2, 0.8] gives slightly more space to the map (left) than text (right)
        gs = self.fig.add_gridspec(1, 2, width_ratios=[1.2, 0.8])
        
        self.ax_img = self.fig.add_subplot(gs[0, 0]) 
        self.ax_text = self.fig.add_subplot(gs[0, 1]) 
        
        # Navigation Buttons (Bottom Right Corner)
        # Position: [left, bottom, width, height]
        ax_prev = plt.axes([0.80, 0.02, 0.08, 0.05])
        ax_next = plt.axes([0.89, 0.02, 0.08, 0.05])
        
        self.btn_prev = Button(ax_prev, 'Prev', color='#95A5A6', hovercolor='#7F8C8D')
        self.btn_next = Button(ax_next, 'Next', color='#2ECC71', hovercolor='#27AE60')
        
        self.btn_prev.on_clicked(self.prev_page)
        self.btn_next.on_clicked(self.next_page)
        
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        self.render_page()
        plt.show()

    def render_page(self):
        json_path = self.pages[self.current_idx]
        png_path = json_path.with_suffix('.png')
        
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        # --- LEFT PANEL: IMAGE ---
        if png_path.exists():
            img = plt.imread(str(png_path))
            self.ax_img.clear()
            self.ax_img.imshow(img)
            self.ax_img.axis('off') # Keep axis off for image, it's fine here
        
        # --- RIGHT PANEL: TEXT ---
        self.ax_text.clear()
        
        # 1. Set the "Paper" Look
        self.ax_text.set_facecolor('#ECF0F1') # Light grey paper background
        
        # 2. Hide Borders/Ticks MANUALLY (Do NOT use axis('off'))
        self.ax_text.set_xticks([])
        self.ax_text.set_yticks([])
        for spine in self.ax_text.spines.values():
            spine.set_visible(False)
        
        full_text = "\n".join(data['text'])
        
        # Add a title header to the text box
        self.ax_text.text(0.05, 0.96, data['title'], 
                         transform=self.ax_text.transAxes,
                         fontsize=16, weight='bold', color='#2C3E50',
                         verticalalignment='top')
        
        # Add the body text
        self.ax_text.text(0.05, 0.88, full_text, 
                         transform=self.ax_text.transAxes,
                         fontsize=12, verticalalignment='top',
                         fontfamily='monospace', color='#2C3E50', linespacing=1.4)
        
        # Page indicator
        self.fig.suptitle(f"Page {self.current_idx} / {len(self.pages)-1}", 
                         fontsize=12, color='#BDC3C7', y=0.98)
        
        self.fig.canvas.draw_idle()
        
        self.fig.canvas.draw_idle()

    def next_page(self, event=None):
        if self.current_idx < len(self.pages) - 1:
            self.current_idx += 1
            self.render_page()

    def prev_page(self, event=None):
        if self.current_idx > 0:
            self.current_idx -= 1
            self.render_page()

    def on_key(self, event):
        if event.key == 'right':
            self.next_page()
        elif event.key == 'left':
            self.prev_page()
        elif event.key == 'escape':
            plt.close()