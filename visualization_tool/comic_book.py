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
        
    def _get_agent_status(self, state: WorldState) -> Dict[str, str]:
        """Helper to extract rich status info from the world state."""
        # 1. Determine Hand Status (UPDATED VARIABLE NAMES)
        if state.robot_hand_empty:
            hand = "Empty"
        elif state.robot_carrying_empty_pods:
            hand = "Holding Empty Pod"
        elif state.robot_carrying_in_pod:
            hand = f"Carrying {state.robot_carrying_in_pod} (in Pod)"
        elif state.robot_carrying:
            hand = f"Carrying {state.robot_carrying}"
        else:
            hand = "Unknown"
            
        # 2. Determine Seal Status (UPDATED VARIABLE NAMES)
        seal = "Sealed (Pressurized)" if state.robot_sealing_mode else "Unsealed (Normal)"
        
        return {
            "loc": state.robot_location,
            "hand": hand,
            "seal": seal
        }

    def generate(self):
        """Generates the comic book pages with strict segmentation for Transit vs Delivery."""
        print(f"🎨 Generating Comic Book in: {self.output_dir}")
        
        sim = WorldState(self.problem_data)
        
        # --- PROLOGUE ---
        start_status = self._get_agent_status(sim)
        self._save_page(
            page_num=0,
            title="Prologue: The Vault Opens",
            state=sim,
            text_lines=[
                "INITIAL STATUS:",
                f"  • Location:   {start_status['loc']}",
                f"  • Inventory:  {start_status['hand']}",
                f"  • Artifacts:  {len(sim.artifact_locations)} items pending",
                "",
                "PLAN OVERVIEW:",
                f"  • Total Steps: {len(self.plan)}",
                "  • Mission Start..."
            ]
        )
        
        episode_count = 1
        current_batch_actions = []
        batch_type = "TRANSIT" 
        
        # Capture state BEFORE the batch begins
        batch_start_info = self._get_agent_status(sim)
        
        # EXPANDED KEYWORDS FOR ROBUST DETECTION
        PICKUP_KEYWORDS = ['pick-up', 'put-in-pod', 'secure']
        DROP_KEYWORDS = ['drop', 'release', 'unload']

        for action_name, params in self.plan:
            # Check for matches
            is_pickup = any(k in action_name for k in PICKUP_KEYWORDS)
            is_drop = any(k in action_name for k in DROP_KEYWORDS)
            
            # --- CHECK FOR MODE SWITCHING (Transit -> Delivery) ---
            if is_pickup and batch_type == "TRANSIT":
                if current_batch_actions:
                    self._save_episode(episode_count, "TRANSIT", batch_start_info, current_batch_actions, sim)
                    episode_count += 1
                
                batch_type = "DELIVERY"
                current_batch_actions = []
                batch_start_info = self._get_agent_status(sim)
            
            # --- EXECUTE ACTION ---
            desc = sim.apply_action(action_name, params)
            current_batch_actions.append(desc)
            
            # --- CHECK FOR MODE SWITCHING (Delivery -> Transit) ---
            if is_drop and batch_type == "DELIVERY":
                self._save_episode(episode_count, "DELIVERY", batch_start_info, current_batch_actions, sim)
                episode_count += 1
                
                batch_type = "TRANSIT"
                current_batch_actions = []
                batch_start_info = self._get_agent_status(sim)

        # --- EPILOGUE (Leftover actions) ---
        if current_batch_actions:
            self._save_episode(episode_count, "EPILOGUE", batch_start_info, current_batch_actions, sim)
            
        print(f"✨ Comic Book Generation Complete! ({episode_count} pages)")

    def _save_episode(self, episode_num, batch_type, start_info, actions, final_state):
        """Formats the detailed text log and saves the page."""
        
        final_info = self._get_agent_status(final_state)
        
        if batch_type == "TRANSIT":
            title = f"Episode {episode_num}: Repositioning"
            context_header = "TRANSIT LOG:"
        elif batch_type == "DELIVERY":
            title = f"Episode {episode_num}: Delivery Mission"
            context_header = "DELIVERY LOG:"
        else:
            title = f"Episode {episode_num}: Final Operations"
            context_header = "ACTION LOG:"

        lines = [
            f"{batch_type} EPISODE",
            "=" * 35,
            "",
            "INITIAL STATE:",
            f"  • Location:   {start_info['loc']}",
            f"  • Inventory:  {start_info['hand']}",
            f"  • Seal Mode:  {start_info['seal']}",
            "",
            context_header,
            *[f"  {i+1}. {act}" for i, act in enumerate(actions)],
            "",
            "TERMINAL STATE:",
            f"  • Location:   {final_info['loc']}",
            f"  • Inventory:  {final_info['hand']}",
            f"  • Seal Mode:  {final_info['seal']}"
        ]
        
        self._save_page(episode_num, title, final_state, lines)

    def _save_page(self, page_num: int, title: str, state: WorldState, text_lines: List[str]):
        filename_base = f"page_{page_num:03d}"
        img_path = self.output_dir / f"{filename_base}.png"
        
        # High-res figure for the map image
        fig, ax = plt.subplots(figsize=(12, 10))
        # Use the visualizer's draw state method
        self.visualizer._draw_state(ax, state, title)
        plt.savefig(img_path, bbox_inches='tight', dpi=100)
        plt.close(fig)
        
        # Save Text Data
        json_path = self.output_dir / f"{filename_base}.json"
        page_data = {"title": title, "page_num": page_num, "text": text_lines}
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
        self.fig = plt.figure(figsize=(18, 9)) 
        self.fig.canvas.manager.set_window_title("PDDL Comic Book Viewer")
        self.fig.patch.set_facecolor('#2C3E50')
        
        plt.subplots_adjust(left=0.02, right=0.98, top=0.92, bottom=0.1)
        
        gs = self.fig.add_gridspec(1, 2, width_ratios=[1, 1])
        self.ax_img = self.fig.add_subplot(gs[0, 0]) 
        self.ax_text = self.fig.add_subplot(gs[0, 1]) 
        
        # Navigation Buttons
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
        
        # Image Panel
        if png_path.exists():
            img = plt.imread(str(png_path))
            self.ax_img.clear()
            self.ax_img.imshow(img)
            self.ax_img.axis('off') 
        
        # Text Panel
        self.ax_text.clear()
        self.ax_text.set_facecolor('#ECF0F1')
        self.ax_text.set_xticks([])
        self.ax_text.set_yticks([])
        for spine in self.ax_text.spines.values():
            spine.set_visible(False)
        
        full_text = "\n".join(data['text'])
        
        self.ax_text.text(0.02, 0.96, data['title'], 
                         transform=self.ax_text.transAxes,
                         fontsize=16, weight='bold', color='#2C3E50',
                         verticalalignment='top')
        
        self.ax_text.text(0.02, 0.90, full_text, 
                         transform=self.ax_text.transAxes,
                         fontsize=10, 
                         linespacing=1.25,
                         verticalalignment='top',
                         fontfamily='monospace', 
                         color='#2C3E50',
                         wrap=True)
        
        self.fig.suptitle(f"Page {self.current_idx} / {len(self.pages)-1}", 
                         fontsize=12, color='#BDC3C7', y=0.98)
        
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