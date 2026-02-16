import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter
import json
import re
from pathlib import Path
from typing import Dict, Any, List, Tuple

class StateParser:
    """
    Parses a list of PDDL atoms (strings or lists) into a structured dictionary
    representing the visual state of the world (robots, items, locations).
    """
    @staticmethod
    def parse(atoms: List[List[str]]) -> Dict[str, Any]:
        state = {
            'robots': {},
            'drones': {},
            'artifacts': {},
            'pods': {},
            'locations': set(),
            'permissions': {}, # access/pickup
            'seismic': []
        }
        
        # Helper to ensure robot dict exists
        def get_robot(name):
            if name not in state['robots']:
                state['robots'][name] = {
                    'loc': None, 'holding': None, 'holding_pod': None,
                    'pod_content': None, 'sealed': False, 'type': 'unknown'
                }
            return state['robots'][name]

        # Helper to ensure drone dict exists
        def get_drone(name):
            if name not in state['drones']:
                state['drones'][name] = {'loc': None, 'holding': None}
            return state['drones'][name]

        for atom in atoms:
            # Atom format: ["predicate", "arg1", "arg2", ...]
            pred = atom[0]
            args = atom[1:]

            # --- ROBOTS ---
            if pred == 'robot-at':
                get_robot(args[0])['loc'] = args[1]
                state['locations'].add(args[1])
            elif pred == 'sealing-mode':
                get_robot(args[0])['sealed'] = True
            elif pred == 'carrying':
                get_robot(args[0])['holding'] = args[1]
            elif pred == 'carrying-empty-pod':
                r = get_robot(args[0])
                r['holding_pod'] = args[1]
                r['pod_content'] = None
            elif pred == 'carrying-full-pod':
                r = get_robot(args[0])
                r['holding_pod'] = args[1]
                # We often need another atom to know WHAT is in the pod, 
                # but sometimes the plan implies it. We'll check 'pod-contains' later.
            
            # --- DRONES ---
            elif pred == 'drone-at':
                get_drone(args[0])['loc'] = args[1]
            elif pred == 'drone-carrying':
                get_drone(args[0])['holding'] = args[1]

            # --- ARTIFACTS ---
            elif pred == 'artifact-at':
                state['artifacts'][args[0]] = {'loc': args[1], 'temp': 'warm'} # default temp
            elif pred == 'cold':
                if args[0] in state['artifacts']:
                    state['artifacts'][args[0]]['temp'] = 'cold'
            
            # --- PODS (In Rooms) ---
            # CASE 1: Domain specific 'contains-empty-pod ?loc ?pod'
            elif pred == 'contains-empty-pod':
                state['pods'][args[1]] = {'loc': args[0], 'content': None}
            
            # CASE 2: Domain specific 'contains-full-pod ?loc ?pod'
            elif pred == 'contains-full-pod':
                state['pods'][args[1]] = {'loc': args[0], 'content': 'unknown'}
            
            # CASE 3: Standard 'pod-at ?pod ?loc' (The missing feature)
            elif pred == 'pod-at':
                # args[0] = pod, args[1] = location
                if args[0] not in state['pods']:
                    state['pods'][args[0]] = {'loc': args[1], 'content': None}
                else:
                    state['pods'][args[0]]['loc'] = args[1]

            elif pred == 'pod-contains':
                # Can apply to pods on floor or in hand
                pod_id = args[0]
                item_id = args[1]
                
                # Check if pod is on floor
                if pod_id in state['pods']:
                    state['pods'][pod_id]['content'] = item_id
                
                # Check if pod is in hand (robot)
                for r in state['robots'].values():
                    if r['holding_pod'] == pod_id:
                        r['pod_content'] = item_id

            # --- ENVIRONMENT ---
            elif pred == 'is-seismic':
                state['seismic'].append(args[0])
            elif pred == 'can-access':
                r, l = args
                if r not in state['permissions']: state['permissions'][r] = []
                state['permissions'][r].append(l)

        return state

class Renderer:
    """Visualizes the PDDL trace using Matplotlib."""
    
    def __init__(self, trace_file: Path):
        with open(trace_file, 'r') as f:
            self.history = json.load(f)
        
        # --- CONFIGURATION ---
        self.location_sizes = {
            'entrance': (2, 3.0),
            'maintenance-tunnel': (9.0, 1.2),
            'hall-a': (3.5, 3.0),
            'hall-b': (3.5, 3.0),
            'cryo-chamber': (3.0, 4.6),
            'anti-vibration-pods-room': (1.5, 3.0),
            'stasis-lab': (6.0, 3.0)
        }
        
        self.location_positions = {
            'cryo-chamber': (-4.0, -1.7),
            'maintenance-tunnel': (2.5, 0),
            'entrance': (-0.75, -2.5),
            'anti-vibration-pods-room': (-1.0, 2.5),
            'hall-a': (1.75, 2.5),
            'hall-b': (5.25, 2.5),
            'stasis-lab': (4.0, -2.5)
        }
        
        self.location_colors = {
            'entrance': '#D3D3D3',
            'maintenance-tunnel': '#F5DEB3',
            'hall-a': '#FAF0E6',
            'hall-b': '#FAF0E6',
            'cryo-chamber': '#D1F2EB',
            'anti-vibration-pods-room': '#E8DAEF',
            'stasis-lab': '#FAF0E6'
        }

        # Analyze roles based on the initial state (Step 0)
        self.robot_metadata = self._analyze_robot_roles(self.history[0]['state'])

    def _analyze_robot_roles(self, initial_atoms) -> Dict[str, Dict]:
        # Parse atoms if they are strings
        parsed_atoms = []
        for atom in initial_atoms:
            if isinstance(atom, str):
                clean = atom.replace('(', '').replace(')', '')
                parsed_atoms.append(clean.split())
            else:
                parsed_atoms.append(atom)
                
        state = StateParser.parse(parsed_atoms)
        metadata = {}
        
        COLOR_ADMIN = '#2E86C1'      # Blue
        COLOR_TECH = '#E67E22'       # Orange
        COLOR_SCI = '#8E44AD'        # Purple
        COLOR_DEFAULT = '#7F8C8D'    # Grey
        
        for r_name in state['robots']:
            name_lower = r_name.lower()
            if 'curator' in name_lower or 'admin' in name_lower:
                role = 'admin'
                color = COLOR_ADMIN
                short = "ADM" if 'admin' in name_lower else "CUR"
            elif 'tech' in name_lower:
                role = 'technician'
                color = COLOR_TECH
                short = f"T{name_lower[-1]}" if name_lower[-1].isdigit() else "TEC"
            elif 'sci' in name_lower:
                role = 'scientist'
                color = COLOR_SCI
                short = "SCI"
            else:
                role = 'unknown'
                color = COLOR_DEFAULT
                short = r_name[:3].upper()
            
            metadata[r_name] = {'color': color, 'short': short, 'role': role}
        return metadata

    def render_frame(self, ax, step_index):
        """Draws a single state onto the provided Axes."""
        step_data = self.history[step_index]
        title = f"Step {step_data['step']}: {step_data['action'].upper()}"
        if step_data.get('error'):
             title += f" ({step_data['error']})"
        self.render_state(ax, step_data['state'], title=title)

    def render_state(self, ax, state_atoms, title=None):
        """Draws a state from a list of atoms/predicates."""
        # Convert state_atoms to list format if necessary
        # The parser expects atoms like ["robot-at", "r1", "l1"]
        # If input is a set or list of strings, convert them
        parsed_atoms = []
        for atom in state_atoms:
            if isinstance(atom, str):
                # Simple parsing for string atoms like "(robot-at r1 l1)"
                clean = atom.replace('(', '').replace(')', '')
                parsed_atoms.append(clean.split())
            else:
                parsed_atoms.append(atom)
                
        state = StateParser.parse(parsed_atoms)
        
        ax.clear()
        ax.set_facecolor('#FDF5E6')
        ax.set_xlim(-8, 9)
        ax.set_ylim(-5, 6)
        ax.set_aspect('equal')
        ax.axis('off')
        
        if title:
            ax.set_title(title, fontsize=14, weight='bold', pad=10)

        # 1. DRAW ROOMS
        for loc, pos in self.location_positions.items():
            color = self.location_colors.get(loc, '#E0E0E0')
            width, height = self.location_sizes.get(loc, (1.5, 1.5))
            
            # Seismic Warning (Red Tint)
            if loc in state['seismic']:
                color = '#FFCCCB' # Light red
            
            rect = patches.Rectangle(
                (pos[0] - width/2, pos[1] - height/2), width, height,
                linewidth=2, edgecolor='#4A4A4A', facecolor=color, alpha=0.9, zorder=1
            )
            ax.add_patch(rect)
            
            # Room Label
            label_y = pos[1] + height/2 - 0.3 if height > 2 else pos[1]
            ax.text(pos[0], label_y, loc.replace('-', ' ').title(), 
                   ha='center', va='center', fontsize=9, weight='bold', color='#333333', zorder=2)

        # 2. DRAW PODS (On Floor)
        for pod_id, pod_data in state['pods'].items():
            loc = pod_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                
                # Jitter based on name hash to avoid overlap
                ox = (hash(pod_id) % 5 - 2) * 0.2
                oy = (hash(pod_id+"y") % 5 - 2) * 0.2
                
                # Special layout for Pod Room
                if loc == 'anti-vibration-pods-room':
                    ox = 0
                    oy = 0.5 if '1' in pod_id else -0.5
                
                draw_x, draw_y = pos[0] + ox - 0.15, pos[1] + oy - 0.15
                
                color = '#228B22' if pod_data['content'] else '#90EE90'
                rect = patches.Rectangle((draw_x, draw_y), 0.3, 0.3, 
                                       facecolor=color, edgecolor='darkgreen', zorder=3)
                ax.add_patch(rect)
                if pod_data['content']:
                     ax.text(draw_x+0.15, draw_y+0.15, "★", ha='center', va='center', color='white', fontsize=6, zorder=4)

        # 3. DRAW ARTIFACTS (On Floor)
        for art_id, art_data in state['artifacts'].items():
            loc = art_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                ox = (hash(art_id) % 8 - 4) * 0.15
                oy = (hash(art_id+"x") % 8 - 4) * 0.15
                
                color = '#4ECDC4' if art_data.get('temp') == 'cold' else '#FF6B6B'
                ax.scatter(pos[0] + ox, pos[1] + oy, s=100, c=color, edgecolors='black', zorder=4)

        # 4. DRAW ROBOTS
        for i, (r_name, r_data) in enumerate(sorted(state['robots'].items())):
            loc = r_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                
                # Smart offset for multiple robots
                ox = (i % 3 - 1) * 0.6
                oy = 0.0
                draw_x, draw_y = pos[0] + ox, pos[1] + oy
                
                meta = self.robot_metadata.get(r_name, {'color': '#999', 'short': '??'})
                
                # Body
                edge = 'red' if r_data['sealed'] else 'black'
                width = 3 if r_data['sealed'] else 1.5
                circle = patches.Circle((draw_x, draw_y), 0.35, 
                                      facecolor=meta['color'], edgecolor=edge, linewidth=width, zorder=10)
                ax.add_patch(circle)
                
                # Label
                ax.text(draw_x, draw_y, meta['short'], ha='center', va='center', 
                       color='white', weight='bold', fontsize=8, zorder=11)
                
                # Holding Visualization
                hold_x, hold_y = draw_x + 0.3, draw_y - 0.3
                
                if r_data['holding_pod']:
                    # Draw Pod in hand
                    p_color = '#228B22' if r_data['pod_content'] else '#A9A9A9'
                    p_rect = patches.Rectangle((hold_x, hold_y), 0.25, 0.25, 
                                             facecolor=p_color, edgecolor='black', zorder=12)
                    ax.add_patch(p_rect)
                    if r_data['pod_content']:
                         ax.text(hold_x+0.125, hold_y+0.125, "★", ha='center', va='center', color='white', fontsize=5, zorder=13)
                
                elif r_data['holding']:
                    # Draw Artifact in hand
                    ax.scatter(hold_x+0.1, hold_y+0.1, s=60, c='#FFD700', edgecolors='black', zorder=12)

        # 5. DRAW DRONES
        for d_name, d_data in state['drones'].items():
            loc = d_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                draw_x, draw_y = pos[0], pos[1] - 0.8
                
                poly = patches.RegularPolygon(xy=(draw_x, draw_y), numVertices=3, radius=0.25, color='#8E44AD', zorder=15)
                ax.add_patch(poly)
                ax.text(draw_x, draw_y, 'D', ha='center', va='center', color='white', fontsize=7, zorder=16)
                
                if d_data['holding']:
                     ax.text(draw_x, draw_y-0.3, "Item", ha='center', fontsize=6, color='purple')

    def save_gif(self, output_path: Path, max_frames=None, interval=800):
        """Generates the GIF animation."""
        fig, ax = plt.subplots(figsize=(16, 10))
        fig.patch.set_facecolor('#FDF5E6')
        
        frames = self.history
        if max_frames:
            frames = frames[:max_frames]
            
        print(f"Rendering {len(frames)} frames to GIF...")
        
        def update(frame_idx):
            self.render_frame(ax, frame_idx)
            
        anim = FuncAnimation(fig, update, frames=len(frames), interval=interval)
        writer = PillowWriter(fps=1000/interval)
        
        anim.save(output_path, writer=writer, dpi=100)
        plt.close()
        print(f"✅ GIF saved to {output_path}")

    def save_static(self, output_path: Path):
        """Saves a comparison of Start vs End."""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        fig.patch.set_facecolor('#FDF5E6')
        
        self.render_frame(ax1, 0)
        ax1.set_title("Initial State", fontsize=16, weight='bold')
        
        self.render_frame(ax2, -1)
        ax2.set_title("Final State", fontsize=16, weight='bold')
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"✅ Static image saved to {output_path}")

if __name__ == "__main__":
    import sys
    try:
        trace_path = Path("viz/trace.json") 
        if len(sys.argv) > 1:
            trace_path = Path(sys.argv[1])
            
        renderer = Renderer(trace_path)
        renderer.save_static(trace_path.parent / "summary.png")
    except FileNotFoundError:
        print("trace.json not found. Run main.py first.")