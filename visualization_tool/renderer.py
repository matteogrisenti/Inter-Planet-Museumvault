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
    ENHANCED: Now supports slot-based inventory and artifact metadata
    """
    @staticmethod
    def parse(atoms: List[List[str]], artifact_metadata: Dict = None) -> Dict[str, Any]:
        state = {
            'robots': {},
            'drones': {},
            'artifacts': {},
            'pods': {},
            'locations': set(),
            'permissions': {},
            'seismic': []
        }
        
        # Helper to ensure robot dict exists
        def get_robot(name):
            if name not in state['robots']:
                state['robots'][name] = {
                    'loc': None,
                    'holding': None,
                    'holding_pod': None,
                    'pod_content': None,
                    'sealed': False,
                    'type': 'unknown',
                    # NEW: Slot-based inventory
                    'slot_1': None,
                    'slot_2': None,
                    'slot_1_type': None,
                    'slot_2_type': None
                }
            return state['robots'][name]

        # Helper to ensure drone dict exists
        def get_drone(name):
            if name not in state['drones']:
                state['drones'][name] = {'loc': None, 'holding': None}
            return state['drones'][name]

        for atom in atoms:
            pred = atom[0]
            args = atom[1:]

            # --- ROBOTS ---
            if pred in ('robot-at', 'robot_at'):
                get_robot(args[0])['loc'] = args[1]
                state['locations'].add(args[1])
            elif pred in ('sealing-mode', 'sealing_mode_on'):
                get_robot(args[0])['sealed'] = True
            elif pred == 'sealing_mode_off':
                get_robot(args[0])['sealed'] = False
            
            # OLD DOMAIN predicates
            elif pred == 'carrying':
                r = get_robot(args[0])
                r['holding'] = args[1]
            elif pred == 'carrying-empty-pod':
                r = get_robot(args[0])
                r['holding_pod'] = args[1]
                r['pod_content'] = None
            elif pred == 'carrying-full-pod':
                r = get_robot(args[0])
                r['holding_pod'] = args[1]
            elif pred == 'carrying-second-object':
                r = get_robot(args[0])
                r['slot_2'] = args[1]
                r['slot_2_type'] = 'artifact'
            
            # NEW DOMAIN predicates - Slot-based
            elif pred in ('carrying-slot-1', 'carrying_slot_1'):
                r = get_robot(args[0])
                r['slot_1'] = args[1]
                r['slot_1_type'] = 'artifact'
            elif pred in ('carrying-slot-2', 'carrying_slot_2'):
                r = get_robot(args[0])
                r['slot_2'] = args[1]
                r['slot_2_type'] = 'artifact'
            elif pred in ('carrying-pod-slot-1', 'carrying_pod_slot_1'):
                r = get_robot(args[0])
                r['slot_1'] = args[1]
                r['slot_1_type'] = 'pod'
            elif pred in ('hands-empty-slot-1', 'hands-empty-slot-2', 'hands_empty_slot_1', 'hands_empty_slot_2', 'hands-empty', 'hands_empty'):
                # Explicitly ignoring hands empty since it just means slots are unoccupied
                pass
            
            # --- DRONES ---
            elif pred in ('drone-at', 'drone_at'):
                get_drone(args[0])['loc'] = args[1]
            elif pred in ('drone-carrying', 'drone_carrying'):
                get_drone(args[0])['holding'] = args[1]

            # --- ARTIFACTS ---
            elif pred in ('artifact-at', 'artifact_at'):
                art_type = None
                art_color = '#FF6B6B'  # Default warm red
                
                # Get type and color from metadata if available
                if artifact_metadata and args[0] in artifact_metadata:
                    meta = artifact_metadata[args[0]]
                    art_type = meta.get('type')
                    art_color = meta.get('display_color', art_color)
                
                state['artifacts'][args[0]] = {
                    'loc': args[1],
                    'temp': 'warm',  # default
                    'type': art_type,
                    'color': art_color
                }
            elif pred in ('cold', ):
                if args[0] in state['artifacts']:
                    state['artifacts'][args[0]]['temp'] = 'cold'
                    # Update color if we have metadata
                    if artifact_metadata and args[0] in artifact_metadata:
                        state['artifacts'][args[0]]['color'] = artifact_metadata[args[0]].get('display_color')
            elif pred in ('warm', ):
                if args[0] in state['artifacts']:
                    state['artifacts'][args[0]]['temp'] = 'warm'
                    if artifact_metadata and args[0] in artifact_metadata:
                        state['artifacts'][args[0]]['color'] = artifact_metadata[args[0]].get('display_color')
            
            # --- PODS (In Rooms) ---
            elif pred in ('contains-empty-pod', 'contains_empty_pod'):
                state['pods'][args[1]] = {'loc': args[0], 'content': None}
            elif pred in ('contains-full-pod', 'contains_full_pod'):
                state['pods'][args[1]] = {'loc': args[0], 'content': 'unknown'}
            elif pred in ('pod-at', 'pod_at'):
                if args[0] not in state['pods']:
                    state['pods'][args[0]] = {'loc': args[1], 'content': None}
                else:
                    state['pods'][args[0]]['loc'] = args[1]
            elif pred in ('pod-contains', 'pod_contains'):
                pod_id = args[0]
                item_id = args[1]
                
                # Check if pod is on floor
                if pod_id in state['pods']:
                    state['pods'][pod_id]['content'] = item_id
                
                # Check if pod is in hand (robot) - OLD DOMAIN
                for r in state['robots'].values():
                    if r['holding_pod'] == pod_id:
                        r['pod_content'] = item_id
                    # NEW DOMAIN
                    if r['slot_1'] == pod_id and r['slot_1_type'] == 'pod':
                        r['pod_content'] = item_id

            # --- ENVIRONMENT ---
            elif pred in ('is-seismic', 'is_seismic'):
                state['seismic'].append(args[0])
            elif pred in ('can-access', 'can_access'):
                r, l = args
                if r not in state['permissions']:
                    state['permissions'][r] = []
                state['permissions'][r].append(l)

        return state

class Renderer:
    """Visualizes the PDDL trace using Matplotlib with enhanced slot and color support."""
    
    def __init__(self, trace_file: Path):
        with open(trace_file, 'r') as f:
            self.history = json.load(f)
        
        # --- CONFIGURATION ---
        # Keys stored in BOTH hyphenated (2.2) and underscore (2.4) styles
        self.location_sizes = {
            'entrance': (2, 3.0),
            'maintenance-tunnel': (9.0, 1.2),  'maintenance_tunnel': (9.0, 1.2),
            'hall-a': (3.5, 3.0),              'hall_a': (3.5, 3.0),
            'hall-b': (3.5, 3.0),              'hall_b': (3.5, 3.0),
            'cryo-chamber': (3.0, 4.6),        'cryo_chamber': (3.0, 4.6),
            'anti-vibration-pods-room': (1.5, 3.0), 'anti_vibration_pods_room': (1.5, 3.0),
            'stasis-lab': (6.0, 3.0),          'stasis_lab': (6.0, 3.0)
        }
        
        self.location_positions = {
            'cryo-chamber': (-4.0, -1.7),      'cryo_chamber': (-4.0, -1.7),
            'maintenance-tunnel': (2.5, 0),    'maintenance_tunnel': (2.5, 0),
            'entrance': (-0.75, -2.5),
            'anti-vibration-pods-room': (-1.0, 2.5), 'anti_vibration_pods_room': (-1.0, 2.5),
            'hall-a': (1.75, 2.5),             'hall_a': (1.75, 2.5),
            'hall-b': (5.25, 2.5),             'hall_b': (5.25, 2.5),
            'stasis-lab': (4.0, -2.5),         'stasis_lab': (4.0, -2.5)
        }
        
        self.location_colors = {
            'entrance': '#D3D3D3',
            'maintenance-tunnel': '#F5DEB3',   'maintenance_tunnel': '#F5DEB3',
            'hall-a': '#FAF0E6',               'hall_a': '#FAF0E6',
            'hall-b': '#FAF0E6',               'hall_b': '#FAF0E6',
            'cryo-chamber': '#D1F2EB',         'cryo_chamber': '#D1F2EB',
            'anti-vibration-pods-room': '#E8DAEF', 'anti_vibration_pods_room': '#E8DAEF',
            'stasis-lab': '#FAF0E6',           'stasis_lab': '#FAF0E6'
        }

        # Analyze roles based on the initial state
        self.robot_metadata = self._analyze_robot_roles(self.history[0]['state'])

    def _analyze_robot_roles(self, initial_atoms) -> Dict[str, Dict]:
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
        COLOR_DRN = '#27AE60'        # Green
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
            elif 'drone' in name_lower:
                role = 'drone'
                color = COLOR_DRN
                short = "DRN"
            else:
                role = 'unknown'
                color = COLOR_DEFAULT
                short = r_name[:3].upper()
            
            metadata[r_name] = {'role': role, 'color': color, 'short': short}
        
        return metadata

    def render_frame(self, ax, step_index):
        """Draws a single state onto the provided Axes."""
        step_data = self.history[step_index]
        title = f"Step {step_data['step']}: {step_data['action'].upper()}"
        if step_data.get('error'):
            title += f" ({step_data['error']})"
        
        # Get artifact metadata if available (from enhanced parser)
        artifact_metadata = step_data.get('artifact_metadata', {})
        
        self.render_state(ax, step_data['state'], title=title, artifact_metadata=artifact_metadata)

    def render_state(self, ax, state_atoms, title=None, artifact_metadata=None):
        """Draws a state from a list of atoms/predicates."""
        parsed_atoms = []
        for atom in state_atoms:
            if isinstance(atom, str):
                clean = atom.replace('(', '').replace(')', '')
                parsed_atoms.append(clean.split())
            else:
                parsed_atoms.append(atom)
                
        state = StateParser.parse(parsed_atoms, artifact_metadata)
        
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
            
            if loc in state['seismic']:
                color = '#FFCCCB'
            
            rect = patches.Rectangle(
                (pos[0] - width/2, pos[1] - height/2), width, height,
                linewidth=2, edgecolor='#4A4A4A', facecolor=color, alpha=0.9, zorder=1
            )
            ax.add_patch(rect)
            
            label_y = pos[1] + height/2 - 0.3 if height > 2 else pos[1]
            ax.text(pos[0], label_y, loc.replace('-', ' ').title(), 
                   ha='center', va='center', fontsize=9, weight='bold', color='#333333', zorder=2)

        # 2. DRAW PODS (On Floor)
        for pod_id, pod_data in state['pods'].items():
            loc = pod_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                
                ox = (hash(pod_id) % 5 - 2) * 0.2
                oy = (hash(pod_id+"y") % 5 - 2) * 0.2
                
                if loc == 'anti-vibration-pods-room':
                    ox = 0
                    oy = 0.5 if '1' in pod_id else -0.5
                
                draw_x, draw_y = pos[0] + ox - 0.15, pos[1] + oy - 0.15
                
                color = '#228B22' if pod_data['content'] else '#90EE90'
                rect = patches.Rectangle((draw_x, draw_y), 0.3, 0.3, 
                                       facecolor=color, edgecolor='darkgreen', zorder=3)
                ax.add_patch(rect)
                if pod_data['content']:
                    ax.text(draw_x+0.15, draw_y+0.15, "★", ha='center', va='center', 
                           color='white', fontsize=6, zorder=4)

        # 3. DRAW ARTIFACTS (On Floor) - WITH COLOR CODING
        for art_id, art_data in state['artifacts'].items():
            loc = art_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                ox = (hash(art_id) % 8 - 4) * 0.15
                oy = (hash(art_id+"x") % 8 - 4) * 0.15
                
                # Use the color from metadata (includes type and temperature)
                color = art_data.get('color', '#FF6B6B')
                
                ax.scatter(pos[0] + ox, pos[1] + oy, s=100, c=color, edgecolors='black', zorder=4)
                
                # Optional: Add type indicator
                if art_data.get('type'):
                    type_initial = art_data['type'][0].upper()
                    ax.text(pos[0] + ox, pos[1] + oy, type_initial, 
                           ha='center', va='center', fontsize=6, weight='bold', 
                           color='white', zorder=5)

        # 4. DRAW ROBOTS - WITH SLOT VISUALIZATION
        for i, (r_name, r_data) in enumerate(sorted(state['robots'].items())):
            loc = r_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                
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
                
                # SLOT VISUALIZATION - Support both old and new domains
                self._draw_robot_inventory(ax, draw_x, draw_y, r_data, artifact_metadata)

        # 5. DRAW DRONES
        for d_name, d_data in state['drones'].items():
            loc = d_data['loc']
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                draw_x, draw_y = pos[0], pos[1] - 0.8
                
                poly = patches.RegularPolygon(xy=(draw_x, draw_y), numVertices=3, 
                                            radius=0.25, color='#8E44AD', zorder=15)
                ax.add_patch(poly)
                ax.text(draw_x, draw_y, 'D', ha='center', va='center', 
                       color='white', fontsize=7, zorder=16)
                
                if d_data['holding']:
                    ax.text(draw_x, draw_y-0.3, "Item", ha='center', fontsize=6, color='purple')

    def _draw_robot_inventory(self, ax, robot_x, robot_y, robot_data, artifact_metadata):
        """Draw what the robot is carrying - supports both old and new domain formats"""
        
        # Position for inventory display
        inv_x = robot_x + 0.5
        inv_y = robot_y
        
        slots_to_draw = []
        
        # NEW DOMAIN - Slot-based
        if robot_data['slot_1'] is not None:
            slots_to_draw.append(('Slot1', robot_data['slot_1'], robot_data['slot_1_type'], robot_data.get('pod_content')))
        if robot_data['slot_2'] is not None:
            slots_to_draw.append(('Slot2', robot_data['slot_2'], robot_data['slot_2_type'], None))
        
        # OLD DOMAIN - Legacy support
        if robot_data['holding_pod'] is not None and not slots_to_draw:
            slots_to_draw.append(('Pod', robot_data['holding_pod'], 'pod', robot_data.get('pod_content')))
        elif robot_data['holding'] is not None and not slots_to_draw:
            slots_to_draw.append(('Hand', robot_data['holding'], 'artifact', None))
        
        # Draw each slot
        for idx, (slot_name, item_id, item_type, pod_content) in enumerate(slots_to_draw):
            offset_y = -0.3 * idx
            draw_y = inv_y + offset_y
            
            if item_type == 'pod':
                # Draw pod
                p_color = '#228B22' if pod_content else '#A9A9A9'
                p_rect = patches.Rectangle((inv_x, draw_y - 0.125), 0.25, 0.25, 
                                         facecolor=p_color, edgecolor='black', zorder=12)
                ax.add_patch(p_rect)
                if pod_content:
                    ax.text(inv_x + 0.125, draw_y, "★", ha='center', va='center', 
                           color='white', fontsize=5, zorder=13)
            elif item_type == 'artifact':
                # Draw artifact with color coding
                art_color = '#FFD700'  # Default gold
                
                # Get color from metadata if available
                if artifact_metadata and item_id in artifact_metadata:
                    art_color = artifact_metadata[item_id].get('display_color', art_color)
                
                ax.scatter(inv_x + 0.125, draw_y, s=60, c=art_color, 
                          edgecolors='black', zorder=12)

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
        renderer.save_gif(trace_path.parent / "animation.gif")
    except FileNotFoundError:
        print("trace.json not found. Run parser first.")