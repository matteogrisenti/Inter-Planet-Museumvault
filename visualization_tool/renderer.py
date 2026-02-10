import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter
from typing import Dict, Any, List, Tuple
from world import WorldState

class Visualizer:
    """Creates visual simulation of the planning execution."""
    
    def __init__(self, problem_data: Dict[str, Any], plan: List[Tuple[str, List[str]]]):
        self.problem_data = problem_data
        self.plan = plan
        self.world_state = WorldState(problem_data)
        
        # Room Dimensions
        self.location_sizes = {
            'entrance': (2, 3.0),
            'maintenance-tunnel': (9.0, 1.2),
            'hall-a': (3.5, 3.0),
            'hall-b': (3.5, 3.0),
            'cryo-chamber': (3.0, 4.6),
            'anti-vibration-pods-room': (1.5, 3.0),
            'stasis-lab': (6.0, 3.0)
        }

        self.location_positions = self._compute_layout()
        
        self.location_colors = {
            'entrance': '#D3D3D3',
            'maintenance-tunnel': '#F5DEB3',
            'hall-a': '#FAF0E6',
            'hall-b': '#FAF0E6',
            'cryo-chamber': '#D1F2EB',
            'anti-vibration-pods-room': '#E8DAEF',
            'stasis-lab': '#FAF0E6'
        }
        
        self.artifact_colors = {
            'warm': '#FF6B6B',
            'cold': '#4ECDC4'
        }
        
        # Analyze robots to assign dynamic roles/colors
        self.robot_metadata = self._analyze_robot_roles()
    
    def _analyze_robot_roles(self) -> Dict[str, Dict[str, Any]]:
        """
        Dynamically determines robot roles, colors, and short names based on:
        1. Name heuristics (e.g., 'tech' -> Technician)
        2. Permissions (Capabilities)
        """
        metadata = {}
        
        # Color Palette
        COLOR_ADMIN = '#2E86C1'      # Blue
        COLOR_TECH = '#E67E22'       # Orange
        COLOR_SCI = '#8E44AD'        # Purple
        COLOR_DEFAULT = '#7F8C8D'    # Grey
        
        for r_name in self.world_state.robots:
            perms = self.world_state.permissions.get(r_name, {'pickup': set(), 'access': set()})
            pickup_types = perms.get('pickup', set())
            
            # --- Role Inference ---
            role = 'unknown'
            color = COLOR_DEFAULT
            short = r_name[:3].upper()
            
            name_lower = r_name.lower()
            
            # Heuristic 1: Name-based (Strongest)
            if 'curator' in name_lower or 'admin' in name_lower:
                role = 'admin'
                color = COLOR_ADMIN
                short = "ADM" if 'admin' in name_lower else "CUR"
            elif 'tech' in name_lower:
                role = 'technician'
                color = COLOR_TECH
                short = "TEC"
                # Add number if present
                if name_lower[-1].isdigit():
                     short = f"T{name_lower[-1]}"
            elif 'sci' in name_lower:
                role = 'scientist'
                color = COLOR_SCI
                short = "SCI"
            
            # Heuristic 2: Capability-based (Fallback)
            else:
                if 'top-secret' in pickup_types:
                    role = 'admin'
                    color = COLOR_ADMIN
                    short = "ADM"
                elif 'scientific' in pickup_types:
                    role = 'scientist'
                    color = COLOR_SCI
                    short = "SCI"
                elif 'technological' in pickup_types:
                    role = 'technician'
                    color = COLOR_TECH
                    short = "TEC"
            
            metadata[r_name] = {
                'role': role,
                'color': color,
                'short_name': short
            }
            
        return metadata

    def _compute_layout(self) -> Dict[str, Tuple[float, float]]:
        layout = {
            'cryo-chamber': (-4.0, -1.7),
            'maintenance-tunnel': (2.5, 0),
            'entrance': (-0.75, -2.5),
            'anti-vibration-pods-room': (-1.0, 2.5),
            'hall-a': (1.75, 2.5),
            'hall-b': (5.25, 2.5),
            'stasis-lab': (4.0, -2.5)
        }
        
        positions = {}
        for loc in self.problem_data['locations']:
            if loc in layout:
                positions[loc] = layout[loc]
            else:
                positions[loc] = (len(positions) * 2, 5)
        return positions
    
    def create_animation(self, output_file: str = 'pddl_simulation.gif', 
                        interval: int = 800, max_steps: int = None):
        fig, ax = plt.subplots(figsize=(16, 10))
        fig.patch.set_facecolor('#FDF5E6')
        ax.set_facecolor('#FDF5E6')
        
        steps_to_show = self.plan[:max_steps] if max_steps else self.plan
        
        states = [WorldState(self.problem_data)]
        descriptions = ["Initial State"]
        
        current_state = WorldState(self.problem_data)
        for action_name, parameters in steps_to_show:
            desc = current_state.apply_action(action_name, parameters)
            descriptions.append(desc)
            
            # Deep Copy for Animation State (Manual serialization/deserialization)
            new_state = WorldState(self.problem_data)
            # Restore state from dict to ensure full copy
            state_dict = current_state.to_dict()
            
            # Reconstruct dynamic properties
            new_state.artifact_locations = state_dict['artifact_locations'].copy()
            new_state.pod_locations = state_dict['pod_locations'].copy()
            new_state.pod_contents = state_dict['pod_contents'].copy()
            new_state.artifact_temperatures = state_dict['artifact_temperatures'].copy()
            
            new_state.robots = {
                k: v.copy() for k, v in state_dict['robots'].items()
            }
            new_state.drones = {
                k: v.copy() for k, v in state_dict['drones'].items()
            }
            new_state.permissions = {
                k: {'access': v['access'].copy(), 'pickup': v['pickup'].copy()} 
                for k, v in self.world_state.permissions.items()
            }

            states.append(new_state)
            current_state = new_state
        
        def update(frame):
            ax.clear()
            self._draw_state(ax, states[frame], f'Step {frame}: {descriptions[frame]}')

        print(f"Creating animation with {len(states)} frames...")
        anim = FuncAnimation(fig, update, frames=len(states), interval=interval, repeat=True)
        writer = PillowWriter(fps=1000/interval)
        anim.save(output_file, writer=writer, dpi=100)
        print(f"Animation saved to: {output_file}")
        plt.close()
        return output_file

    def create_static_visualization(self, output_file: str = 'pddl_static.png'):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        fig.patch.set_facecolor('#FDF5E6')
        
        self._draw_state(ax1, self.world_state, "Initial State")
        
        final_state = WorldState(self.problem_data)
        for action_name, parameters in self.plan:
             final_state.apply_action(action_name, parameters)
        
        self._draw_state(ax2, final_state, "Final State")
        
        plt.tight_layout()
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        return output_file
    
    def _draw_state(self, ax, state: WorldState, title: str):
        ax.set_facecolor('#FDF5E6')
        ax.set_xlim(-8, 9)
        ax.set_ylim(-5, 6)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title(title, fontsize=16, weight='bold', pad=15)
        
        # 1. Draw Rooms
        for loc, pos in self.location_positions.items():
            color = self.location_colors.get(loc, '#E0E0E0')
            width, height = self.location_sizes.get(loc, (1.5, 1.5))
            rect = patches.Rectangle(
                (pos[0] - width/2, pos[1] - height/2), width, height,
                linewidth=2, edgecolor='#4A4A4A', facecolor=color, alpha=0.9
            )
            ax.add_patch(rect)
            
            label_y_offset = height/2 - 0.4 if height > 2 else 0
            ax.text(pos[0], pos[1] + label_y_offset, loc.replace('-', ' ').title(), 
                   ha='center', va='center', fontsize=10, weight='bold', color='#333333')
            
            # Draw Permissions Info (Little icons/text in corner of room)
            # This is complex because permissions are by robot. 
            # We will list who can ACCESS this room.
            access_list = []
            for r_name, perms in state.permissions.items():
                if loc in perms['access']:
                    # Use dynamic short name
                    meta = self.robot_metadata.get(r_name, {'short_name': r_name[:3].upper()})
                    access_list.append(meta['short_name'])

            
            if access_list:
                access_str = "Acc: " + ",".join(access_list)
                ax.text(pos[0], pos[1] - height/2 + 0.2, access_str, 
                       ha='center', va='bottom', fontsize=5, color='#555555')


        # 2. Draw Connections
        for loc1, loc2 in self.problem_data['connections']:
            if loc1 in self.location_positions and loc2 in self.location_positions:
                pos1 = self.location_positions[loc1]
                pos2 = self.location_positions[loc2]
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 'k:', alpha=0.15, linewidth=1)

        # 3. Draw Pods
        for pod, location in state.pod_locations.items():
            if location in self.location_positions:
                pos = self.location_positions[location]
                room_w, room_h = self.location_sizes.get(location, (1.5, 1.5))
                pod_size = 0.3
                
                # Custom layout for the pod room
                if location == 'anti-vibration-pods-room':
                    if '1' in pod:
                        draw_x = (pos[0] - room_w/2) + 0.2
                        draw_y = (pos[1] - room_h/2) + 0.2
                    elif '2' in pod:
                        draw_x = (pos[0] + room_w/2) - pod_size - 0.2
                        draw_y = (pos[1] - room_h/2) + 0.2
                    else:
                        draw_x = pos[0] - pod_size/2
                        draw_y = pos[1] - room_h/2 + 0.2
                else:
                    offset_x = (hash(pod) % 5 - 2) * 0.2
                    offset_y = (hash(pod + "y") % 5 - 2) * 0.2
                    draw_x = pos[0] - pod_size/2 + offset_x
                    draw_y = pos[1] - pod_size/2 + offset_y
                
                # Check if pod contains something
                is_full = pod in state.pod_contents
                face_color = '#228B22' if is_full else '#32CD32' # Darker green if full
                
                pod_rect = patches.Rectangle((draw_x, draw_y), pod_size, pod_size, linewidth=1.0, edgecolor='#006400', facecolor=face_color, alpha=1.0, zorder=4)
                ax.add_patch(pod_rect)
                
                if is_full:
                    ax.text(draw_x + pod_size/2, draw_y + pod_size/2, "★", ha='center', va='center', fontsize=6, color='white')

        # 4. Draw Artifacts (On floor only)
        for artifact, location in state.artifact_locations.items():
            if location in self.location_positions:
                pos = self.location_positions[location]
                temp = state.artifact_temperatures.get(artifact, 'warm')
                color = self.artifact_colors.get(temp, '#888888')
                offset_x = (hash(artifact) % 10 - 5) * 0.15
                offset_y = (hash(artifact + "y") % 10 - 5) * 0.15
                ax.scatter(pos[0] + offset_x, pos[1] + offset_y, s=150, c=color, marker='o', edgecolors='black', linewidths=1.5, zorder=5)

        # 5. Draw Robots
        # Sort keys to ensure deterministic drawing order
        for i, r_name in enumerate(sorted(state.robots.keys())):
            r_data = state.robots[r_name]
            loc = r_data['loc']
            
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                
                # Offset multiple robots in same room
                
                offset_x = (i % 3 - 1) * 0.6
                offset_y = 0.3 # Move robot above center
                if i > 2: offset_y += 0.3 # Second row if many robots
                
                draw_x, draw_y = pos[0] + offset_x, pos[1] + offset_y
                
                # Use dynamic color
                meta = self.robot_metadata.get(r_name, {'color': '#7F8C8D', 'short_name': r_name[:3].upper()})
                color = meta['color']

                # --- ROBOT BODY ---
                # Sealing Mode: Red Border
                edge_color = 'red' if r_data['sealing_mode'] else 'black'
                edge_width = 3 if r_data['sealing_mode'] else 2
                
                robot_circle = patches.Circle((draw_x, draw_y), 0.35, color=color, ec=edge_color, linewidth=edge_width, zorder=10)
                ax.add_patch(robot_circle)
                
                # Letter: Use short name (first 2 chars if possible, else 1)
                # T1, T2, AD, CU etc.
                letter = meta['short_name'][:2]
                font_size = 10 if len(letter) > 1 else 12
                ax.text(draw_x, draw_y, letter, ha='center', va='center', fontsize=font_size, weight='bold', color='white', zorder=11)
                
                # --- POD / CARRYING STATUS VISUALIZATION ---
                # Draw small circles next to robot to represent holding items visually
                
                item_x = draw_x + 0.35  # Right of robot
                item_y = draw_y - 0.25  # Lower right
                
                if r_data['carrying_empty_pods']:
                    # Draw Empty Grey Circle (Pod)
                    pod_circle = patches.Circle((item_x, item_y), 0.15, color='#A9A9A9', ec='black', linewidth=1, zorder=12)
                    ax.add_patch(pod_circle)
                
                elif r_data['carrying_in_pod']:
                    # Draw Pod with Artifact Inside
                    # 1. Pod Circle
                    pod_circle = patches.Circle((item_x, item_y), 0.15, color='#A9A9A9', ec='black', linewidth=1, zorder=12)
                    ax.add_patch(pod_circle)
                    
                    # 2. Artifact Inside (Color based on temp if possible, default red)
                    # We need to look up artifact temp, but r_data only has name.
                    # We can try to look it up in state.artifact_temperatures if it was tracked there? 
                    # Actually state.artifact_temperatures usually tracks items on floor.
                    # But we can assume 'warm' (Red) or 'cold' (Blue) based on artifact name/knowledge?
                    # Simplified: Use a generic color or try to find it. 
                    # Let's use a distinct color for the "inner" item.
                    
                    inner_color = '#FFD700' # Gold generic for item inside
                    
                    artifact_circle = patches.Circle((item_x, item_y), 0.08, color=inner_color, ec='black', linewidth=0.5, zorder=13)
                    ax.add_patch(artifact_circle)
                
                elif r_data['carrying']:
                    # Draw Artifact directly
                    item_color = '#FF6B6B' # Default warm red
                    artifact_circle = patches.Circle((item_x, item_y), 0.12, color=item_color, ec='black', linewidth=1, zorder=12)
                    ax.add_patch(artifact_circle)

                # --- ROBOT STATUS TEXT ---
                status_text = []
                status_text.append(f"{r_name.title()}")
                if r_data['sealing_mode']: status_text.append("[LOCK]")
                
                if r_data['carrying_in_pod']: 
                    item_name = r_data['carrying_in_pod']
                    if len(item_name) > 10: item_name = item_name[:8] + ".."
                    status_text.append(f"InPod: {item_name}")
                elif r_data['carrying_empty_pods']: 
                    status_text.append("Pod: Empty")
                elif r_data['carrying']: 
                    item_name = r_data['carrying']
                    if len(item_name) > 10: item_name = item_name[:8] + ".."
                    status_text.append(f"Hold: {item_name}")
                
                full_label = "\n".join(status_text)
                ax.text(draw_x, draw_y + 0.5, full_label, 
                       ha='center', va='bottom', fontsize=6, weight='bold', color='black',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor='#333333'),
                       zorder=20)
        
        # 6. Draw Drones
        for d_name in sorted(state.drones.keys()):
            d_data = state.drones[d_name]
            loc = d_data['loc']
            
            if loc in self.location_positions:
                pos = self.location_positions[loc]
                
                # Drones fly higher/offset
                offset_x = 0
                offset_y = -0.8 
                
                draw_x, draw_y = pos[0] + offset_x, pos[1] + offset_y
                
                # Drone visualization: Purple Triangle
                drone_patch = patches.RegularPolygon((draw_x, draw_y), numVertices=3, radius=0.25, orientation=0, color='#8E44AD', ec='black', zorder=15)
                ax.add_patch(drone_patch)
                
                ax.text(draw_x, draw_y, 'D', ha='center', va='center', fontsize=8, weight='bold', color='white', zorder=16)

                if not d_data['empty']:
                     item = d_data['carrying']
                     if len(item) > 10: item = item[:8] + ".."
                     ax.text(draw_x, draw_y - 0.35, f"Carry: {item}", 
                             ha='center', va='top', fontsize=5, color='purple', weight='bold')