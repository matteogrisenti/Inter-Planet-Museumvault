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
            
            # Deep Copy for Animation State
            new_state = WorldState(self.problem_data)
            new_state.robot_location = current_state.robot_location
            new_state.artifact_locations = current_state.artifact_locations.copy()
            new_state.pod_locations = current_state.pod_locations.copy() 
            new_state.pod_contents = current_state.pod_contents.copy() # Copy Pod Contents
            new_state.robot_carrying = current_state.robot_carrying
            new_state.robot_hand_empty = current_state.robot_hand_empty
            new_state.robot_carrying_in_pod = current_state.robot_carrying_in_pod
            new_state.robot_carrying_empty_pods = current_state.robot_carrying_empty_pods
            new_state.robot_sealing_mode = current_state.robot_sealing_mode
            new_state.artifact_temperatures = current_state.artifact_temperatures.copy()
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
                
                # Custom layout for the pod room to make them visible
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
                
                # Draw indicator if full
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

        # 5. Draw Robot
        if state.robot_location in self.location_positions:
            robot_pos = self.location_positions[state.robot_location]
            draw_x, draw_y = robot_pos[0], robot_pos[1] + 0.3
            robot_circle = patches.Circle((draw_x, draw_y), 0.35, color='#2E86C1', ec='black', linewidth=2, zorder=10)
            ax.add_patch(robot_circle)
            ax.text(draw_x, draw_y, 'R', ha='center', va='center', fontsize=14, weight='bold', color='white', zorder=11)
            
            # --- UPDATED ROBOT STATUS LABEL ---
            status_text = []
            if state.robot_sealing_mode: status_text.append("[LOCK]")
            
            # Simplify text to save space, but keep it clear
            if state.robot_carrying_in_pod: 
                # Split long names
                item_name = state.robot_carrying_in_pod
                if len(item_name) > 15:
                    item_name = item_name[:12] + "..."
                status_text.append(f"In-Pod:\n{item_name}")
            elif state.robot_carrying_empty_pods: 
                status_text.append("Pod: Empty")
            elif state.robot_carrying: 
                item_name = state.robot_carrying
                if len(item_name) > 15:
                    item_name = item_name[:12] + "..."
                status_text.append(f"Hold:\n{item_name}")
            
            if status_text:
                full_label = "\n".join(status_text)
                # Removed 'bbox' constraint that might cut text, added zorder=20 to stay on top
                ax.text(draw_x, draw_y + 0.6, full_label, 
                       ha='center', va='bottom', fontsize=7, weight='bold', color='black',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9, edgecolor='#333333'),
                       zorder=20)