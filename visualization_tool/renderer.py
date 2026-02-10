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
        
        # Define custom sizes for rooms (width, height) to match the floor plan image
        self.location_sizes = {
            'entrance': (2, 3.0),
            'maintenance-tunnel': (9.0, 1.2),  # Long horizontal tunnel
            'hall-a': (3.5, 3.0),
            'hall-b': (3.5, 3.0),
            'cryo-chamber': (3.0, 4.6),        # Tall room on the left
            'anti-vibration-pods-room': (1.5, 3.0),
            'stasis-lab': (6.0, 3.0)           # Large room at bottom
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
            'cryo-chamber': (-4.0, -1.7),        # Left large block
            'maintenance-tunnel': (2.5, 0),   # Center connecting corridor
            'entrance': (-0.75, -2.5),            # Far left entry point
            'anti-vibration-pods-room': (-1.0, 2.5), # Small room above tunnel left
            'hall-a': (1.75, 2.5),             # Above tunnel, middle
            'hall-b': (5.25, 2.5),             # Above tunnel, right
            'stasis-lab': (4.0, -2.5)         # Below tunnel, right side
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
            
            new_state = WorldState(self.problem_data)
            # Copy all attributes using NEW NAMES
            new_state.robot_location = current_state.robot_location
            new_state.artifact_locations = current_state.artifact_locations.copy()
            new_state.pod_locations = current_state.pod_locations.copy() 
            new_state.robot_carrying = current_state.robot_carrying
            new_state.robot_hand_empty = current_state.robot_hand_empty              # Fixed
            new_state.robot_carrying_in_pod = current_state.robot_carrying_in_pod    # Fixed
            new_state.robot_carrying_empty_pods = current_state.robot_carrying_empty_pods # Fixed
            new_state.robot_sealing_mode = current_state.robot_sealing_mode          # Fixed
            new_state.artifact_temperatures = current_state.artifact_temperatures.copy()
            states.append(new_state)
            current_state = new_state
        
        def update(frame):
            ax.clear()
            state = states[frame]
            desc = descriptions[frame]
            
            ax.set_xlim(-8, 9)
            ax.set_ylim(-5, 6)
            ax.set_aspect('equal')
            ax.axis('off')
            
            # Draw Rooms
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
            
            # Draw Connections
            for loc1, loc2 in self.problem_data['connections']:
                if loc1 in self.location_positions and loc2 in self.location_positions:
                    pos1 = self.location_positions[loc1]
                    pos2 = self.location_positions[loc2]
                    ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 'k:', alpha=0.15, linewidth=1)
            
            # --- DRAW PODS ---
            for pod, location in state.pod_locations.items():
                if location in self.location_positions:
                    pos = self.location_positions[location]
                    room_w, room_h = self.location_sizes.get(location, (1.5, 1.5))
                    pod_size = 0.3 
                    
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

                    pod_rect = patches.Rectangle(
                        (draw_x, draw_y), 
                        pod_size, pod_size,
                        linewidth=1.0, edgecolor='#006400', facecolor='#32CD32', 
                        alpha=1.0, zorder=4 
                    )
                    ax.add_patch(pod_rect)
                    ax.text(draw_x + pod_size/2, draw_y - 0.15, pod,
                           ha='center', va='top', fontsize=5, weight='bold', color='#006400')

            # Draw Artifacts
            for artifact, location in state.artifact_locations.items():
                if location in self.location_positions:
                    pos = self.location_positions[location]
                    temp = state.artifact_temperatures.get(artifact, 'warm')
                    color = self.artifact_colors.get(temp, '#888888')
                    
                    offset_x = (hash(artifact) % 10 - 5) * 0.15
                    offset_y = (hash(artifact + "y") % 10 - 5) * 0.15
                    
                    ax.scatter(pos[0] + offset_x, pos[1] + offset_y, 
                              s=150, c=color, marker='o', edgecolors='black', linewidths=1.5, zorder=5)
                    ax.text(pos[0] + offset_x, pos[1] + offset_y, artifact[:2].upper(),
                           ha='center', va='center', fontsize=6, color='white', weight='bold', zorder=6)
            
            # Draw Robot
            if state.robot_location in self.location_positions:
                robot_pos = self.location_positions[state.robot_location]
                draw_x = robot_pos[0]
                draw_y = robot_pos[1] + 0.3
                
                robot_circle = patches.Circle((draw_x, draw_y), 0.35, color='#2E86C1', ec='black', linewidth=2, zorder=10)
                ax.add_patch(robot_circle)
                ax.text(draw_x, draw_y, 'R', ha='center', va='center', fontsize=14, weight='bold', color='white', zorder=11)
                
                # Robot Status Banner (UPDATED NAMES)
                status_text = []
                if state.robot_sealing_mode: status_text.append("[LOCKED]")
                if state.robot_carrying_in_pod: status_text.append(f"Pod: {state.robot_carrying_in_pod}")
                elif state.robot_carrying_empty_pods: status_text.append("Empty Pod")
                elif state.robot_carrying: status_text.append(f"Hold: {state.robot_carrying}")
                
                if status_text:
                    ax.text(draw_x, draw_y + 0.5, "\n".join(status_text), 
                           ha='center', va='bottom', fontsize=8, weight='bold',
                           bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8), zorder=12)
            
            ax.set_title(f'Interplanetary Museum Vault - Step {frame}/{len(states)-1}', fontsize=18, weight='bold', pad=20)
            ax.text(0.5, 0.98, desc, transform=ax.transAxes, ha='center', va='top', fontsize=12, 
                   fontfamily='monospace', weight='bold', bbox=dict(boxstyle='round,pad=0.5', facecolor='#FFF8DC', edgecolor='#DAA520'))
            ax.text(0.5, 0.02, state.get_state_summary(), transform=ax.transAxes, ha='center', va='bottom', fontsize=10, 
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.9))
            
            legend_elements = [
                patches.Patch(color=self.artifact_colors['warm'], label='Warm Artifact'),
                patches.Patch(color=self.artifact_colors['cold'], label='Cold Artifact'),
                patches.Patch(color='#32CD32', label='Pod'),
                patches.Patch(color='#2E86C1', label='Robot')
            ]
            ax.legend(handles=legend_elements, loc='upper left', fontsize=10)
        
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
        print(f"Static visualization saved to: {output_file}")
        plt.close()
        return output_file
    
    def _draw_state(self, ax, state: WorldState, title: str):
        ax.set_facecolor('#FDF5E6')
        ax.set_xlim(-8, 9)
        ax.set_ylim(-5, 6)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title(title, fontsize=16, weight='bold', pad=15)
        
        # Rooms
        for loc, pos in self.location_positions.items():
            color = self.location_colors.get(loc, '#E0E0E0')
            width, height = self.location_sizes.get(loc, (1.5, 1.5))
            rect = patches.Rectangle((pos[0] - width/2, pos[1] - height/2), width, height, linewidth=2, edgecolor='#4A4A4A', facecolor=color, alpha=0.9)
            ax.add_patch(rect)
            label_y_offset = height/2 - 0.4 if height > 2 else 0
            ax.text(pos[0], pos[1] + label_y_offset, loc.replace('-', ' ').title(), ha='center', va='center', fontsize=10, weight='bold', color='#333333')
            
        # Pods
        for pod, location in state.pod_locations.items():
            if location in self.location_positions:
                pos = self.location_positions[location]
                room_w, room_h = self.location_sizes.get(location, (1.5, 1.5))
                pod_size = 0.3
                
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
                
                pod_rect = patches.Rectangle((draw_x, draw_y), pod_size, pod_size, linewidth=1.0, edgecolor='#006400', facecolor='#32CD32', alpha=1.0, zorder=4)
                ax.add_patch(pod_rect)
        
        # Artifacts
        for artifact, location in state.artifact_locations.items():
            if location in self.location_positions:
                pos = self.location_positions[location]
                temp = state.artifact_temperatures.get(artifact, 'warm')
                color = self.artifact_colors.get(temp, '#888888')
                offset_x = (hash(artifact) % 10 - 5) * 0.15
                offset_y = (hash(artifact + "y") % 10 - 5) * 0.15
                ax.scatter(pos[0] + offset_x, pos[1] + offset_y, s=150, c=color, marker='o', edgecolors='black', linewidths=1.5, zorder=5)

        # Robot
        if state.robot_location in self.location_positions:
            robot_pos = self.location_positions[state.robot_location]
            draw_x, draw_y = robot_pos[0], robot_pos[1] + 0.3
            robot_circle = patches.Circle((draw_x, draw_y), 0.35, color='#2E86C1', ec='black', linewidth=2, zorder=10)
            ax.add_patch(robot_circle)
            ax.text(draw_x, draw_y, 'R', ha='center', va='center', fontsize=14, weight='bold', color='white', zorder=11)