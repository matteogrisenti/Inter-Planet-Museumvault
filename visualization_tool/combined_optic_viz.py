#!/usr/bin/env python3
"""
Combined OPTIC Visualization
Generates animated GIF showing map with robot movements + temporal diagram below.
"""

import re
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
import json
from pathlib import Path
from typing import Dict, Any, List
from optic_parser import OpticParser
from temporal_diagram import TemporalDiagram


class CombinedOpticVisualization:
    """Creates combined visualization: animated map + temporal diagram."""
    
    def __init__(self, timeline_data: Dict[str, Any], problem_pddl_path: Path = None):
        """
        Initialize combined visualization.
        
        Args:
            timeline_data: Parsed OPTIC timeline data
            problem_pddl_path: Optional path to problem.pddl for map layout
        """
        self.timeline_data = timeline_data
        self.actions = timeline_data['actions']
        self.robots = timeline_data['robots']
        self.total_time = timeline_data['total_time']
        self.problem_pddl_path = problem_pddl_path
        
        # Parse action categories
        self.parser = OpticParser(Path("dummy"))
        
        # Get room layout from problem file if available
        self.locations = self._extract_locations()
        
    def _extract_locations(self) -> Dict[str, tuple]:
        """
        Extract location information from problem.pddl.
        Returns a simple layout for visualization.
        """
        # Default layout for museum vault
        default_layout = {
            'entrance': (1, 3),
            'maintenance-tunnel': (3, 3),
            'hall-a': (5, 4),
            'hall-b': (5, 2),
            'cryo-chamber': (7, 3),
            'anti-vibration-pods-room': (5, 0),
            'stasis-lab': (3, 1)
        }
        
        # TODO: Parse from problem.pddl if available
        return default_layout
    
    def _get_robot_state_at_time(self, current_time: float) -> Dict[str, Any]:
        """
        Get robot positions and actions at a specific time.
        
        Args:
            current_time: Time in seconds
            
        Returns:
            Dictionary with robot states
        """
        robot_states = {}
        
        for robot in self.robots:
            robot_states[robot] = {
                'location': 'entrance',  # Default
                'current_action': None,
                'carrying': None
            }
        
        # Find which actions are active at current_time
        for action in self.actions:
            if action['start'] <= current_time <= action['end'] and action['robot']:
                robot = action['robot']
                if robot in robot_states:
                    robot_states[robot]['current_action'] = action['action']
                    
                    # Try to extract location from movement actions
                    if 'move' in action['action']:
                        params = action['params']
                        if len(params) >= 3:
                            # move-to-*-room robot from to
                            robot_states[robot]['location'] = params[-1]  # destination
        
        return robot_states
    

    
    
    def _parse_initial_state(self) -> List[str]:
        """Parse initial state from problem.pddl using standard parser."""
        if not self.problem_pddl_path or not self.problem_pddl_path.exists():
            print("⚠️ No problem.pddl found, cannot determine initial state correctly.")
            return []
            
        try:
            # Simple manual parsing of :init section
            # Pass 1: Read content
            with open(self.problem_pddl_path, 'r') as f:
                content = f.read()
            
            # Remove comments
            content = re.sub(r';.*', '', content)
            
            # Finding :init section
            # This is a bit hacky but avoids importing the full PDDL parser if not needed
            # or we could try to import ProblemParser
            
            # Let's try to find atoms in (:init ... ) block
            init_match = re.search(r'\(:init\s+(.*?)\s*\(:goal', content, re.DOTALL)
            if not init_match:
                # Try finding up to end of file if no goal (unlikely)
                init_match = re.search(r'\(:init\s+(.*)\)', content, re.DOTALL)
                
            if init_match:
                init_block = init_match.group(1)
                # Normalize whitespace
                init_block = re.sub(r'\s+', ' ', init_block)
                
                # Extract atoms: (predicate arg1 arg2)
                atoms = re.findall(r'\([^\)]+\)', init_block)
                return atoms
                
        except Exception as e:
            print(f"⚠️ Error parsing initial state: {e}")
            
        return []

    def _apply_action_effects(self, current_atoms: List[str], action: Dict[str, Any]):
        """
        Apply effects of a temporal action to the state atoms.
        Mapping logic derived from domain analysis (2.2 vs 2.4).
        """
        act_name = action['action']
        params = action['params']
        robot = action['robot']
        
        # Helper to remove an atom matching pattern
        def remove_atom(predicate, *args):
            # Construct regex for the atom (ignoring whitespace)
            # e.g. \(robot-at\s+curator\s+entrance\)
            pattern = f"\\({predicate}"
            for arg in args:
                pattern += f"\\s+{arg}"
            pattern += "\\)"
            
            regex = re.compile(pattern, re.IGNORECASE)
            # Filter in place
            current_atoms[:] = [a for a in current_atoms if not regex.search(a)]

        # Helper to add an atom
        def add_atom(predicate, *args):
            atom = f"({predicate} {' '.join(args)})"
            if atom not in current_atoms:
                current_atoms.append(atom)
        
        # --- ACTION MAPPING LOGIC ---
        
        # 1. MOVEMENT
        if 'move-to' in act_name:
            # params: robot, from, to
            if len(params) >= 3:
                r, loc_from, loc_to = params[0], params[1], params[2]
                remove_atom('robot-at', r, loc_from)
                add_atom('robot-at', r, loc_to)
                
                # Side effect: Move turns off sealing if entering pressurized
                if 'unpressurized' in act_name:
                    # Enters tunnel -> requires sealing (handled by activate-seal usually)
                    pass
                else:
                    # Enters pressurized -> sealing off
                    remove_atom('sealing-mode', r)
                    remove_atom('sealing-mode-on', r) # 2.4 variant
        
        # 2. PICK UP
        elif 'pick-up-empty-pod' in act_name:
            # params: robot, loc, pod
            if len(params) >= 3:
                r, l, p = params[0], params[1], params[2]
                remove_atom('contains-empty-pod', l, p)
                remove_atom('hands-empty', r)
                add_atom('carrying-empty-pod', r, p)
                
        elif 'pick-up-full-pod' in act_name:
            # params: robot, loc, pod, artifact, type
            if len(params) >= 4:
                r, l, p, a = params[0], params[1], params[2], params[3]
                remove_atom('contains-full-pod', l, p)
                remove_atom('hands-empty', r)
                add_atom('carrying-full-pod', r, p)
                # Implicit: pod still contains artifact (pod-contains p a)
                # We make sure it's in the state just in case
                add_atom('pod-contains', p, a)

        elif 'pick-up' in act_name: # standard artifact pickup
            # params: artifact, type, loc, robot (order varies? check 2.4)
            # 2.4: ?a - artifact ?at - artifact-type ?l - location ?r - robot
            if len(params) >= 4:
                a, at, l, r = params[0], params[1], params[2], params[3]
                remove_atom('artifact-at', a, l)
                remove_atom('hands-empty', r)
                add_atom('carrying', r, a)

        # 3. DROP / RELEASE
        elif 'drop-empty-pod' in act_name:
             # params: robot, pod, loc
             if len(params) >= 3:
                 r, p, l = params[0], params[1], params[2]
                 remove_atom('carrying-empty-pod', r, p)
                 add_atom('hands-empty', r)
                 add_atom('contains-empty-pod', l, p)
        
        elif 'drop-full-pod' in act_name:
             # params: robot, pod, loc
             if len(params) >= 3:
                 r, p, l = params[0], params[1], params[2]
                 remove_atom('carrying-full-pod', r, p)
                 add_atom('hands-empty', r)
                 add_atom('contains-full-pod', l, p)

        elif 'release-artifact-from-pod' in act_name:
            # params: ?r - robot ?a - artifact ?l - location ?p - pod
            if len(params) >= 4:
                r, a, l, p = params[0], params[1], params[2], params[3]
                remove_atom('carrying-full-pod', r, p)
                remove_atom('pod-contains', p, a)
                add_atom('artifact-at', a, l)
                add_atom('pod-empty', p)
                add_atom('carrying-empty-pod', r, p)

        elif 'release-artifact' in act_name: # standard release
            # params: robot, artifact, loc
            if len(params) >= 3:
                r, a, l = params[0], params[1], params[2]
                remove_atom('carrying', r, a)
                add_atom('hands-empty', r)
                add_atom('artifact-at', a, l)

        # 4. LOAD INTO POD
        elif 'put-in-pod' in act_name:
            # params: ?a - artifact ?at - artifact-type ?l - location ?r - robot ?p - pod
            if len(params) >= 5:
                a, at, l, r, p = params[0], params[1], params[2], params[3], params[4]
                remove_atom('artifact-at', a, l)
                remove_atom('carrying-empty-pod', r, p)
                remove_atom('pod-empty', p)
                add_atom('carrying-full-pod', r, p)
                add_atom('pod-contains', p, a)

        # 5. COOLING
        elif 'cool-artifact' in act_name:
            # params: robot, artifact, loc...
            # effect: not warm, cold
            if len(params) >= 2:
                # Find artifact in params
                # Usually 2nd param based on actions
                 pass # Renderer handles color via 'cold' predicate if present
                 # We need to find which param is artifact.
                 # Heuristic: find param starting with 'rock' or 'alien' or known artifact
                 # simpler: if it has 'warm' atom, replace with 'cold'
                 pass 
                 # We'll rely on the fact that we don't strictly simulate temp for map, 
                 # but for color it might matters.
                 # Let's try to update state
                 for p_val in params:
                     # Check if p_val is an artifact
                     if any(f"(warm {p_val})" in atom for atom in current_atoms):
                         remove_atom('warm', p_val)
                         add_atom('cold', p_val)

        # 6. SEALING
        elif 'activate-seal' in act_name:
            r = params[0]
            add_atom('sealing-mode', r) # 2.2 name
            add_atom('sealing-mode-on', r) # 2.4 name

    def render_combined_gif(self, output_path: Path, fps=2, max_frames=None, trace_file: Path = None):
        """
        Render combined GIF with map animation and temporal diagram.
        Also attempts to save as MP4 if ffmpeg is available.
        """
        # ... (setup code same as before until loop) ...
        # Calculate time steps (one frame per second by default)
        time_step = 1.0  # 1 second per frame
        num_frames = int(self.total_time / time_step) + 1
        
        if max_frames and num_frames > max_frames:
            time_step = self.total_time / max_frames
            num_frames = max_frames
        
        print(f"Generating combined animation: {num_frames} frames over {self.total_time:.1f} seconds...")
        
        # Parse initial state first
        initial_atoms = self._parse_initial_state()
        print(f"Initial atoms parsed: {len(initial_atoms)}")
        
        if not initial_atoms:
            # Fallback for when we can't parse PDDL
            print("⚠️ Using fallback initial state")
            initial_atoms = [
                '(robot-at curator entrance)', '(robot-at technician entrance)', '(robot-at scientist entrance)',
                '(hands-empty curator)', '(hands-empty technician)', '(hands-empty scientist)',
                '(can-access curator entrance)', '(can-access technician entrance)', '(can-access scientist entrance)'
            ]

        # Initialize Renderer with dummy trace containing initial state
        from renderer import Renderer
        
        # Create a dummy trace with the initial state so Renderer can analyze roles
        dummy_trace = Path("trace.json")
        dummy_data = [{
            "step": 0,
            "action": "initial-state",
            "state": initial_atoms, # StateParser handles list of strings
            "error": None
        }]
        
        with open(dummy_trace, 'w') as f:
            json.dump(dummy_data, f)
        
        # Now init renderer
        renderer = Renderer(dummy_trace)
        
        # Simulation State
        current_atoms = list(initial_atoms)
        processed_actions = set() # Track by index in self.actions
        
        # Sort actions by END time for sequential updates
        # But we iterate by time steps.
        
        # Create figure (Reduced height to remove excess vertical space)
        fig = plt.figure(figsize=(16, 20))
        
        # Manually adjust margins to be tight
        plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05, hspace=0.0)
        
        # Ratios: Map (4.5), Actions (2.0), Timeline (1.0)
        gs = fig.add_gridspec(3, 1, height_ratios=[4.5, 2.0, 1.0], hspace=0.0)
        ax_map = fig.add_subplot(gs[0])
        ax_actions = fig.add_subplot(gs[1])
        ax_timeline = fig.add_subplot(gs[2])
        
        # Pre-calculate state at each time-step to avoid accumulative errors during skipping
        # (For simpler implementation, we'll just simulate incrementally in update)
        
        # Reset state for animation
        sim_state = {
            'atoms': list(initial_atoms),
            'last_processed_action_indices': set()
        }
        
        # We need to restart simulation for the animation loop
        # So we'll clone the state inside update? No, update is called sequentially.
        # FuncAnimation calls update(0), update(1), ...
        
        # Optimization: Pre-compute states for all frames
        frame_states = []
        temp_atoms = list(initial_atoms)
        temp_processed = set()
        
        # We need to sort actions by end time to apply effects correctly
        # actions is already sorted by start time usually.
        
        for frame_idx in range(num_frames):
            t = frame_idx * time_step
            
            # Apply all actions that ENDED before or at t
            # and haven't been processed yet
            for i, action in enumerate(self.actions):
                if i not in temp_processed and action['end'] <= t:
                    self._apply_action_effects(temp_atoms, action)
                    temp_processed.add(i)
            
            frame_states.append(list(temp_atoms))

        def update(frame_idx):
            current_time = frame_idx * time_step
            
            # Clear axes
            ax_map.clear()
            ax_actions.clear()
            ax_timeline.clear()
            
            # --- TOP: Render Map (Authentic Renderer) ---
            # Use the pre-computed state for this frame
            atoms = frame_states[frame_idx]
            renderer.render_state(ax_map, atoms, title=None)
            ax_map.set_title("Museum Vault State", fontsize=16, fontweight='bold')
            
            # --- MIDDLE: Show Current Actions ---
            self._render_actions_panel(ax_actions, current_time)
            
            # --- BOTTOM: Render Timeline ---
            self._render_timeline_frame(ax_timeline, current_time)
            
            # Overall title
            fig.suptitle(
                f'OPTIC Planner Visualization - Time: {current_time:.1f}s / {self.total_time:.1f}s', 
                fontsize=20, 
                fontweight='bold'
            )
        
        # Create animation
        anim = FuncAnimation(fig, update, frames=num_frames, repeat=True)
        
        # Save as GIF
        print(f"Saving GIF to {output_path}...")
        writer = PillowWriter(fps=fps)
        anim.save(output_path, writer=writer)
        print(f"✅ GIF saved to: {output_path}")

        # Save as MP4
        mp4_path = output_path.with_suffix('.mp4')
        print(f"Attempting to save MP4 to {mp4_path}...")
        try:
            writer_mp4 = FFMpegWriter(fps=fps, metadata=dict(artist='OpticViz'), bitrate=1800)
            anim.save(mp4_path, writer=writer_mp4)
            print(f"✅ MP4 saved to: {mp4_path}")
        except Exception as e:
            print(f"⚠️ Could not save MP4 (ffmpeg might be missing): {e}")

        plt.close()
    
    def _render_actions_panel(self, ax, current_time: float):
        """Render panel showing currently executing actions."""
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis('off')
        
        # Get currently active actions
        active_actions = []
        for action in self.actions:
            if action['start'] <= current_time <= action['end'] and action['robot']:
                active_actions.append(action)
        
        # Display active actions
        if active_actions:
            title_text = "CURRENT ACTIONS:"
            ax.text(0.5, 0.96, title_text, ha='center', va='top', 
                   fontsize=18, fontweight='bold', color='#2C3E50')
            
            # Robot colors matching the renderer
            robot_colors = {
                'curator': '#2E86C1',
                'technician': '#E67E22',
                'scientist': '#8E44AD'
            }
            
            y_pos = 0.65  # Start lower to create padding below title
            
            # Use smaller font or fewer items if needed, but we have more space now
            for i, action in enumerate(active_actions[:8]): 
                robot = action['robot']
                action_name = action['full_text']
                color = robot_colors.get(robot, '#7F8C8D')
                
                clean_action = action_name.replace('(', '').replace(')', '')
                display_text = f"[{robot.upper()}] {clean_action}"
                
                ax.text(0.02, y_pos, display_text, ha='left', va='center',
                       fontsize=14, fontweight='bold', color=color, 
                       bbox=dict(boxstyle='round,pad=0.8', facecolor='white', 
                                edgecolor=color, linewidth=2, alpha=0.9))
                
                y_pos -= 0.15 # Reduced relative spacing as panel is taller
        else:
            # No active actions
            ax.text(0.5, 0.5, "No active actions", ha='center', va='center',
                   fontsize=16, style='italic', color='gray')

    
    def _render_timeline_frame(self, ax, current_time: float):
        """Render the temporal diagram with progress indicator."""
        # Similar to TemporalDiagram.render but with progress marker
        robot_positions = {robot: idx for idx, robot in enumerate(self.robots)}
        used_categories = set()
        
        # Color scheme
        COLORS = {
            'movement': '#3498db',
            'sealing': '#e74c3c',
            'pod_management': '#2ecc71',
            'artifact_handling': '#f39c12',
            'cooling': '#1abc9c',
            'other': '#95a5a6'
        }
        
        # Draw action bars
        for action in self.actions:
            robot = action['robot']
            if not robot or robot not in robot_positions:
                continue
            
            start = action['start']
            duration = action['duration']
            y_pos = robot_positions[robot]
            
            category = self.parser.categorize_action(action['action'])
            color = COLORS.get(category, COLORS['other'])
            used_categories.add(category)
            
            # Highlight if currently active
            alpha = 1.0 if (start <= current_time <= action['end']) else 0.5
            linewidth = 2.0 if (start <= current_time <= action['end']) else 0.8
            
            ax.barh(
                y_pos, 
                duration, 
                left=start, 
                height=0.7,
                color=color, 
                edgecolor='black',
                linewidth=linewidth,
                alpha=alpha
            )
        
        # Draw vertical line for current time
        ax.axvline(x=current_time, color='red', linewidth=3, linestyle='--', label='Current Time', zorder=20)
        
        # Configure axes
        ax.set_yticks(range(len(self.robots)))
        ax.set_yticklabels([robot.capitalize() for robot in self.robots], fontsize=10, fontweight='bold')
        ax.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
        ax.set_ylabel('Robots', fontsize=11, fontweight='bold')
        ax.set_title('Action Timeline', fontsize=12, fontweight='bold')
        ax.set_xlim(0, self.total_time * 1.05)
        ax.grid(True, axis='x', alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        
        # Legend
        legend_elements = [Line2D([0], [0], color='red', linewidth=3, linestyle='--', label='Current Time')]
        # Add category colors (simplified for animation)
        ax.legend(handles=legend_elements, loc='upper right', fontsize=8)


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python combined_optic_viz.py <output.txt> [output.gif] [problem.pddl]")
        sys.exit(1)
    
    # Parse OPTIC output
    parser = OpticParser(Path(sys.argv[1]))
    timeline_data = parser.parse()
    
    # Optional: problem.pddl for better map layout
    problem_pddl = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    
    # Generate combined visualization
    viz = CombinedOpticVisualization(timeline_data, problem_pddl)
    
    output_file = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("combined_animation.gif")
    viz.render_combined_gif(output_file, fps=2, max_frames=100)  # Limit to 100 frames for testing
