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
from typing import Dict, Any, List, Tuple
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
        
        # Parse earthquake windows if problem file is provided
        self.earthquake_windows = self._parse_earthquake_windows() if problem_pddl_path else []
        
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
    
    def _parse_earthquake_windows(self) -> List[Tuple[float, float]]:
        """
        Parse earthquake windows from problem.pddl TIL (Timed Initial Literals).
        Reuses the same logic as TemporalDiagram.
        
        Returns:
            List of (start_time, end_time) tuples for earthquake periods
        """
        if not self.problem_pddl_path or not self.problem_pddl_path.exists():
            return []
        
        try:
            with open(self.problem_pddl_path, 'r') as f:
                content = f.read()
            
            # Remove comments
            content = re.sub(r';.*', '', content)
            
            # Find all TIL events for is-safe hall-b
            unsafe_pattern = r'\(at\s+(\d+(?:\.\d+)?)\s+\(not\s+\(is-safe\s+hall-b\)\)\)'
            safe_pattern = r'\(at\s+(\d+(?:\.\d+)?)\s+\(is-safe\s+hall-b\)\)'
            
            unsafe_times = [float(m.group(1)) for m in re.finditer(unsafe_pattern, content)]
            safe_times = [float(m.group(1)) for m in re.finditer(safe_pattern, content)]
            
            # Pair up unsafe/safe times to create windows
            windows = []
            unsafe_times.sort()
            safe_times.sort()
            
            for unsafe_time in unsafe_times:
                # Find the next safe time after this unsafe time
                matching_safe = [t for t in safe_times if t > unsafe_time]
                if matching_safe:
                    windows.append((unsafe_time, matching_safe[0]))
            
            return windows
            
        except Exception as e:
            print(f"⚠️ Warning: Could not parse earthquake windows: {e}")
            return []
    
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
        Handles both hyphenated (2.2) and underscore (2.4) predicate naming conventions.
        """
        act_name = action['action']
        params = action['params']

        # Detect naming convention from current state
        # Use underscore style if that's what's in the initial atoms
        use_underscore = any('robot_at' in a for a in current_atoms)
        sep = '_' if use_underscore else '-'

        def pred(name):
            """Return predicate with correct naming convention separator."""
            # name given in hyphenated form, convert if needed
            return name.replace('-', sep)

        # Helper to remove an atom matching pattern (handles both sep styles)
        def remove_atom(predicate, *args):
            # Build patterns for BOTH hyphen and underscore variants
            for p_name in {predicate, predicate.replace('-', '_'), predicate.replace('_', '-')}:
                pattern = f"\\({re.escape(p_name)}"
                for arg in args:
                    pattern += f"\\s+{re.escape(arg)}"
                pattern += "\\)"
                regex = re.compile(pattern, re.IGNORECASE)
                current_atoms[:] = [a for a in current_atoms if not regex.search(a)]

        # Helper to add an atom (use same style as existing state)
        def add_atom(predicate, *args):
            atom = f"({pred(predicate)} {' '.join(args)})"
            if atom not in current_atoms:
                current_atoms.append(atom)

        # --- ACTION MAPPING LOGIC ---
        # Normalize action name for matching (convert underscores to hyphens for keywords)
        act_norm = act_name.replace('_', '-')

        # 1. MOVEMENT
        if 'move-to' in act_norm or 'fly-into' in act_norm:
            # params: robot, from, to
            if len(params) >= 3:
                r, loc_from, loc_to = params[0], params[1], params[2]

                # fly_into_seismic_room: ?r ?to ?from  (reversed!)
                if 'fly-into-seismic-room' in act_norm:
                    loc_to, loc_from = params[1], params[2]

                remove_atom('robot-at', r, loc_from)
                add_atom('robot-at', r, loc_to)

                # Moving to pressurized -> sealing off
                if 'unpressurized' not in act_norm:
                    remove_atom('sealing-mode-on', r)
                    add_atom('sealing-mode-off', r)

        # 2. PICK UP (slot-based, 2.4 style)
        elif 'pick-up-empty-pod-slot' in act_norm or 'pick-up-empty-pod' in act_norm:
            # params: robot, loc, pod
            if len(params) >= 3:
                r, l, p = params[0], params[1], params[2]
                remove_atom('pod-at', p, l)
                remove_atom('hands-empty-slot-1', r)
                add_atom('carrying-pod-slot-1', r, p)

        elif 'pick-up-full-pod-slot' in act_norm or 'pick-up-full-pod' in act_norm:
            # params: robot, loc, pod, artifact, type
            if len(params) >= 3:
                r, l, p = params[0], params[1], params[2]
                remove_atom('pod-at', p, l)
                remove_atom('hands-empty-slot-1', r)
                add_atom('carrying-pod-slot-1', r, p)

        elif 'pick-up-slot-1' in act_norm:
            # 2.4: ?a ?at ?l ?r
            if len(params) >= 4:
                a, at, l, r = params[0], params[1], params[2], params[3]
                remove_atom('artifact-at', a, l)
                remove_atom('hands-empty-slot-1', r)
                add_atom('carrying-slot-1', r, a)

        elif 'pick-up-slot-2' in act_norm:
            # 2.4: ?r ?a ?at ?l
            if len(params) >= 4:
                r, a, at, l = params[0], params[1], params[2], params[3]
                remove_atom('artifact-at', a, l)
                remove_atom('hands-empty-slot-2', r)
                add_atom('carrying-slot-2', r, a)

        elif 'pick-up' in act_norm:
            # Legacy: params artifact, type, loc, robot
            if len(params) >= 4:
                a, at, l, r = params[0], params[1], params[2], params[3]
                remove_atom('artifact-at', a, l)
                add_atom('carrying-slot-1', r, a)

        # 3. DROP POD
        elif 'drop-pod-slot' in act_norm or 'drop-empty-pod' in act_norm or 'drop-full-pod' in act_norm:
            # params: robot, pod, loc
            if len(params) >= 3:
                r, p, l = params[0], params[1], params[2]
                remove_atom('carrying-pod-slot-1', r, p)
                add_atom('hands-empty-slot-1', r)
                add_atom('pod-at', p, l)

        # 4. RELEASE ARTIFACT FROM POD
        elif 'release-artifact-from-pod-slot' in act_norm or 'release-artifact-from-pod' in act_norm:
            # params: robot, artifact, loc, pod
            if len(params) >= 4:
                r, a, l, p = params[0], params[1], params[2], params[3]
                remove_atom('pod-contains', p, a)
                remove_atom('pod-empty', p)
                add_atom('artifact-at', a, l)
                add_atom('pod-empty', p)

        # 5. RELEASE ARTIFACT (from hand)
        elif 'release-artifact-slot-1' in act_norm:
            if len(params) >= 3:
                r, a, l = params[0], params[1], params[2]
                remove_atom('carrying-slot-1', r, a)
                add_atom('hands-empty-slot-1', r)
                add_atom('artifact-at', a, l)

        elif 'release-artifact-slot-2' in act_norm:
            if len(params) >= 3:
                r, a, l = params[0], params[1], params[2]
                remove_atom('carrying-slot-2', r, a)
                add_atom('hands-empty-slot-2', r)
                add_atom('artifact-at', a, l)

        elif 'release-artifact' in act_norm:
            if len(params) >= 3:
                r, a, l = params[0], params[1], params[2]
                remove_atom('carrying-slot-1', r, a)
                add_atom('artifact-at', a, l)

        # 6. PUT IN POD
        elif 'put-in-pod-slot' in act_norm or 'put-in-pod' in act_norm:
            # params: artifact, type, loc, robot, pod
            if len(params) >= 5:
                a, at, l, r, p = params[0], params[1], params[2], params[3], params[4]
                remove_atom('artifact-at', a, l)
                remove_atom('pod-empty', p)
                add_atom('pod-contains', p, a)

        # 7. SEALING
        elif 'activate-seal' in act_norm:
            r = params[0]
            remove_atom('sealing-mode-off', r)
            add_atom('sealing-mode-on', r)

        # 8. COOLING (no visual effect on map needed)
        elif 'cool-artifact' in act_norm:
            for p_val in params:
                if any(f'warm {p_val}' in atom or f'warm_{p_val}' in atom for atom in current_atoms):
                    remove_atom('warm', p_val)
                    add_atom('cold', p_val)

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
                '(robot-at curator entrance)', '(robot-at technician entrance)', '(robot-at scientist entrance)', '(robot-at drone entrance)',
                '(hands-empty curator)', '(hands-empty technician)', '(hands-empty scientist)', '(hands-empty drone)',
                '(can-access curator entrance)', '(can-access technician entrance)', '(can-access scientist entrance)', '(can-access drone entrance)'
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
            
            # Print state debug
            # if frame_idx % 10 == 0:
            #    print(f"Frame {frame_idx} (Time {t}): {temp_atoms}")
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
                'scientist': '#8E44AD',
                'drone': '#27AE60'
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
        
        # Draw earthquake indicators FIRST (so they appear behind action bars)
        for start, end in self.earthquake_windows:
            ax.axvspan(start, end, color='#ff6b6b', alpha=0.15, zorder=0, label='_nolegend_')
            # Add earthquake label at the top
            mid_time = (start + end) / 2
            ax.text(mid_time, len(self.robots) - 0.5, 'EARTHQUAKE', 
                   ha='center', va='center', fontsize=8, fontweight='bold',
                   color='red', alpha=0.6, rotation=0,
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='white', 
                            edgecolor='red', linewidth=1.0, alpha=0.8))
        
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
