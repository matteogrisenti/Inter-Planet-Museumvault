#!/usr/bin/env python3
"""
Temporal Diagram Renderer
Generates Gantt-style charts showing robot actions over time.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
from typing import Dict, Any, List
from optic_parser import OpticParser


class TemporalDiagram:
    """Renders temporal Gantt charts from OPTIC timeline data."""
    
    # Color scheme for different action categories
    COLORS = {
        'movement': '#3498db',        # Blue
        'sealing': '#e74c3c',         # Red
        'pod_management': '#2ecc71',  # Green
        'artifact_handling': '#f39c12',  # Orange
        'cooling': '#1abc9c',         # Teal/Cyan
        'other': '#95a5a6'            # Gray
    }
    
    def __init__(self, timeline_data: Dict[str, Any]):
        """
        Initialize the temporal diagram renderer.
        
        Args:
            timeline_data: Dictionary with 'robots', 'actions', and 'total_time'
        """
        self.data = timeline_data
        self.robots = timeline_data['robots']
        self.actions = timeline_data['actions']
        self.total_time = timeline_data['total_time']
        
        # Create parser instance for categorization
        self.parser = OpticParser(Path("dummy"))  # Just for categorization method
        
    def render(self, output_path: Path, figsize=None, show_labels=True):
        """
        Render the temporal diagram and save to file.
        
        Args:
            output_path: Path where to save the PNG
            figsize: Figure size (width, height). If None, calculated dynamically.
            show_labels: Whether to show action labels on bars
        """
        if not self.robots:
            print("⚠️ No robots found in timeline data")
            return
        
        # Dynamic Width Calculation based on total time
        # E.g., 1 inch per 5 seconds of timeline, with a minimum of 16 inches
        if figsize is None:
            width = max(16, self.total_time / 4.0)
            figsize = (width, 8) # Increased height slightly
        
        fig, ax = plt.subplots(figsize=figsize)
        
        # Create a mapping from robot name to Y position
        robot_positions = {robot: idx for idx, robot in enumerate(self.robots)}
        
        # Track which categories are used for legend
        used_categories = set()
        
        # Draw each action as a horizontal bar
        for action in self.actions:
            robot = action['robot']
            if not robot or robot not in robot_positions:
                continue
            
            start = action['start']
            duration = action['duration']
            y_pos = robot_positions[robot]
            
            # Categorize and color the action
            category = self.parser.categorize_action(action['action'])
            color = self.COLORS.get(category, self.COLORS['other'])
            used_categories.add(category)
            
            # Draw the bar with better spacing
            bar = ax.barh(
                y_pos, 
                duration, 
                left=start, 
                height=0.7,  # Increased from 0.6 for better visibility
                color=color, 
                edgecolor='black',
                linewidth=0.8,  # Slightly thicker borders
                alpha=0.85  # Slightly more opaque
            )
            
            # Add label with improved threshold
            if show_labels and duration > (self.total_time * 0.005):  # Show if > 0.5% (was 1.5%) - since we are wider now
                # Better action name abbreviation
                action_label = self._format_action_label(action['action'], duration)
                
                # Dynamic font sizing
                # Base size 9, increase with duration
                # Linear scale: approx +1 pt for every 2 seconds
                calc_size = 9 + int(duration * 0.4)
                
                # Reduce size if text is long to prevent wrapping issues/overflow
                if len(action_label) > 10:
                    calc_size = max(9, calc_size - 2)
                    
                # Cap minimum and maximum sizes
                final_size = max(9, min(20, calc_size))

                ax.text(
                    start + duration / 2, 
                    y_pos, 
                    action_label,
                    ha='center', 
                    va='center', 
                    fontsize=final_size,
                    fontweight='bold',
                    color='white',
                    wrap=True, # Enable wrapping
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.3, edgecolor='none')
                )
        
        # Configure axes with larger fonts
        ax.set_yticks(range(len(self.robots)))
        ax.set_yticklabels([robot.capitalize() for robot in self.robots], fontsize=12, fontweight='bold')
        ax.set_xlabel('Time (seconds)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Robots', fontsize=14, fontweight='bold')
        ax.set_title(f'Temporal Action Timeline - OPTIC Planner (Scale: {figsize[0]/self.total_time:.2f} in/s)', fontsize=16, fontweight='bold', pad=15)
        
        # Set X axis limits
        ax.set_xlim(0, self.total_time * 1.02)
        
        # Better grid
        ax.grid(True, axis='x', alpha=0.4, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)  # Grid behind bars
        
        # Tick parameters
        ax.tick_params(axis='both', which='major', labelsize=11)
        
        # Create legend with larger font
        legend_elements = []
        for category in sorted(used_categories):
            color = self.COLORS.get(category, self.COLORS['other'])
            label = category.replace('_', ' ').title()
            legend_elements.append(mpatches.Patch(color=color, label=label, edgecolor='black', linewidth=0.5))
        
        ax.legend(
            handles=legend_elements, 
            loc='upper right',
            fontsize=11,
            framealpha=0.95,
            edgecolor='black',
            fancybox=True,
            shadow=True
        )
        
        # Tight layout
        plt.tight_layout()
        
        # Save figure with higher DPI
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white') # Return to 150 dpi to avoid massive files if width is huge
        plt.close()
        
        print(f"✅ Temporal diagram saved to: {output_path}")
    
    def _format_action_label(self, action_name: str, duration: float) -> str:
        """
        Format action name for display on bars (wrapping and shortening).
        """
        # 1. Clean up common prefixes
        name = action_name.replace('move-to-', 'Move ').replace('activate-', 'Act ')
        name = name.replace('pick-up-', 'Pick ').replace('drop-', 'Drop ')
        name = name.replace('put-in-', 'Put ').replace('release-', 'Rel ')
        name = name.replace('cool-', 'Cool ')
        
        # 2. Replace hyphens with spaces to allow wrapping
        name = name.replace('-', ' ')
        name = name.replace('_', ' ')
        
        # 3. Abbreviate specific terms
        replacements = {
            'pressurized': 'Press',
            'unpressurized': 'Unpress',
            'room': 'Rm',
            'artifact': 'Art',
            'carrying': 'Carry',
            'technician': 'Tech',
            'curator': 'Cur',
            'scientist': 'Sci',
            'empty': 'Emp',
            'full': 'Full',
            'entrance': 'Ent',
            'maintenance': 'Maint',
            'tunnel': 'Tun',
            'chamber': 'Chbr',
            'stasis': 'Stas',
            'anti': '',
            'vibration': 'Vib',
            'pods': 'Pods'
        }
        
        for k, v in replacements.items():
            name = name.replace(k, v)
            
        # 4. Smart Wrapping: Add newlines
        # Split into words
        words = name.split()
        if len(words) >= 2:
            # Group by 2 words or just join with newlines
            if len(words) > 3:
                # e.g., "Move Curator Ent Hall" -> "Move Curator\nEnt Hall"
                mid = len(words) // 2
                name = " ".join(words[:mid]) + "\n" + " ".join(words[mid:])
            else:
                # "Move Hall" -> "Move\nHall"
                name = "\n".join(words)
        
        return name
    
    def print_summary(self):
        """Print a text summary of the timeline."""
        print(f"\n{'='*60}")
        print(f"TEMPORAL TIMELINE SUMMARY".center(60))
        print(f"{'='*60}")
        print(f"Robots: {', '.join(self.robots)}")
        print(f"Total Actions: {len(self.actions)}")
        print(f"Total Duration: {self.total_time:.3f} seconds")
        print(f"\nActions by Robot:")
        
        for robot in self.robots:
            robot_actions = [a for a in self.actions if a['robot'] == robot]
            print(f"  {robot.capitalize()}: {len(robot_actions)} actions")
        
        print(f"\nActions by Category:")
        category_counts = {}
        for action in self.actions:
            cat = self.parser.categorize_action(action['action'])
            category_counts[cat] = category_counts.get(cat, 0) + 1
        
        for cat, count in sorted(category_counts.items()):
            print(f"  {cat.replace('_', ' ').title()}: {count} actions")
        
        print(f"{'='*60}\n")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python temporal_diagram.py <output.txt> [output_image.png]")
        sys.exit(1)
    
    # Parse the OPTIC output
    parser = OpticParser(Path(sys.argv[1]))
    timeline_data = parser.parse()
    
    # Generate diagram
    diagram = TemporalDiagram(timeline_data)
    diagram.print_summary()
    
    output_file = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("temporal_diagram.png")
    diagram.render(output_file)
