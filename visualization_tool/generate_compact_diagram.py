#!/usr/bin/env python3
"""
Generate a compact temporal diagram focused on the last 120 seconds.
Perfect for reports and presentations.
"""
from pathlib import Path
from optic_parser import OpticParser
from temporal_diagram import TemporalDiagram

def main():
    # Parse OPTIC output from 2.4/base
    base_dir = Path("../2.4/base")
    output_file = base_dir / "output.txt"
    problem_file = base_dir / "problem.pddl"
    
    print("Parsing OPTIC output...")
    parser = OpticParser(output_file)
    timeline_data = parser.parse()
    
    # Get total time
    total_time = timeline_data['total_time']
    print(f"Total duration: {total_time:.1f}s")
    
    # Filter actions to keep only those in the last 120 seconds
    time_window = 120
    start_time = max(0, total_time - time_window)
    
    print(f"Creating compact diagram: {start_time:.1f}s - {total_time:.1f}s")
    
    filtered_actions = []
    for action in timeline_data['actions']:
        # Include action if it overlaps with our time window
        if action['end'] >= start_time:
            # Adjust times relative to start_time
            filtered_action = action.copy()
            filtered_action['start'] = max(0, action['start'] - start_time)
            filtered_action['end'] = action['end'] - start_time
            filtered_action['duration'] = filtered_action['end'] - filtered_action['start']
            filtered_actions.append(filtered_action)
    
    # Create new timeline data with filtered actions
    compact_timeline = {
        'robots': timeline_data['robots'],
        'actions': filtered_actions,
        'total_time': time_window
    }
    
    # Create temporal diagram with compact size
    diagram = TemporalDiagram(compact_timeline, problem_file)
    
    # Adjust earthquake windows to the new time range
    if diagram.earthquake_windows:
        adjusted_windows = []
        for start, end in diagram.earthquake_windows:
            # Only include earthquakes that overlap with our time window
            if end >= start_time:
                adj_start = max(0, start - start_time)
                adj_end = min(time_window, end - start_time)
                adjusted_windows.append((adj_start, adj_end))
        diagram.earthquake_windows = adjusted_windows
    
    print(f"Filtered actions: {len(filtered_actions)} (from {len(timeline_data['actions'])})")
    print(f"Earthquakes in window: {len(diagram.earthquake_windows)}")
    
    # Generate compact diagram with smaller width
    output_path = base_dir / "viz" / "temporal_diagram_compact.png"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Compact figure size: 14 inches wide instead of ~47
    diagram.render(output_path, figsize=(20, 8), show_labels=True)
    
    print(f"\n✅ Compact diagram saved to: {output_path}")

if __name__ == "__main__":
    main()
