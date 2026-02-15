#!/usr/bin/env python3
"""
OPTIC Output Parser
Parses temporal planner output files from OPTIC.
"""

import re
from pathlib import Path
from typing import Dict, List, Any


class OpticParser:
    """Parses OPTIC temporal planner output files."""
    
    def __init__(self, output_file: Path):
        self.output_file = output_file
        self.actions = []
        self.robots = set()
        
    def parse(self) -> Dict[str, Any]:
        """
        Parse the OPTIC output file and extract temporal actions.
        
        Returns:
            Dictionary with 'robots' and 'actions' keys
        """
        with open(self.output_file, 'r') as f:
            lines = f.readlines()
        
        # Parse only the plan lines (format: "timestamp: (action params) [duration]")
        # Example: "0.000: (activate-seal curator)  [2.000]"
        plan_pattern = re.compile(r'^\s*(\d+\.\d+):\s*\(([^)]+)\)\s*\[(\d+\.\d+)\]')
        
        for line in lines:
            match = plan_pattern.match(line)
            if match:
                start_time = float(match.group(1))
                action_text = match.group(2).strip()
                duration = float(match.group(3))
                
                # Parse action and parameters
                parts = action_text.split()
                action_name = parts[0] if parts else "unknown"
                params = parts[1:] if len(parts) > 1 else []
                
                # Try to identify the robot from parameters
                robot = self._identify_robot(action_name, params)
                if robot:
                    self.robots.add(robot)
                
                # Calculate end time
                end_time = start_time + duration
                
                # Store action
                action_data = {
                    "start": start_time,
                    "end": end_time,
                    "duration": duration,
                    "action": action_name,
                    "robot": robot,
                    "params": params,
                    "full_text": action_text
                }
                self.actions.append(action_data)
        
        # Sort actions by start time
        self.actions.sort(key=lambda x: x["start"])
        
        return {
            "robots": sorted(list(self.robots)),
            "actions": self.actions,
            "total_time": max([a["end"] for a in self.actions]) if self.actions else 0
        }
    
    def _identify_robot(self, action_name: str, params: List[str]) -> str:
        """
        Identify which robot is performing the action.
        
        Common robot names: curator, technician, scientist
        
        Args:
            action_name: Name of the action
            params: List of parameters
            
        Returns:
            Robot name or None
        """
        # Known robot types
        known_robots = {'curator', 'technician', 'scientist'}
        
        # Check if any parameter is a known robot
        for param in params:
            if param.lower() in known_robots:
                return param.lower()
        
        # Some actions might have the robot as the first parameter
        if params and params[0].lower() in known_robots:
            return params[0].lower()
        
        return None
    
    def categorize_action(self, action_name: str) -> str:
        """
        Categorize action for color coding.
        
        Args:
            action_name: Name of the action
            
        Returns:
            Category name
        """
        action = action_name.lower()
        
        if 'move' in action:
            return 'movement'
        elif 'seal' in action:
            return 'sealing'
        elif 'pod' in action or 'pick-up' in action or 'drop' in action:
            return 'pod_management'
        elif 'artifact' in action or 'carrying' in action or 'release' in action or 'put-in' in action:
            return 'artifact_handling'
        elif 'cool' in action:
            return 'cooling'
        else:
            return 'other'


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python optic_parser.py <output.txt>")
        sys.exit(1)
    
    parser = OpticParser(Path(sys.argv[1]))
    data = parser.parse()
    
    print(f"Robots found: {data['robots']}")
    print(f"Total actions: {len(data['actions'])}")
    print(f"Total time: {data['total_time']:.3f} seconds")
    print(f"\nFirst 5 actions:")
    for action in data['actions'][:5]:
        print(f"  {action['start']:.3f} - {action['end']:.3f}: {action['full_text']} (robot: {action['robot']})")
