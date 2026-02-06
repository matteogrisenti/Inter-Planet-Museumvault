from typing import Dict, List, Tuple, Any

class WorldState:
    """Represents the state of the planning world."""
    
    def __init__(self, problem_data: Dict[str, Any]):
        self.locations = problem_data['locations']
        self.artifacts = problem_data['artifacts']
        self.connections = problem_data['connections']
        
        # Current state predicates
        self.robot_location = None
        self.artifact_locations = {}
        self.robot_carrying = None
        self.hand_empty = True
        self.carrying_in_pod = None
        self.carrying_empty_pods = False
        self.sealing_mode = False
        self.artifact_temperatures = {}
        self.artifact_fragility = {}
        
        # Initialize from problem initial state
        self._initialize_from_problem(problem_data['initial_state'])
    
    def _initialize_from_problem(self, initial_state: Dict[str, List[Tuple]]):
        """Initialize world state from problem initial state."""
        # Robot location
        if 'robot-at' in initial_state:
            self.robot_location = initial_state['robot-at'][0][0]
        
        # Hand empty
        self.hand_empty = 'hand-empty' in initial_state
        
        # Artifact locations
        if 'artifact-at' in initial_state:
            for artifact, location in initial_state['artifact-at']:
                self.artifact_locations[artifact] = location
        
        # Artifact temperatures
        if 'warm' in initial_state:
            for (artifact,) in initial_state['warm']:
                self.artifact_temperatures[artifact] = 'warm'
        
        if 'cold' in initial_state:
            for (artifact,) in initial_state['cold']:
                self.artifact_temperatures[artifact] = 'cold'
        
        # Artifact fragility
        if 'fragile' in initial_state:
            for (artifact,) in initial_state['fragile']:
                self.artifact_fragility[artifact] = True
        
        if 'no-fragile' in initial_state:
            for (artifact,) in initial_state['no-fragile']:
                self.artifact_fragility[artifact] = False
    
    def apply_action(self, action_name: str, parameters: List[str]) -> str:
        """Apply an action to update the world state. Returns action description."""
        description = f"{action_name}"
        if parameters:
            description += f" ({', '.join(parameters)})"
        
        # Handle different action types
        if action_name == 'activate-seal':
            self.sealing_mode = True
            return "[LOCK] Robot activates sealing mode"
        
        elif action_name == 'deactivate-seal':
            self.sealing_mode = False
            return "[UNLOCK] Robot deactivates sealing mode"
        
        elif 'move-empty' in action_name:
            from_loc, to_loc = parameters[0], parameters[1]
            self.robot_location = to_loc
            return f"Robot moves empty: {from_loc} -> {to_loc}"
        
        elif 'move-carrying' in action_name or 'move-fragile' in action_name:
            from_loc = parameters[0]
            to_loc = parameters[1]
            artifact = parameters[2] if len(parameters) > 2 else "unknown"
            self.robot_location = to_loc
            
            if 'fragile' in action_name or self.carrying_in_pod:
                return f"Robot moves with {artifact} in pod: {from_loc} -> {to_loc}"
            else:
                return f"Robot moves carrying {artifact}: {from_loc} -> {to_loc}"
        
        elif action_name == 'pick-up':
            artifact, location = parameters[0], parameters[1]
            self.robot_carrying = artifact
            self.hand_empty = False
            if artifact in self.artifact_locations:
                del self.artifact_locations[artifact]
            return f"PICK UP {artifact} at {location}"
        
        elif action_name == 'secure-pick-up':
            artifact, location = parameters[0], parameters[1]
            self.carrying_in_pod = artifact
            self.carrying_empty_pods = False
            if artifact in self.artifact_locations:
                del self.artifact_locations[artifact]
            return f"[SECURE] Robot picks up {artifact} in pod at {location}"
        
        elif action_name == 'pick-up-empty-pod':
            location = parameters[0]
            self.carrying_empty_pods = True
            self.hand_empty = False
            return f"PICK UP empty pod at {location}"
        
        elif action_name == 'drop-standard' or action_name == 'drop-in-cryo':
            artifact, location = parameters[0], parameters[1]
            self.artifact_locations[artifact] = location
            self.robot_carrying = None
            self.hand_empty = True
            
            if action_name == 'drop-in-cryo':
                self.artifact_temperatures[artifact] = 'cold'
                return f"DROP {artifact} in cryo-chamber (now cold)"
            return f"DROP {artifact} at {location}"
        
        elif 'drop-standard-from-pod' in action_name or 'drop-in-cryo-from-pod' in action_name:
            artifact, location = parameters[0], parameters[1]
            self.artifact_locations[artifact] = location
            self.carrying_in_pod = None
            self.carrying_empty_pods = True
            
            if 'cryo' in action_name:
                self.artifact_temperatures[artifact] = 'cold'
                return f"UNLOAD {artifact} from pod in cryo-chamber (now cold)"
            return f"UNLOAD {artifact} from pod at {location}"
        
        elif action_name == 'drop-empty-pod':
            self.carrying_empty_pods = False
            self.hand_empty = True
            return f"DROP empty pod"
        
        return description
    
    def get_state_summary(self) -> str:
        """Get a text summary of current state."""
        summary = []
        summary.append(f"Robot at: {self.robot_location}")
        
        if self.sealing_mode:
            summary.append("Mode: SEALED")
        else:
            summary.append("Mode: Normal")
        
        if self.hand_empty:
            summary.append("Hands: Empty")
        elif self.carrying_in_pod:
            summary.append(f"Carrying in pod: {self.carrying_in_pod}")
        elif self.carrying_empty_pods:
            summary.append("Carrying: Empty pod")
        elif self.robot_carrying:
            summary.append(f"Carrying: {self.robot_carrying}")
        
        return " | ".join(summary)