from typing import Dict, List, Tuple, Any

class WorldState:
    """
    Represents the state of the planning world.
    Tracks locations of robots, artifacts, pods, and their properties.
    """
    
    def __init__(self, problem_data: Dict[str, Any]):
        # --- Static World Configuration ---
        self.locations = problem_data['locations']          
        self.artifacts = problem_data['artifacts']          
        self.connections = problem_data['connections']      
        
        # --- Dynamic State: Robot ---
        self.robot_location = None              
        self.robot_hand_empty = True            
        self.robot_carrying = None              # Held directly
        self.robot_carrying_in_pod = None       # Held inside a pod
        self.robot_carrying_empty_pods = False  
        self.robot_sealing_mode = False         

        # --- Dynamic State: Environment ---
        self.artifact_locations = {}            # Map: Artifact -> Location (Floor)
        self.pod_locations = {}                 # Map: Pod -> Location (Floor)
        self.pod_contents = {}                  # Map: Pod -> Artifact (If pod is on floor and full)
        
        self.artifact_temperatures = {}         
        self.artifact_fragility = {}            
        
        self._initialize_from_problem(problem_data['initial_state'])
    
    def _initialize_from_problem(self, initial_state: Dict[str, List[Tuple]]):
        """Parse the PDDL :init section to populate the starting state."""
        
        # 1. Robot Configuration
        if 'robot-at' in initial_state:
            self.robot_location = initial_state['robot-at'][0][1]
        
        self.robot_hand_empty = 'hands-empty' in initial_state
        self.robot_sealing_mode = 'sealing-mode-on' in initial_state
        
        # 2. Object Locations
        if 'artifact-at' in initial_state:
            for artifact, location in initial_state['artifact-at']:
                self.artifact_locations[artifact] = location

        # --- FIX: Handle generic pod-at predicate ---
        if 'pod-at' in initial_state:
            for pod, location in initial_state['pod-at']:
                self.pod_locations[pod] = location
        
        # Fallback for old PDDL if used
        if 'contains-empty-pod' in initial_state:
            for location, pod in initial_state['contains-empty-pod']:
                self.pod_locations[pod] = location
        
        # 3. Artifact Properties
        if 'warm' in initial_state:
            for (artifact,) in initial_state['warm']:
                self.artifact_temperatures[artifact] = 'warm'
        
        if 'cold' in initial_state:
            for (artifact,) in initial_state['cold']:
                self.artifact_temperatures[artifact] = 'cold'
        
        if 'fragile' in initial_state:
            for (artifact,) in initial_state['fragile']:
                self.artifact_fragility[artifact] = True
    
    def apply_action(self, action_name: str, parameters: List[str]) -> str:
        """
        Updates the world state based on the action applied.
        """
        clean_name = action_name.split('_')[0].lower()
        description = f"{clean_name}"
        if parameters:
            description += f" ({', '.join(parameters)})"
        
        # ==========================
        #  ROBOT MODES & MOVEMENT
        # ==========================
        if clean_name == 'activate-seal':
            self.robot_sealing_mode = True
            return "[LOCK] Sealing Activated"
        
        elif clean_name == 'deactivate-seal':
            self.robot_sealing_mode = False
            return "[UNLOCK] Sealing Deactivated"
        
        elif 'move-to' in clean_name:
            if len(parameters) >= 3:
                # parameters: ?r ?from ?to
                to_loc = parameters[2]
                self.robot_location = to_loc
                return f"Move -> {to_loc}"
            
        elif 'try-to-enter' in clean_name:
             # Parameters: ?r ?to ?from
             if len(parameters) >= 3:
                 to_loc = parameters[1]
                 self.robot_location = to_loc
                 return f"Risk Entry -> {to_loc}"

        # ==========================
        #  POD MANAGEMENT
        # ==========================
        elif clean_name == 'pick-up-empty-pod':
            # Parameters: ?r ?l ?p
            pod = parameters[2]
            self.robot_carrying_empty_pods = True
            self.robot_hand_empty = False
            if pod in self.pod_locations:
                del self.pod_locations[pod]
            return f"Pick Up Empty {pod}"

        elif clean_name == 'drop-empty-pod':
            # Parameters: ?r ?p ?l
            pod = parameters[1]
            location = parameters[2]
            self.robot_carrying_empty_pods = False
            self.robot_hand_empty = True
            self.pod_locations[pod] = location
            return f"Drop Empty {pod}"
            
        elif clean_name == 'put-in-pod':
            # Parameters: ?a ?l ?r ?p
            # Robot puts item from FLOOR into HELD pod
            artifact = parameters[0]
            self.robot_carrying_empty_pods = False # Pod is now full
            self.robot_carrying_in_pod = artifact  # Robot holds full pod
            
            if artifact in self.artifact_locations:
                del self.artifact_locations[artifact]
            return f"Secure {artifact} in Pod"

        elif clean_name == 'pick-up-full-pod':
             # Parameters: ?r ?l ?p ?a
             # This assumes picking up from floor.
             # If PDDL state had (pod-contains p a), we must trust the plan parameters
             pod = parameters[2]
             artifact = parameters[3]
             
             self.robot_carrying_in_pod = artifact
             self.robot_hand_empty = False
             
             # Remove pod from floor
             if pod in self.pod_locations:
                 del self.pod_locations[pod]
             
             # Remove artifact from "pod contents on floor" tracker
             if pod in self.pod_contents:
                 del self.pod_contents[pod]
                 
             return f"Pick Up Full {pod}"

        elif clean_name == 'drop-full-pod':
            # Parameters: ?r ?p ?l ?a
            pod = parameters[1]
            location = parameters[2]
            artifact = parameters[3]
            
            self.robot_carrying_in_pod = None
            self.robot_hand_empty = True
            
            # --- FIX: Artifact stays INSIDE pod, not on floor ---
            self.pod_locations[pod] = location
            self.pod_contents[pod] = artifact 
            # We do NOT add to self.artifact_locations
            
            return f"Drop Full {pod}"

        # ==========================
        #  ARTIFACT MANIPULATION
        # ==========================
        elif clean_name == 'pick-up-artifact-standard':
            # Parameters: ?a ?l ?r
            artifact = parameters[0]
            self.robot_carrying = artifact
            self.robot_hand_empty = False
            if artifact in self.artifact_locations:
                del self.artifact_locations[artifact]
            return f"Pick Up {artifact}"

        elif 'release-artifact' in clean_name:
            # Handles: release-artifact, release-artifact-from-pod, release-artifact-in-cryo
            # Parameters usually: ?r ?a ?l ...
            
            # Check parameter index for artifact/location based on action variant
            if 'from-pod' in clean_name:
                 # release-artifact-from-pod: ?r ?a ?l ?p
                 artifact = parameters[1]
                 location = parameters[2]
                 self.robot_carrying_in_pod = None
                 self.robot_carrying_empty_pods = True  # Keep the empty pod
                 desc = "Unpack"
            else:
                 # release-artifact: ?r ?a ?l
                 artifact = parameters[1]
                 location = parameters[2]
                 self.robot_carrying = None
                 self.robot_hand_empty = True
                 desc = "Drop"

            # Apply Temperature Changes
            if 'cryo' in clean_name:
                self.artifact_temperatures[artifact] = 'cold'
                desc += " (Cool)"

            # Update Location (Item is now on the floor)
            self.artifact_locations[artifact] = location
            return f"{desc} {artifact}"

        return description

    def get_state_summary(self) -> str:
        summary = []
        summary.append(f"Loc: {self.robot_location}")
        if self.robot_sealing_mode: summary.append("[SEALED]")
        
        if self.robot_carrying_in_pod:
            summary.append(f"Holding: Full Pod ({self.robot_carrying_in_pod})")
        elif self.robot_carrying_empty_pods:
            summary.append("Holding: Empty Pod")
        elif self.robot_carrying:
            summary.append(f"Holding: {self.robot_carrying}")
        else:
            summary.append("Hands: Empty")
        return " | ".join(summary)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'robot_location': self.robot_location,
            'hand_empty': self.robot_hand_empty,
            'robot_carrying': self.robot_carrying,
            'carrying_in_pod': self.robot_carrying_in_pod,
            'pod_contents': self.pod_contents.copy(), # Track floor pods content
            'artifact_locations': self.artifact_locations.copy(),
            'pod_locations': self.pod_locations.copy(),
            'artifact_temperatures': self.artifact_temperatures.copy(),
            'artifact_count': len(self.artifact_locations) + len(self.pod_contents)
        }