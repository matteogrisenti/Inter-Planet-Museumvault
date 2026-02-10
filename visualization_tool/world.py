from typing import Dict, List, Tuple, Any

class WorldState:
    """
    Represents the state of the planning world.
    Tracks locations of robots, artifacts, pods, and their properties.
    """
    
    def __init__(self, problem_data: Dict[str, Any]):
        # --- Static World Configuration ---
        self.locations = problem_data['locations']          # List of all locations
        self.artifacts = problem_data['artifacts']          # List of all artifacts
        self.connections = problem_data['connections']      # Topology (connections between rooms)
        
        # --- Dynamic State: Robot ---
        self.robot_location = None              # Current room of the robot
        self.robot_hand_empty = True            # Flag: Is the robot's hand free?
        self.robot_carrying = None              # Name of artifact held in hand (if any)
        self.robot_carrying_in_pod = None       # Name of artifact held inside a pod (if any)
        self.robot_carrying_empty_pods = False  # Flag: Is robot holding an empty pod?
        self.robot_sealing_mode = False         # Flag: Is the robot sealed/pressurized?

        # --- Dynamic State: Environment ---
        self.artifact_locations = {}            # Map: Artifact Name -> Location
        self.pod_locations = {}                 # Map: Pod Name -> Location (for pods on the floor)
        self.artifact_temperatures = {}         # Map: Artifact Name -> 'warm' or 'cold'
        self.artifact_fragility = {}            # Map: Artifact Name -> boolean (is fragile?)
        
        # Initialize state from the parsed PDDL problem
        self._initialize_from_problem(problem_data['initial_state'])
    
    def _initialize_from_problem(self, initial_state: Dict[str, List[Tuple]]):
        """Parse the PDDL :init section to populate the starting state."""
        
        # 1. Robot Configuration
        if 'robot-at' in initial_state:
            # PDDL: (robot-at curator entrance) -> We want 'entrance' (index 1)
            self.robot_location = initial_state['robot-at'][0][1]
        
        self.robot_hand_empty = 'hands-empty' in initial_state
        
        # 2. Object Locations
        if 'artifact-at' in initial_state:
            for artifact, location in initial_state['artifact-at']:
                self.artifact_locations[artifact] = location

        if 'contains-empty-pod' in initial_state:
            for location, pod in initial_state['contains-empty-pod']:
                self.pod_locations[pod] = location
        
        if 'contains-full-pod' in initial_state:
            for location, pod in initial_state['contains-full-pod']:
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
        Returns a human-readable description of the event.
        """
        
        # Clean up action name (remove solver suffixes like _DETDUP)
        clean_name = action_name.split('_')[0].lower()
        
        # Build base description
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
            # Parameters: ?r ?from ?to
            if len(parameters) >= 3:
                from_loc, to_loc = parameters[1], parameters[2]
                self.robot_location = to_loc
                return f"Move: {from_loc} -> {to_loc}"
            
        elif 'try-to-enter' in clean_name:
             # Parameters: ?r ?to ?from (Note reversed order in domain!)
             if len(parameters) >= 3:
                 to_loc, from_loc = parameters[1], parameters[2]
                 self.robot_location = to_loc
                 return f"Enter Seismic Room: {from_loc} -> {to_loc}"

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
            # Logic: Robot puts an artifact (from hand or immediate vicinity) into the held pod
            artifact = parameters[0]
            self.robot_carrying_empty_pods = False
            self.robot_carrying_in_pod = artifact
            # Ensure artifact is removed from map if it was there
            if artifact in self.artifact_locations:
                del self.artifact_locations[artifact]
            return f"Secure {artifact} in Pod"

        elif clean_name == 'pick-up-full-pod':
             # Parameters: ?r ?l ?p ?a
             pod = parameters[2]
             artifact = parameters[3]
             self.robot_carrying_in_pod = artifact
             self.robot_hand_empty = False
             
             # Remove items from the floor
             if pod in self.pod_locations:
                 del self.pod_locations[pod]
             if artifact in self.artifact_locations:
                 del self.artifact_locations[artifact]
             return f"Pick Up Full {pod}"

        elif clean_name == 'drop-full-pod':
            # Parameters: ?r ?p ?l ?a
            pod = parameters[1]
            location = parameters[2]
            artifact = parameters[3]
            
            self.robot_carrying_in_pod = None
            self.robot_hand_empty = True
            
            # Place both pod and artifact on the map
            self.artifact_locations[artifact] = location
            self.pod_locations[pod] = location
            return f"Drop Full {pod} with {artifact}"

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
            # Common structure is usually: ?r ?a ?l ...
            artifact = parameters[1]
            location = parameters[2]
            
            desc_prefix = ""
            
            # 1. Determine Unload Source
            if 'from-pod' in clean_name:
                self.robot_carrying_in_pod = None
                self.robot_carrying_empty_pods = True  # Keep the empty pod
                desc_prefix = "Unload Pod: "
            else:
                self.robot_carrying = None
                self.robot_hand_empty = True
                desc_prefix = "Drop: "

            # 2. Apply Temperature Changes
            if 'cryo' in clean_name:
                self.artifact_temperatures[artifact] = 'cold'
                desc_prefix += "(Cooling) "

            # 3. Update Location
            self.artifact_locations[artifact] = location
            return f"{desc_prefix} {artifact} at {location}"

        return description

    def get_state_summary(self) -> str:
        """Get a text summary of current state for the visualization footer."""
        summary = []
        summary.append(f"Loc: {self.robot_location}")
        
        if self.robot_sealing_mode:
            summary.append("[SEALED]")
        
        if self.robot_carrying_in_pod:
            summary.append(f"Holding: Pod({self.robot_carrying_in_pod})")
        elif self.robot_carrying_empty_pods:
            summary.append("Holding: Empty Pod")
        elif self.robot_carrying:
            summary.append(f"Holding: {self.robot_carrying}")
        else:
            summary.append("Hands: Empty")
        
        return " | ".join(summary)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize current state to dictionary for debugging."""
        return {
            'robot_location': self.robot_location,
            'hand_empty': self.robot_hand_empty,
            'robot_carrying': self.robot_carrying,
            'carrying_in_pod': self.robot_carrying_in_pod,
            'carrying_empty_pods': self.robot_carrying_empty_pods,
            'sealing_mode': self.robot_sealing_mode,
            'artifact_locations': self.artifact_locations.copy(),
            'pod_locations': self.pod_locations.copy(),
            'artifact_temperatures': self.artifact_temperatures.copy(),
            'artifact_fragility': self.artifact_fragility.copy(),
            'artifact_count': len(self.artifact_locations)
        }