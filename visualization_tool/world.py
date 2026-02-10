from typing import Dict, List, Tuple, Any

class WorldState:
    """
    Represents the state of the planning world.
    Tracks locations of robots, drones, artifacts, pods, and their properties.
    """
    
    def __init__(self, problem_data: Dict[str, Any]):
        # --- Static World Configuration ---
        self.locations = problem_data['locations']          
        self.artifacts = problem_data['artifacts']          
        self.connections = problem_data['connections']      
        
        # --- Dynamic State: Robots & Drones ---
        # Structure: { 'robot_name': { 'loc': ..., 'hand': ..., 'seal': ... } }
        self.robots = {}
        self.drones = {}

        # --- Static Permissions ---
        # Structure: { 'robot_name': { 'access': set(), 'pickup': set() } }
        self.permissions = {}

        # --- Dynamic State: Environment ---
        self.artifact_locations = {}            # Map: Artifact -> Location (Floor)
        self.pod_locations = {}                 # Map: Pod -> Location (Floor)
        self.pod_contents = {}                  # Map: Pod -> Artifact (If pod is on floor and full)
        
        self.artifact_temperatures = {}         
        self.artifact_fragility = {}            
        
        self._initialize_from_problem(problem_data)
    
    def _initialize_from_problem(self, problem_data: Dict[str, Any]):
        """Parse the PDDL :init section to populate the starting state."""
        initial_state = problem_data['initial_state']
        objects = problem_data.get('objects', {})

        # 1. Identify Robots and Drones
        robot_names = objects.get('robot', [])
        # If no explicit robot objects (like in 2.1), try to infer from 'robot-at' or default to 'robot'
        if not robot_names:
            if 'robot-at' in initial_state:
                robot_names = list(set(r for r, l in initial_state['robot-at']))
            else:
                robot_names = ['robot'] # Default for single robot domain 2.1

        for r in robot_names:
            self.robots[r] = {
                'loc': None,
                'hand_empty': True,
                'carrying': None,           # Held directly
                'carrying_in_pod': None,    # Held inside a pod
                'carrying_empty_pods': False,
                'sealing_mode': False
            }
            self.permissions[r] = {'access': set(), 'pickup': set()}

        drone_names = objects.get('drone', [])
        for d in drone_names:
            self.drones[d] = {
                'loc': None,
                'carrying': None,
                'empty': True
            }

        # 2. Parse Initial State
        
        # Robot State
        if 'robot-at' in initial_state:
            for r, l in initial_state['robot-at']:
                if r in self.robots: self.robots[r]['loc'] = l
        
        if 'hands-empty' in initial_state:
            for (r,) in initial_state['hands-empty']:
                 if r in self.robots: self.robots[r]['hand_empty'] = True

        if 'sealing-mode-on' in initial_state: # 2.1
            for (r,) in initial_state['sealing-mode-on']:
                 if r in self.robots: self.robots[r]['sealing_mode'] = True
        
        if 'sealing-mode' in initial_state: # 2.2
            for (r,) in initial_state['sealing-mode']:
                 if r in self.robots: self.robots[r]['sealing_mode'] = True

        # Drone State
        if 'drone-at' in initial_state:
             for d, l in initial_state['drone-at']:
                 if d in self.drones: self.drones[d]['loc'] = l
        
        if 'drone-empty' in initial_state:
             for (d,) in initial_state['drone-empty']:
                 if d in self.drones: self.drones[d]['empty'] = True

        # Permissions
        if 'can-access' in initial_state:
            for r, l in initial_state['can-access']:
                if r in self.permissions: self.permissions[r]['access'].add(l)
        
        if 'can-pickup' in initial_state:
             for r, t in initial_state['can-pickup']:
                 # storing artifact types they can pick up
                 if r in self.permissions: self.permissions[r]['pickup'].add(t)

        # Object Locations
        if 'artifact-at' in initial_state:
            for artifact, location in initial_state['artifact-at']:
                self.artifact_locations[artifact] = location

        if 'pod-at' in initial_state:
            for pod, location in initial_state['pod-at']:
                self.pod_locations[pod] = location
        
        # Fallback for old PDDL if used
        if 'contains-empty-pod' in initial_state:
            for location, pod in initial_state['contains-empty-pod']:
                self.pod_locations[pod] = location
        
        if 'contains-full-pod' in initial_state:
             for location, pod in initial_state['contains-full-pod']:
                self.pod_locations[pod] = location
                # Ideally we need to know WHAT logic says is inside, but :init might separate (pod-contains p a)
                
        if 'pod-contains' in initial_state:
             # If pod is on floor and full, we need to track it
             for pod, artifact in initial_state['pod-contains']:
                 if pod in self.pod_locations:
                     self.pod_contents[pod] = artifact

        # Artifact Properties
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
        
        # Identify agent by checking which parameter matches a known robot or drone
        agent = None
        if parameters:
            for p in parameters:
                if p in self.robots:
                    agent = p
                    break
                elif p in self.drones:
                    agent = p
                    break
        
        # Fallback for old single-robot compatibility if agent not found
        if not agent and parameters:
             agent = parameters[0]
        
        description = f"{clean_name}"
        if parameters:
            description += f" ({', '.join(parameters)})"
        
        # ==========================
        #  ROBOT MODES & MOVEMENT
        # ==========================
        if agent in self.robots:
            robot = self.robots[agent]
            
            if clean_name == 'activate-seal':
                robot['sealing_mode'] = True
                return f"[{agent}] SEALING ON"
            
            elif clean_name == 'deactivate-seal':
                robot['sealing_mode'] = False
                return f"[{agent}] SEALING OFF"
            
            elif 'move-to' in clean_name:
                 # parameters: ?r ?from ?to
                 # We need to find the destination. Usually the last parameter is 'to' or 'from'
                 # But let's look at standard PDDL: move-to ?r ?from ?to
                 # If we found the agent, we can try to find the location in the remaining params.
                 # Heuristic: The last parameter that is a location? 
                 # Or simply use the indices if we know the action signature.
                 
                 # 2.2 signatures:
                 # move-to-pressurized-room ?r ?from ?to
                 # move-to-unpressurized-room ?r ?from ?to
                 
                 if len(parameters) >= 3:
                    # In standard move actions, destination is usually last
                    to_loc = parameters[-1] 
                    robot['loc'] = to_loc
                    
                    # --- AUTO UNSEAL LOGIC ---
                    # Strictly check for 'pressurized' but NOT 'unpressurized'
                    if 'pressurized' in clean_name and 'unpressurized' not in clean_name:
                         robot['sealing_mode'] = False
                    
                    # Also explicit check for the specific action name to be safe
                    if 'move-to-pressurized-room' in clean_name:
                        robot['sealing_mode'] = False

                    return f"[{agent}] Move -> {to_loc}"
            
            elif 'try-to-enter' in clean_name:
                 # Parameters: ?r ?to ?from
                 if len(parameters) >= 3:
                     to_loc = parameters[1]
                     robot['loc'] = to_loc
                     return f"[{agent}] Risk Entry -> {to_loc}"

            # ==========================
            #  POD MANAGEMENT
            # ==========================
            elif clean_name == 'pick-up-empty-pod':
                # Parameters: ?r ?l ?p
                pod = parameters[2]
                robot['carrying_empty_pods'] = True
                robot['hand_empty'] = False
                if pod in self.pod_locations:
                    del self.pod_locations[pod]
                return f"[{agent}] Pick Up Empty {pod}"

            elif clean_name == 'drop-empty-pod':
                # Parameters: ?r ?p ?l
                pod = parameters[1]
                location = parameters[2]
                robot['carrying_empty_pods'] = False
                robot['hand_empty'] = True
                self.pod_locations[pod] = location
                return f"[{agent}] Drop Empty {pod}"
                
            elif clean_name == 'put-in-pod':
                # Parameters: ?a ?l ?r ?p or ?a ?at ?l ?r ?p (2.2)
                # We need to find args relative to robot.
                # 2.1: ?a ?l ?r ?p (r is index 2)
                # 2.2: ?a ?at ?l ?r ?p (r is index 3)
                
                # Heuristic: find which param matches the robot name we identified
                try:
                    r_idx = parameters.index(agent)
                    artifact = parameters[0] # Artifact is usually first
                    # pod is last
                    pod = parameters[-1] 
                except:
                    # Fallback
                    artifact = parameters[0]
                    pod = parameters[-1]

                robot['carrying_empty_pods'] = False # Pod is now full
                robot['carrying_in_pod'] = artifact  # Robot holds full pod
                
                if artifact in self.artifact_locations:
                    del self.artifact_locations[artifact]
                return f"[{agent}] Secure {artifact} in Pod"

            elif clean_name == 'pick-up-full-pod':
                 # Varies: ?r ?l ?p ?a OR ?r ?l ?p ?a ?at
                 pod = parameters[2]
                 artifact = parameters[3]
                 
                 robot['carrying_in_pod'] = artifact
                 robot['hand_empty'] = False
                 
                 if pod in self.pod_locations:
                     del self.pod_locations[pod]
                 
                 if pod in self.pod_contents:
                     del self.pod_contents[pod]
                     
                 return f"[{agent}] Pick Up Full {pod}"

            elif clean_name == 'drop-full-pod':
                # Varies: ?r ?p ?l ?a
                pod = parameters[1]
                location = parameters[2]
                artifact = parameters[3]
                
                robot['carrying_in_pod'] = None
                robot['hand_empty'] = True
                
                self.pod_locations[pod] = location
                self.pod_contents[pod] = artifact 
                
                return f"[{agent}] Drop Full {pod}"

            # ==========================
            #  ARTIFACT MANIPULATION
            # ==========================
            elif clean_name == 'pick-up-artifact-standard':
                # 2.1: ?a ?l ?r (r is index 2)
                # 2.2: ?a ?at ?l ?r (r is index 3)
                artifact = parameters[0]
                robot['carrying'] = artifact
                robot['hand_empty'] = False
                if artifact in self.artifact_locations:
                    del self.artifact_locations[artifact]
                return f"[{agent}] Pick Up {artifact}"

            elif 'release-artifact' in clean_name:
                # 2.1: release-artifact ?r ?a ?l
                # 2.2: release-artifact ?r ?a ?l
                
                if 'from-pod' in clean_name:
                     # release-artifact-from-pod: ?r ?a ?l ?p
                     artifact = parameters[1]
                     location = parameters[2]
                     robot['carrying_in_pod'] = None
                     robot['carrying_empty_pods'] = True  # Keep the empty pod
                     desc = "Unpack"
                else:
                     # release-artifact: ?r ?a ?l
                     artifact = parameters[1]
                     location = parameters[2]
                     robot['carrying'] = None
                     robot['hand_empty'] = True
                     desc = "Drop"

                # Apply Temperature Changes
                if 'cryo' in clean_name:
                    self.artifact_temperatures[artifact] = 'cold'
                    desc += " (Cool)"

                # Update Location (Item is now on the floor)
                self.artifact_locations[artifact] = location
                return f"[{agent}] {desc} {artifact}"
        
        # ==========================
        #  DRONE ACTIONS
        # ==========================
        elif agent in self.drones:
            drone = self.drones[agent]
            
            if 'enters-dangerous-room' in clean_name or 'exits-dangerous-room' in clean_name:
                # ?d ?to ?from
                to_loc = parameters[1]
                drone['loc'] = to_loc
                return f"[{agent}] Fly -> {to_loc}"
            
            elif 'pickup-artifact' in clean_name:
                # ?d ?a ?l
                artifact = parameters[1]
                drone['carrying'] = artifact
                drone['empty'] = False
                if artifact in self.artifact_locations:
                    del self.artifact_locations[artifact]
                return f"[{agent}] Airlift {artifact}"
                
            elif 'release-artifact' in clean_name:
                # ?d ?a ?l
                artifact = parameters[1]
                loc = parameters[2]
                drone['carrying'] = None
                drone['empty'] = True
                self.artifact_locations[artifact] = loc
                return f"[{agent}] Drop {artifact}"

        return description

    def get_state_summary(self) -> str:
        # Simple summary of the first robot or general status
        if self.robots:
            r = list(self.robots.values())[0]
            return f"Loc: {r['loc']} | Agents: {len(self.robots)+len(self.drones)}"
        return "Empty World"

    def to_dict(self) -> Dict[str, Any]:
        return {
            'robots': {k: v.copy() for k, v in self.robots.items()},
            'drones': {k: v.copy() for k, v in self.drones.items()},
            'permissions': {k: {'access': list(v['access']), 'pickup': list(v['pickup'])} for k, v in self.permissions.items()},
            'pod_contents': self.pod_contents.copy(), 
            'artifact_locations': self.artifact_locations.copy(),
            'pod_locations': self.pod_locations.copy(),
            'artifact_temperatures': self.artifact_temperatures.copy(),
            'artifact_count': len(self.artifact_locations) + len(self.pod_contents)
        }