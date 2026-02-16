import re
import json
from pathlib import Path

class PDDLParser:
    """Parses PDDL files (Domain, Problem) and Plan files."""
    
    @staticmethod
    def tokenize(text):
        # Remove comments
        text = re.sub(r';.*$', '', text, flags=re.MULTILINE)
        text = text.lower()
        # Pad parentheses
        text = text.replace('(', ' ( ').replace(')', ' ) ')
        return text.split()

    @staticmethod
    def parse_sexpr(tokens):
        if not tokens:
            raise SyntaxError("Unexpected EOF")
        token = tokens.pop(0)
        if token == '(':
            L = []
            while tokens[0] != ')':
                L.append(PDDLParser.parse_sexpr(tokens))
            tokens.pop(0) # pop ')'
            return L
        elif token == ')':
            raise SyntaxError("Unexpected )")
        else:
            return token

    @staticmethod
    def parse_file(filepath):
        with open(filepath, 'r') as f:
            content = f.read()
        tokens = PDDLParser.tokenize(content)
        expressions = []
        while tokens:
            expressions.append(PDDLParser.parse_sexpr(tokens))
        return expressions

class Action:
    def __init__(self, name, parameters, effects):
        self.name = name
        self.parameters = parameters
        self.effects = effects

    def __repr__(self):
        return f"<Action {self.name}>"

class World:
    def __init__(self, domain_file, problem_file, plan_file):
        self.actions = {}
        self.current_state = set()
        # Artifact metadata for visualization
        self.artifact_types = {}  # artifact_name -> type (scientific, technological, top-secret)
        self.artifact_colors = {}  # artifact_name -> color code
        
        # Structured history for JSON export (OLD FORMAT - just a list)
        self.history = [] 
        
        # 1. Load Domain
        self._load_domain(domain_file)
        
        # 2. Load Initial State
        self._load_problem(problem_file)
        
        # Record Initial State with artifact metadata
        self.history.append({
            "step": 0,
            "action": "INITIAL_STATE",
            "parameters": [],
            "effects": {"add": [], "del": []},
            "state": sorted([list(atom) for atom in self.current_state], key=lambda x: (x[0], x[1])),
            "artifact_metadata": self._get_artifact_metadata(),
            "slot_info": self._extract_slot_information()
        })

        # 3. Execute Plan
        self._execute_plan(plan_file)

    def _load_domain(self, filepath):
        print(f"Loading Domain: {filepath.name}...")
        exprs = PDDLParser.parse_file(filepath)
        domain_def = next(e for e in exprs if e[0] == 'define' and 'domain' in e[1])
        
        for item in domain_def:
            if isinstance(item, list) and item[0] == ':action':
                name = item[1]
                
                # Parse Parameters (stripping types like '- robot')
                raw_params = item[item.index(':parameters') + 1]
                clean_params = []
                i = 0
                while i < len(raw_params):
                    token = raw_params[i]
                    if token.startswith('?'):
                        clean_params.append(token)
                    if token == '-':
                        i += 1 
                    i += 1
                
                # Parse Effect
                effect_idx = item.index(':effect') + 1
                effects = item[effect_idx]
                
                self.actions[name] = Action(name, clean_params, effects)

    def _load_problem(self, filepath):
        print(f"Loading Problem: {filepath.name}...")
        exprs = PDDLParser.parse_file(filepath)
        problem_def = next(e for e in exprs if e[0] == 'define' and 'problem' in e[1])
        
        for item in problem_def:
            if isinstance(item, list) and item[0] == ':init':
                for atom in item[1:]:
                    if isinstance(atom, list):
                        self.current_state.add(tuple(atom))
                        
                        # Extract artifact type information
                        if len(atom) >= 3 and atom[0] == 'is-type':
                            artifact_name = atom[1]
                            artifact_type = atom[2]
                            self.artifact_types[artifact_name] = artifact_type
                            self.artifact_colors[artifact_name] = self._assign_color(artifact_type)

    def _assign_color(self, artifact_type):
        """Assign colors based on artifact type"""
        color_map = {
            'scientific': '#3B82F6',      # Blue
            'technological': '#10B981',   # Green
            'top-secret': '#EF4444',      # Red
            'default': '#6B7280'          # Gray
        }
        return color_map.get(artifact_type, color_map['default'])

    def _get_artifact_metadata(self):
        """Generate metadata for all artifacts including type, color, and temperature"""
        metadata = {}
        
        for artifact, artifact_type in self.artifact_types.items():
            # Determine if artifact is warm or cold
            is_warm = any(atom[0] == 'warm' and atom[1] == artifact for atom in self.current_state)
            is_cold = any(atom[0] == 'cold' and atom[1] == artifact for atom in self.current_state)
            
            # Get base color and adjust for temperature
            base_color = self.artifact_colors[artifact]
            
            metadata[artifact] = {
                'type': artifact_type,
                'base_color': base_color,
                'display_color': self._adjust_color_for_temperature(base_color, is_warm, is_cold),
                'temperature': 'warm' if is_warm else ('cold' if is_cold else 'unknown'),
                'is_warm': is_warm,
                'is_cold': is_cold
            }
        
        return metadata

    def _adjust_color_for_temperature(self, base_color, is_warm, is_cold):
        """
        Adjust color intensity based on temperature:
        - warm: stronger/darker version
        - cold: lighter/pastel version
        """
        if is_warm:
            return base_color
        elif is_cold:
            return self._lighten_color(base_color)
        else:
            return base_color

    def _lighten_color(self, hex_color, factor=0.5):
        """Convert hex color to a lighter shade"""
        hex_color = hex_color.lstrip('#')
        
        # Convert to RGB
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
        
        # Lighten by mixing with white
        r = int(r + (255 - r) * factor)
        g = int(g + (255 - g) * factor)
        b = int(b + (255 - b) * factor)
        
        return f'#{r:02X}{g:02X}{b:02X}'

    def _execute_plan(self, filepath):
        print(f"Executing Plan: {filepath.name}...")
        with open(filepath, 'r') as f:
            content = f.read()
        
        tokens = PDDLParser.tokenize(content)
        step = 1
        
        while tokens:
            if tokens[0] == '(':
                expr = PDDLParser.parse_sexpr(tokens)
                action_name = expr[0]
                args = expr[1:]
                self.apply_action(action_name, args, step)
                step += 1
            else:
                tokens.pop(0)

    def apply_action(self, action_name, args, step_num):
        # Clean action name (remove _detdup_0, _psi_1, etc.)
        base_name = re.sub(r'_(detdup|psi).*', '', action_name)

        if base_name not in self.actions:
            print(f"⚠️ Warning Step {step_num}: Unknown action '{action_name}' (normalized: '{base_name}')")
            # Record state anyway to keep continuity
            self.history.append({
                "step": step_num,
                "action": action_name,
                "error": "Unknown Action",
                "state": sorted([list(atom) for atom in self.current_state]),
                "artifact_metadata": self._get_artifact_metadata(),
                "slot_info": self._extract_slot_information()
            })
            return

        action_def = self.actions[base_name]
        bindings = dict(zip(action_def.parameters, args))
        
        add_list = set()
        del_list = set()
        
        self._resolve_effects(action_def.effects, bindings, add_list, del_list)
        
        # Apply updates
        self.current_state = {atom for atom in self.current_state if atom not in del_list}
        self.current_state.update(add_list)
        
        # Save structured history (OLD FORMAT - just append to list)
        self.history.append({
            "step": step_num,
            "action": base_name,
            "raw_action": action_name,
            "parameters": args,
            "effects": {
                "add": [list(a) for a in add_list],
                "del": [list(d) for d in del_list]
            },
            "state": sorted([list(atom) for atom in self.current_state], key=lambda x: (x[0], x[1])),
            "artifact_metadata": self._get_artifact_metadata(),
            "slot_info": self._extract_slot_information()
        })

    def _extract_slot_information(self):
        """
        Extract what each robot is carrying in each slot.
        Compatible with both old domain (hands-empty, carrying, carrying-second-object)
        and new domain (hands-empty-slot-1/2, carrying-slot-1/2, carrying-pod-slot-1)
        """
        slot_info = {}
        
        for atom in self.current_state:
            if len(atom) < 2:
                continue
                
            predicate = atom[0]
            
            # NEW DOMAIN - Slot-based predicates
            if predicate == 'carrying-slot-1' and len(atom) >= 3:
                robot = atom[1]
                artifact = atom[2]
                if robot not in slot_info:
                    slot_info[robot] = {}
                slot_info[robot]['slot_1'] = {'type': 'artifact', 'item': artifact}
            
            elif predicate == 'carrying-slot-2' and len(atom) >= 3:
                robot = atom[1]
                artifact = atom[2]
                if robot not in slot_info:
                    slot_info[robot] = {}
                slot_info[robot]['slot_2'] = {'type': 'artifact', 'item': artifact}
            
            elif predicate == 'carrying-pod-slot-1' and len(atom) >= 3:
                robot = atom[1]
                pod = atom[2]
                # Check if pod contains artifact
                artifact_in_pod = None
                for other_atom in self.current_state:
                    if len(other_atom) >= 3 and other_atom[0] == 'pod-contains' and other_atom[1] == pod:
                        artifact_in_pod = other_atom[2]
                        break
                
                if robot not in slot_info:
                    slot_info[robot] = {}
                slot_info[robot]['slot_1'] = {
                    'type': 'pod',
                    'item': pod,
                    'contains': artifact_in_pod,
                    'empty': artifact_in_pod is None
                }
            
            # OLD DOMAIN - Legacy predicates (for backward compatibility)
            elif predicate == 'carrying' and len(atom) >= 3:
                robot = atom[1]
                artifact = atom[2]
                if robot not in slot_info:
                    slot_info[robot] = {}
                slot_info[robot]['main_hand'] = {'type': 'artifact', 'item': artifact}
            
            elif predicate == 'carrying-empty-pod' and len(atom) >= 3:
                robot = atom[1]
                pod = atom[2]
                if robot not in slot_info:
                    slot_info[robot] = {}
                slot_info[robot]['main_hand'] = {'type': 'pod', 'item': pod, 'empty': True}
            
            elif predicate == 'carrying-full-pod' and len(atom) >= 3:
                robot = atom[1]
                pod = atom[2]
                # Find what's in the pod
                artifact_in_pod = None
                for other_atom in self.current_state:
                    if len(other_atom) >= 3 and other_atom[0] == 'pod-contains' and other_atom[1] == pod:
                        artifact_in_pod = other_atom[2]
                        break
                
                if robot not in slot_info:
                    slot_info[robot] = {}
                slot_info[robot]['main_hand'] = {
                    'type': 'pod',
                    'item': pod,
                    'contains': artifact_in_pod,
                    'empty': False
                }
            
            elif predicate == 'carrying-second-object' and len(atom) >= 3:
                robot = atom[1]
                artifact = atom[2]
                if robot not in slot_info:
                    slot_info[robot] = {}
                slot_info[robot]['second_hand'] = {'type': 'artifact', 'item': artifact}
        
        return slot_info

    def _resolve_effects(self, effect_node, bindings, add_list, del_list):
        if not isinstance(effect_node, list):
            return
        
        op = effect_node[0]
        
        if op == 'and':
            for sub in effect_node[1:]:
                self._resolve_effects(sub, bindings, add_list, del_list)
        elif op == 'not':
            atom = self._ground_atom(effect_node[1], bindings)
            del_list.add(atom)
        elif op == 'oneof':
            # HEURISTIC: Always assume the first outcome (Case A: Success) 
            self._resolve_effects(effect_node[1], bindings, add_list, del_list)
        else:
            # Base atom add
            atom = self._ground_atom(effect_node, bindings)
            add_list.add(atom)

    def _ground_atom(self, template, bindings):
        return tuple(bindings.get(item, item) for item in template)

    def save_trace_to_json(self, output_dir):
        """Saves the complete execution history to trace.json - OLD FORMAT (simple list)"""
        output_path = output_dir / "trace.json"
        
        # Save as simple list (OLD FORMAT for backward compatibility)
        with open(output_path, 'w') as f:
            json.dump(self.history, f, indent=2)
        
        print(f"✅ Trace saved to: {output_path}")
        print(f"   Total Steps: {len(self.history)}")
        print(f"   Artifacts tracked: {len(self.artifact_types)}")
        
        # Print color summary
        if self.artifact_types:
            print("\n📊 Artifact Color Mapping:")
            type_counts = {}
            for artifact, atype in self.artifact_types.items():
                type_counts[atype] = type_counts.get(atype, 0) + 1
            
            for atype, count in type_counts.items():
                color = self._assign_color(atype)
                print(f"   {atype}: {count} artifacts ({color})")