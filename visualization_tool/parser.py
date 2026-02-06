import re
from typing import Dict, List, Tuple, Any

class PDDLParser:
    """Parser for PDDL domain and problem files."""
    
    @staticmethod
    def parse_domain(filepath: str) -> Dict[str, Any]:
        """Parse PDDL domain file to extract types and predicates."""
        with open(filepath, 'r') as f:
            content = f.read()
        
        domain_data = {
            'name': None,
            'types': [],
            'predicates': [],
            'actions': []
        }
        
        # Extract domain name
        name_match = re.search(r'\(define\s+\(domain\s+(\S+)\)', content)
        if name_match:
            domain_data['name'] = name_match.group(1)
        
        # Extract types
        types_match = re.search(r'\(:types\s+(.*?)\)', content, re.DOTALL)
        if types_match:
            types_text = types_match.group(1)
            domain_data['types'] = [t.strip() for t in types_text.split() if t.strip() and t.strip() != '-']
        
        # Extract action names
        action_pattern = r'\(:action\s+(\S+)'
        domain_data['actions'] = re.findall(action_pattern, content)
        
        return domain_data
    
    @staticmethod
    def parse_problem(filepath: str) -> Dict[str, Any]:
        """Parse PDDL problem file to extract objects and initial state."""
        with open(filepath, 'r') as f:
            content = f.read()
        
        problem_data = {
            'name': None,
            'domain': None,
            'objects': {},
            'locations': [],
            'artifacts': [],
            'connections': [],
            'initial_state': {}
        }
        
        # Extract problem name
        name_match = re.search(r'\(define\s+\(problem\s+(\S+)\)', content)
        if name_match:
            problem_data['name'] = name_match.group(1)
        
        # Extract domain reference
        domain_match = re.search(r'\(:domain\s+(\S+)\)', content)
        if domain_match:
            problem_data['domain'] = domain_match.group(1)
        
        # Extract objects section - improved parsing
        objects_match = re.search(r'\(:objects(.*?)\n\s*\)', content, re.DOTALL)
        if objects_match:
            objects_text = objects_match.group(1)
            
            # Remove comments
            lines = []
            for line in objects_text.split('\n'):
                # Remove comment part
                if ';' in line:
                    line = line[:line.index(';')]
                lines.append(line.strip())
            
            objects_text = ' '.join(lines)
            
            # Parse object declarations: name1 name2 - type
            parts = re.split(r'\s+-\s+', objects_text)
            
            for i in range(1, len(parts)):
                # Get type (last word of this part before next -)
                type_and_names = parts[i].split()
                if not type_and_names:
                    continue
                    
                obj_type = type_and_names[0]
                
                # Get names (remaining words from previous part)
                if i > 0:
                    prev_part = parts[i-1]
                    obj_names = prev_part.split()
                    
                    if obj_type not in problem_data['objects']:
                        problem_data['objects'][obj_type] = []
                    
                    problem_data['objects'][obj_type].extend(obj_names)
                    
                    # Store locations and artifacts separately
                    if obj_type == 'location':
                        problem_data['locations'].extend(obj_names)
                    elif obj_type == 'artifact':
                        problem_data['artifacts'].extend(obj_names)
        
        # Parse initial state
        init_match = re.search(r'\(:init\s+(.*?)\)\s+\(:goal', content, re.DOTALL)
        if init_match:
            init_text = init_match.group(1)
            
            # Extract connections
            connection_pattern = r'\(connected\s+(\S+)\s+(\S+)\)'
            connections = re.findall(connection_pattern, init_text)
            problem_data['connections'] = connections
            
            # Extract all predicates
            predicate_pattern = r'\(([^)]+)\)'
            predicates = re.findall(predicate_pattern, init_text)
            
            for pred in predicates:
                parts = pred.split()
                if len(parts) > 0:
                    pred_name = parts[0]
                    pred_args = parts[1:]
                    
                    if pred_name not in problem_data['initial_state']:
                        problem_data['initial_state'][pred_name] = []
                    
                    problem_data['initial_state'][pred_name].append(tuple(pred_args))
        
        return problem_data
    
    @staticmethod
    def parse_plan(filepath: str) -> List[Tuple[str, List[str]]]:
        """Parse SAS plan file to extract actions."""
        actions = []
        
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith(';'):
                    continue
                
                # Remove outer parentheses
                if line.startswith('(') and line.endswith(')'):
                    line = line[1:-1]
                
                # Split action name and parameters
                parts = line.split()
                if parts:
                    action_name = parts[0]
                    parameters = parts[1:]
                    actions.append((action_name, parameters))
        
        return actions