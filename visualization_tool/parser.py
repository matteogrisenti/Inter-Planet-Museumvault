import re
from typing import Dict, List, Tuple, Any

class PDDLParser:
    """Parser for PDDL domain and problem files."""
    
    @staticmethod
    def remove_comments(content: str) -> str:
        """Removes all comments (starting with ;) from the PDDL content."""
        lines = []
        for line in content.splitlines():
            # Keep everything before the first semicolon
            clean_line = line.split(';')[0]
            lines.append(clean_line)
        return '\n'.join(lines)

    @staticmethod
    def parse_domain(filepath: str) -> Dict[str, Any]:
        with open(filepath, 'r') as f:
            raw_content = f.read()
        
        content = PDDLParser.remove_comments(raw_content)
        
        domain_data = {
            'name': None,
            'types': [],
            'predicates': [],
            'actions': []
        }
        
        name_match = re.search(r'\(define\s+\(domain\s+(\S+)\)', content, re.IGNORECASE)
        if name_match:
            domain_data['name'] = name_match.group(1)
        
        types_match = re.search(r'\(:types\s+(.*?)\)', content, re.DOTALL | re.IGNORECASE)
        if types_match:
            types_text = types_match.group(1)
            # Normalize types to lowercase just in case
            domain_data['types'] = [t.strip().lower() for t in types_text.split() if t.strip() and t.strip() != '-']
        
        action_pattern = r'\(:action\s+(\S+)'
        # Normalize action names to lowercase
        domain_data['actions'] = [a.lower() for a in re.findall(action_pattern, content, re.IGNORECASE)]
        
        return domain_data
    
    @staticmethod
    def parse_problem(filepath: str) -> Dict[str, Any]:
        with open(filepath, 'r') as f:
            raw_content = f.read()
            
        content = PDDLParser.remove_comments(raw_content)
        
        problem_data = {
            'name': None,
            'domain': None,
            'objects': {},
            'locations': [],
            'artifacts': [],
            'connections': [],
            'initial_state': {}
        }
        
        # 1. Metadata
        name_match = re.search(r'\(define\s+\(problem\s+(\S+)\)', content, re.IGNORECASE)
        if name_match:
            problem_data['name'] = name_match.group(1)
        
        domain_match = re.search(r'\(:domain\s+(\S+)\)', content, re.IGNORECASE)
        if domain_match:
            problem_data['domain'] = domain_match.group(1)
        
        # 2. Objects (Now with Lowercase Normalization)
        obj_start = re.search(r'\(:objects', content, re.IGNORECASE)
        if obj_start:
            start_idx = obj_start.end()
            objects_text, _ = PDDLParser._extract_balanced_block(content, start_idx)
            
            flat_text = objects_text.replace('\n', ' ')
            parts = re.split(r'\s+-\s+', flat_text)
            
            for i in range(1, len(parts)):
                current_chunk = parts[i].split()
                if not current_chunk: continue
                
                obj_type = current_chunk[0].lower() # Lowercase Type
                
                if i == 1:
                    prev_chunk = parts[i-1].split()
                    obj_names = prev_chunk
                else:
                    prev_chunk = parts[i-1].split()
                    obj_names = prev_chunk[1:]
                
                # --- CRITICAL FIX: Lowercase all object names ---
                obj_names = [name.lower() for name in obj_names]
                
                if obj_type not in problem_data['objects']:
                    problem_data['objects'][obj_type] = []
                
                problem_data['objects'][obj_type].extend(obj_names)
                
                if obj_type == 'location':
                    problem_data['locations'].extend(obj_names)
                elif 'artifact' in obj_type or 'item' in obj_type:
                    problem_data['artifacts'].extend(obj_names)

        # 3. Init State (Now with Lowercase Normalization)
        init_start = re.search(r'\(:init', content, re.IGNORECASE)
        if init_start:
            start_idx = init_start.end()
            init_text, _ = PDDLParser._extract_balanced_block(content, start_idx)
            
            predicate_pattern = r'\(\s*([^\s\)]+)(?:\s+([^)]*))?\s*\)'
            predicates = re.findall(predicate_pattern, init_text)
            
            for pred_name, args_str in predicates:
                pred_name = pred_name.lower()
                
                # --- CRITICAL FIX: Lowercase all arguments ---
                args = [a.lower() for a in args_str.split()] if args_str else []
                
                if 'connected' in pred_name and len(args) >= 2:
                    problem_data['connections'].append((args[0], args[1]))
                
                if pred_name not in problem_data['initial_state']:
                    problem_data['initial_state'][pred_name] = []
                
                problem_data['initial_state'][pred_name].append(tuple(args))

        return problem_data
    
    @staticmethod
    def _extract_balanced_block(content: str, start_index: int) -> Tuple[str, int]:
        balance = 1 
        current_idx = start_index
        extracted = []
        
        while current_idx < len(content) and balance > 0:
            char = content[current_idx]
            if char == '(':
                balance += 1
            elif char == ')':
                balance -= 1
            
            if balance > 0:
                extracted.append(char)
            current_idx += 1
            
        return "".join(extracted), current_idx

    @staticmethod
    def parse_plan(filepath: str) -> List[Tuple[str, List[str]]]:
        actions = []
        with open(filepath, 'r') as f:
            for line in f:
                line = line.split(';')[0].strip()
                if not line: continue
                
                if line.startswith('(') and line.endswith(')'):
                    line = line[1:-1]
                
                parts = line.split()
                if parts:
                    # --- CRITICAL FIX: Lowercase action name and parameters ---
                    action_name = parts[0].lower()
                    parameters = [p.lower() for p in parts[1:]]
                    actions.append((action_name, parameters))
        return actions