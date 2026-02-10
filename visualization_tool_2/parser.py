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
        # Structured history for JSON export
        # Format: [{ "step": 0, "action": "INIT", "state": [...] }, { "step": 1, ... }]
        self.history = [] 
        
        # 1. Load Domain
        self._load_domain(domain_file)
        
        # 2. Load Initial State
        self._load_problem(problem_file)
        
        # Record Initial State
        self.history.append({
            "step": 0,
            "action": "INITIAL_STATE",
            "parameters": [],
            "effects": {"add": [], "del": []},
            "state": sorted([list(atom) for atom in self.current_state], key=lambda x: (x[0], x[1]))
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
        # Regex: matches _ followed by detdup/psi, optional chars, end of string
        base_name = re.sub(r'_(detdup|psi).*', '', action_name)

        if base_name not in self.actions:
            print(f"⚠️ Warning Step {step_num}: Unknown action '{action_name}' (normalized: '{base_name}')")
            # Record state anyway to keep continuity
            self.history.append({
                "step": step_num,
                "action": action_name,
                "error": "Unknown Action",
                "state": sorted([list(atom) for atom in self.current_state])
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
        
        # Save structured history
        self.history.append({
            "step": step_num,
            "action": base_name,
            "raw_action": action_name,
            "parameters": args,
            "effects": {
                "add": [list(a) for a in add_list],
                "del": [list(d) for d in del_list]
            },
            "state": sorted([list(atom) for atom in self.current_state], key=lambda x: (x[0], x[1]))
        })

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
        """Saves the complete execution history to trace.json"""
        output_path = output_dir / "trace.json"
        with open(output_path, 'w') as f:
            json.dump(self.history, f, indent=2)
        print(f"✅ Trace saved to: {output_path}")
        print(f"   Total Steps: {len(self.history)}")