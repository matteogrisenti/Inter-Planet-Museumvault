import os
import re
import subprocess
import sys
from pathlib import Path

def get_location_map(sas_path):
    """Parses output.sas to find the 'robot-at' variable."""
    loc_map = {}
    target_var_id = -1
    
    # Ensure sas_path is a string for os.path.exists if passed as Path
    if not os.path.exists(str(sas_path)):
        return -1, {}

    with open(sas_path, 'r') as f:
        lines = [l.strip() for l in f.readlines()]
        
    iterator = iter(lines)
    try:
        var_counter = 0
        while True:
            line = next(iterator)
            if line == 'begin_variable':
                next(iterator) # name
                next(iterator) # axiom
                num_vals = int(next(iterator))
                values = []
                is_loc_var = False
                for _ in range(num_vals):
                    val = next(iterator)
                    values.append(val)
                    if "robot-at" in val: is_loc_var = True
                next(iterator) # end

                if is_loc_var:
                    target_var_id = var_counter
                    for idx, val in enumerate(values):
                        # Clean: "Atom robot-at(curator, hall-a)" -> "hall-a"
                        clean = val.replace("Atom robot-at(", "").replace(")", "").split(", ")[-1]
                        loc_map[idx] = clean
                    return target_var_id, loc_map
                var_counter += 1
    except StopIteration:
        pass
    return -1, {}

def generate_dot_content(policy_path, loc_var_id, loc_map):
    dot_lines = [
        'digraph CyclicPolicy {',
        'rankdir=LR;',
        'node [shape=circle, style=filled, fillcolor=lightblue, fontname="Helvetica", width=1.5];',
        'edge [fontname="Helvetica", fontsize=10];'
    ]
    
    unique_edges = set()
    
    with open(policy_path, 'r') as f:
        content = f.read()

    blocks = content.split("If holds:")
    
    for block in blocks:
        if not block.strip(): continue
        
        # 1. FIND START LOCATION
        match = re.search(f"var{loc_var_id}:(\d+)", block)
        start_loc = None
        if match:
            val_idx = int(match.group(1))
            if val_idx in loc_map: start_loc = loc_map[val_idx]
        
        if not start_loc or "<" in start_loc: continue

        # 2. FIND ACTION
        if "Execute:" in block:
            exec_line = block.split("Execute:")[1].strip()
            full_action = exec_line.split("/")[0].strip()
            parts = full_action.split()
            act_name = parts[0]
            
            # --- SPECIAL LOGIC: SEISMIC LOOP ---
            if "try-to-enter" in act_name:
                # Find destination (arg that isn't start_loc)
                dest_loc = start_loc
                for p in parts[1:]:
                    if p in loc_map.values() and p != start_loc:
                        dest_loc = p; break
                
                # EDGE 1: SUCCESS (Green)
                edge_sig_ok = f'{start_loc}->{dest_loc}:Success'
                if edge_sig_ok not in unique_edges:
                    label = f"{act_name}\\n(Success)"
                    dot_lines.append(f'"{start_loc}" -> "{dest_loc}" [label="{label}", color=green, penwidth=2.0, fontcolor=darkgreen];')
                    unique_edges.add(edge_sig_ok)
                
                # EDGE 2: FAILURE (Red Loop)
                edge_sig_fail = f'{start_loc}->{start_loc}:Fail'
                if edge_sig_fail not in unique_edges:
                    label = f"{act_name}\\n(Fail)"
                    dot_lines.append(f'"{start_loc}" -> "{start_loc}" [label="{label}", color=red, style=dashed, penwidth=2.0, fontcolor=firebrick];')
                    unique_edges.add(edge_sig_fail)
                continue 

            # --- STANDARD ACTIONS ---
            end_loc = start_loc
            if "move" in act_name:
                for p in parts[1:]:
                    if p in loc_map.values() and p != start_loc:
                        end_loc = p; break
            
            edge_sig = f'{start_loc}->{end_loc}:{act_name}'
            if edge_sig not in unique_edges:
                dot_lines.append(f'"{start_loc}" -> "{end_loc}" [label="{act_name}"];')
                unique_edges.add(edge_sig)

    dot_lines.append("}")
    return "\n".join(dot_lines)

def run_policy_viz(input_dir: Path, output_dir: Path):
    """
    Main entry point for Policy Visualization.
    Expects input_dir to contain a 'policy' subdirectory with 'output.sas' and 'policy.out'.
    """
    # 1. Define Paths
    policy_subdir = input_dir / "policy"
    sas_file = policy_subdir / "output.sas"
    policy_file = policy_subdir / "policy.out"
    
    output_dot = output_dir / "cyclic_graph.dot"
    output_img = output_dir / "cyclic_graph.png"

    print(f"--- Analyzing Policy in {policy_subdir} ---")

    # 2. Validation
    if not policy_subdir.exists():
        print(f"❌ Error: 'policy' subdirectory not found in {input_dir}")
        return
    if not sas_file.exists():
        print(f"❌ Error: {sas_file.name} not found.")
        return
    if not policy_file.exists():
        print(f"❌ Error: {policy_file.name} not found.")
        return

    # 3. Logic
    print("Parsing SAS file for location variables...")
    loc_id, loc_map = get_location_map(str(sas_file))
    
    if loc_id == -1:
        print("❌ Could not find 'robot-at' variable in SAS file.")
        return

    print("Generating Graph Definitions...")
    dot_output = generate_dot_content(str(policy_file), loc_id, loc_map)
    
    if not dot_output:
        print("❌ Failed to generate graph content (empty output).")
        return

    # 4. Save and Render
    with open(output_dot, "w") as f:
        f.write(dot_output)
    print(f"Saved DOT file: {output_dot}")

    try:
        # Check if dot is installed
        subprocess.run(["dot", "-V"], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print("Rendering Image (Graphviz)...")
        subprocess.run(["dot", "-Tpng", str(output_dot), "-o", str(output_img)], check=True)
        print(f"✅ Generated Graph Image: {output_img}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("⚠️  Warning: 'dot' command (Graphviz) not found or failed.")
        print("   The .dot file was saved, but the PNG could not be rendered.")
        print("   Install Graphviz (e.g., 'sudo apt install graphviz') to fix this.")