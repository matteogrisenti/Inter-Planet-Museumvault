import re
import sys

ROOMS = {"entrance", "maintenance-tunnel", "hall-a", "hall-b", "cryo-chamber", "anti-vibration-pods-room", "stasis-lab"}
TYPES = {"technological", "scientific", "top-secret"}
ROBOTS = {"curator", "technician", "scientist"}

def get_artifact(args):
    for arg in args:
        if arg not in ROOMS and arg not in TYPES and arg not in ROBOTS:
            return arg
    return "unknown_artifact"

def wrap_text(text, max_len=15):
    """Spezza il testo lungo inserendo degli a capo (\\n) sui trattini"""
    if len(text) > max_len and '-' in text:
        parts = text.split('-')
        mid = len(parts) // 2
        return "-".join(parts[:mid]) + "-\\n" + "-".join(parts[mid:])
    return text

def build_tree_from_dot(input_path, output_path):
    with open(input_path, 'r') as f:
        content = f.read()

    node_pattern = re.compile(r'(PS\d+)\s*\[.*?label="(.*?)".*?\];')
    process_nexts, process_dones, actions = [], [], []

    for match in node_pattern.finditer(content):
        nid, label = match.groups()
        args_match = re.search(r'\[(.*?)\]', label)
        if not args_match: continue
        
        args = [a.strip() for a in args_match.group(1).split(',')]
        artifact = get_artifact(args)
        rooms = [arg for arg in args if arg in ROOMS]
        
        if "m_process_room_next" in label:
            process_nexts.append((artifact, rooms[0], label))
        elif "m_process_room_done" in label:
            process_dones.append((rooms[0], label))
        elif "delivery" in label:
            from_loc, to_loc = args[2], args[3]
            robot = args[0]
            actions.append((robot, artifact, from_loc, to_loc, label))

    # --- SETUP GRAFICO OTTIMIZZATO PER A4 VERTICALE ---
    dot_lines = [
        'digraph HierarchicalPlan {',
        '  rankdir="TB";',                  # Sviluppo Verticale (Top to Bottom)
        '  size="8.3,11.7!";',              # Dimensioni esatte foglio A4 (forzate con il !)
        '  ratio="compress";',              # Comprime per riempire lo spazio
        '  nodesep=0.15;',                  # Spazio orizzontale minimo tra nodi fratelli
        '  ranksep=0.3;',                   # Spazio verticale ridotto tra livelli
        '  node [fontname="Helvetica", fontsize=9, style="filled", margin="0.05,0.02", width=0.8];',
        '  edge [color="#555555", penwidth=1.0, arrowsize=0.6];',
        ''
    ]
    
    node_counter = 0
    def new_id():
        nonlocal node_counter
        node_counter += 1
        return f"N{node_counter}"

    root_id = new_id()
    dot_lines.append(f'  {root_id} [label="ROOT\\n(Mission)", shape="box", fillcolor="#e2e3e5"];')

    rooms_to_process = set([r for a, r, l in process_nexts])

    for room in rooms_to_process:
        curr_spine_task = new_id()
        dot_lines.append(f'  {curr_spine_task} [label="Task:\\nprocess-room\\n({room})", shape="box", fillcolor="#d4edda"];')
        dot_lines.append(f'  {root_id} -> {curr_spine_task};')
        
        room_nexts = [x for x in process_nexts if x[1] == room]
        
        for art, _, m_label in room_nexts:
            w_art = wrap_text(art)
            m_id = new_id()
            
            # Colore per m_process_room_next (Grigio Chiaro)
            dot_lines.append(f'  {m_id} [label="Method:\\nprocess_next\\n({w_art})", shape="ellipse", fillcolor="#e9ecef"];')
            dot_lines.append(f'  {curr_spine_task} -> {m_id};')
            
            deliver_task = new_id()
            dot_lines.append(f'  {deliver_task} [label="Task:\\ndeliver\\n({w_art})", shape="box", fillcolor="#d4edda"];')
            dot_lines.append(f'  {m_id} -> {deliver_task};')
            
            next_spine_task = new_id()
            dot_lines.append(f'  {next_spine_task} [label="Task:\\nprocess-room\\n({room})", shape="box", fillcolor="#d4edda"];')
            dot_lines.append(f'  {m_id} -> {next_spine_task};')
            
            curr_spine_task = next_spine_task
            
            art_actions = [a for a in actions if a[1] == art]
            curr_deliver_task = deliver_task
            path, current_loc, remaining = [], room, art_actions.copy()
            
            while remaining:
                next_act = next((a for a in remaining if a[2] == current_loc), None)
                if next_act:
                    path.append(next_act)
                    current_loc = next_act[3]
                    remaining.remove(next_act)
                else: break 
                    
            for i, (rob, art, f_loc, t_loc, a_label) in enumerate(path):
                is_last = (i == len(path) - 1)
                
                dm_id = new_id()
                if is_last:
                    method_name = "direct"
                    m_color = "#fff3cd"  # Giallo chiaro per il Direct
                else:
                    method_name = "relay"
                    m_color = "#ffe69c"  # Giallo leggermente più scuro per il Relay
                    
                dot_lines.append(f'  {dm_id} [label="Method:\\n{method_name}\\n({f_loc} ->\\n{t_loc})", shape="ellipse", fillcolor="{m_color}"];')
                dot_lines.append(f'  {curr_deliver_task} -> {dm_id};')
                
                act_id = new_id()
                dot_lines.append(f'  {act_id} [label="Action:\\ndelivery\\n-\\n[{rob}]\\n[{w_art}]\\n[{f_loc} ->\\n{t_loc}]", shape="box", fillcolor="#cce5ff"];')
                dot_lines.append(f'  {dm_id} -> {act_id};')
                
                if not is_last:
                    next_deliver = new_id()
                    dot_lines.append(f'  {next_deliver} [label="Task:\\ndeliver\\n({w_art})", shape="box", fillcolor="#d4edda"];')
                    dot_lines.append(f'  {dm_id} -> {next_deliver};')
                    curr_deliver_task = next_deliver

        if any(r == room for r, l in process_dones):
            done_id = new_id()
            # Colore per m_process_room_done (Rosso chiaro)
            dot_lines.append(f'  {done_id} [label="Method:\\nprocess_done\\n(Room Empty)", shape="ellipse", fillcolor="#f8d7da"];')
            dot_lines.append(f'  {curr_spine_task} -> {done_id};')

    dot_lines.append('}')
    
    with open(output_path, 'w') as f:
        f.write("\n".join(dot_lines))
        
    print(f"✅ Grafo A4 VERTICALE generato: {output_path}")

if __name__ == "__main__":
    INPUT = sys.argv[1] if len(sys.argv) > 1 else "raw_plan.dot"
    OUTPUT = sys.argv[2] if len(sys.argv) > 2 else "albero_a4.dot"
    build_tree_from_dot(INPUT, OUTPUT)