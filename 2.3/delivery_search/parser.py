import re
import sys

ROOMS = {"entrance", "maintenance-tunnel", "hall-a", "hall-b", "cryo-chamber", "anti-vibration-pods-room", "stasis-lab"}
TYPES = {"technological", "scientific", "top-secret"}
ROBOTS = {"curator", "technician", "scientist"}

def get_artifacts(args):
    """Estrae tutti gli artefatti (tutto ciò che non è stanza, tipo o robot)"""
    return [arg for arg in args if arg not in ROOMS and arg not in TYPES and arg not in ROBOTS]

def wrap_text(text, max_len=15):
    if len(text) > max_len and '-' in text:
        parts = text.split('-')
        mid = len(parts) // 2
        return "-".join(parts[:mid]) + "-\\n" + "-".join(parts[mid:])
    return text

def build_tree_from_dot(input_path, output_path):
    with open(input_path, 'r') as f:
        content = f.read()

    node_pattern = re.compile(r'(PS\d+)\s*\[.*?label="(.*?)".*?\];')
    
    # Strutture dati per i nodi
    process_steps = [] # Lista di (tipo, artifacts, room, label)
    process_dones = []
    actions = []       # Lista di (robot, artifacts, from, to, type_action, label)

    for match in node_pattern.finditer(content):
        nid, label = match.groups()
        args_match = re.search(r'\[(.*?)\]', label)
        if not args_match: continue
        
        args = [a.strip() for a in args_match.group(1).split(',')]
        arts = get_artifacts(args)
        rooms = [arg for arg in args if arg in ROOMS]
        
        if "m_process_room_double" in label:
            process_steps.append(("double", arts, rooms[0], label))
        elif "m_process_room_single" in label:
            process_steps.append(("single", arts, rooms[0], label))
        elif "m_process_room_done" in label:
            process_dones.append((rooms[0], label))
        elif "double_delivery" in label:
            # args: robot, a1, a2, from, to, at1, at2
            actions.append((args[0], [args[1], args[2]], args[3], args[4], "double", label))
        elif "delivery" in label:
            # args: robot, a, from, to, at
            actions.append((args[0], [args[1]], args[2], args[3], "single", label))

    dot_lines = [
        'digraph HierarchicalPlan {',
        '  rankdir="TB"; size="8.3,11.7!"; ratio="compress"; nodesep=0.15; ranksep=0.3;',
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

    # Raggruppa per stanza
    rooms_found = sorted(list(set([step[2] for step in process_steps])))

    for room in rooms_found:
        curr_spine_task = new_id()
        dot_lines.append(f'  {curr_spine_task} [label="Task:\\nprocess-room\\n({room})", shape="box", fillcolor="#d4edda"];')
        dot_lines.append(f'  {root_id} -> {curr_spine_task};')
        
        room_steps = [x for x in process_steps if x[2] == room]
        
        for kind, arts, _, m_label in room_steps:
            m_id = new_id()
            label_arts = " &\\n".join([wrap_text(a) for a in arts])
            
            # Colore Method: Grigio per single, Viola per double
            m_color = "#e9ecef" if kind == "single" else "#e1bee7"
            dot_lines.append(f'  {m_id} [label="Method:\\nprocess_{kind}\\n({label_arts})", shape="ellipse", fillcolor="{m_color}"];')
            dot_lines.append(f'  {curr_spine_task} -> {m_id};')
            
            # Task di consegna
            task_name = "double_deliver" if kind == "double" else "deliver"
            deliver_task = new_id()
            dot_lines.append(f'  {deliver_task} [label="Task:\\n{task_name}\\n({label_arts})", shape="box", fillcolor="#d4edda"];')
            dot_lines.append(f'  {m_id} -> {deliver_task};')
            
            # Ricorsione spine
            next_spine_task = new_id()
            dot_lines.append(f'  {next_spine_task} [label="Task:\\nprocess-room\\n({room})", shape="box", fillcolor="#d4edda"];')
            dot_lines.append(f'  {m_id} -> {next_spine_task};')
            curr_spine_task = next_spine_task
            
            # Trova le azioni corrispondenti a questi artefatti
            relevant_actions = [a for a in actions if any(art in a[1] for art in arts)]
            
            # Disegna le azioni sotto il deliver_task
            for i, (rob, a_list, f_loc, t_loc, a_kind, a_label) in enumerate(relevant_actions):
                # Semplificazione: trattiamo ogni azione trovata come un metodo diretto/relay per ora
                act_m_id = new_id()
                m_color = "#fff3cd" if a_kind == "single" else "#d1c4e9"
                dot_lines.append(f'  {act_m_id} [label="Method:\\n{a_kind}_step\\n({f_loc}->{t_loc})", shape="ellipse", fillcolor="{m_color}"];')
                dot_lines.append(f'  {deliver_task} -> {act_m_id};')
                
                act_id = new_id()
                label_act = f"Action:\\n{a_kind}_delivery\\n[{rob}]\\n{f_loc}->{t_loc}"
                dot_lines.append(f'  {act_id} [label="{label_act}", shape="box", fillcolor="#cce5ff"];')
                dot_lines.append(f'  {act_m_id} -> {act_id};')

        # Chiusura stanza
        if any(r == room for r, l in process_dones):
            done_id = new_id()
            dot_lines.append(f'  {done_id} [label="Method:\\nprocess_done\\n(Room Empty)", shape="ellipse", fillcolor="#f8d7da"];')
            dot_lines.append(f'  {curr_spine_task} -> {done_id};')

    dot_lines.append('}')
    with open(output_path, 'w') as f:
        f.write("\n".join(dot_lines))
    print(f"✅ Grafo HTN con Double Delivery generato: {output_path}")

if __name__ == "__main__":
    INPUT = sys.argv[1] if len(sys.argv) > 1 else "raw_plan.dot"
    OUTPUT = sys.argv[2] if len(sys.argv) > 2 else "albero_a4.dot"
    build_tree_from_dot(INPUT, OUTPUT)