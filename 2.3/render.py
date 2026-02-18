import re
import graphviz
import argparse

# ==========================================
# INCOLLA QUI IL RISULTATO DELLA TUA RICERCA
# ==========================================
HTN_LOG = """
m_deliver_relay[?handoff_loc=maintenance-tunnel,?target=stasis-lab,?at=scientific,?r_fetch=curator,?artifact_loc=hall-a,?a=mart-nord-core-drill,?r_deliver=scientist] @ deliver[mart-nord-core-drill,scientific,stasis-lab]
SHOP_methodm_deliver_relay_1_precondition[maintenance-tunnel,scientific,curator,mart-nord-core-drill,hall-a,stasis-lab,scientist]
m_deliver_relay_m_deliver_relay_2[?r_fetch=curator,?fetch_start=entrance,?artifact_loc=hall-a] @ prepare_robot_m_deliver_relay_2[curator,hall-a]
m_prep_fetch_pod[?start_loc=entrance,?p=pod1,?pod_loc=anti-vibration-pods-room,?r=curator,?artifact_loc=hall-a] @ prepare_robot[curator,entrance,hall-a]
SHOP_methodm_prep_fetch_pod_3_precondition[curator,entrance,pod1,anti-vibration-pods-room]      
SHOP_methodm_get_to_transit_20_precondition[entrance,maintenance-tunnel,anti-vibration-pods-room]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,entrance,maintenance-tunnel]
SHOP_methodm_step_pressurized_21_precondition[anti-vibration-pods-room]
move-to-pressurized-room[curator,maintenance-tunnel,anti-vibration-pods-room]
pick-up-pod-slot-1[curator,anti-vibration-pods-room,pod1]
SHOP_methodm_get_to_transit_20_precondition[anti-vibration-pods-room,maintenance-tunnel,hall-a] 
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,anti-vibration-pods-room,maintenance-tunnel]
SHOP_methodm_step_pressurized_21_precondition[hall-a]
move-to-pressurized-room[curator,maintenance-tunnel,hall-a]
m_load_into_pod[?loc=hall-a,?r=curator,?at=scientific,?p=pod1,?a=mart-nord-core-drill] @ load_artifact[curator,mart-nord-core-drill,scientific,hall-a]
put-in-pod[mart-nord-core-drill,scientific,hall-a,curator,pod1]
m_transport_chill[?curr_loc=hall-a,?a=mart-nord-core-drill,?r=curator,?target_loc=maintenance-tunnel,?cryo_loc=cryo-chamber,?at=scientific] @ transport_to_target[curator,mart-nord-core-drill,scientific,hall-a,maintenance-tunnel]
SHOP_methodm_transport_chill_9_precondition[cryo-chamber,curator]
SHOP_methodm_get_to_transit_20_precondition[hall-a,maintenance-tunnel,cryo-chamber]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,hall-a,maintenance-tunnel]
SHOP_methodm_step_pressurized_21_precondition[cryo-chamber]
move-to-pressurized-room[curator,maintenance-tunnel,cryo-chamber]
m_unload_pod_keep_cryo[?a=mart-nord-core-drill,?r=curator,?loc=cryo-chamber,?p=pod1] @ unload_artifact[curator,mart-nord-core-drill,cryo-chamber]
SHOP_methodm_unload_pod_keep_cryo_16_precondition[cryo-chamber]
release-artifact-cryo-from-pod-slot-1[curator,mart-nord-core-drill,cryo-chamber,pod1]
m_load_into_pod[?loc=cryo-chamber,?r=curator,?at=scientific,?p=pod1,?a=mart-nord-core-drill] @ load_artifact[curator,mart-nord-core-drill,scientific,cryo-chamber]
put-in-pod[mart-nord-core-drill,scientific,cryo-chamber,curator,pod1]
SHOP_methodm_get_to_direct_19_precondition[cryo-chamber,maintenance-tunnel]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,cryo-chamber,maintenance-tunnel]
m_unload_pod_drop_std[?p=pod1,?a=mart-nord-core-drill,?r=curator,?loc=maintenance-tunnel] @ unload_artifact[curator,mart-nord-core-drill,maintenance-tunnel]
SHOP_methodm_unload_pod_drop_std_15_precondition[maintenance-tunnel]
release-artifact-from-pod-slot-1[curator,mart-nord-core-drill,maintenance-tunnel,pod1]
drop-pod-slot-1[curator,pod1,maintenance-tunnel]
m_deliver_relay_m_deliver_relay_5[?handoff_loc=maintenance-tunnel,?r_deliver=scientist,?deliver_start=stasis-lab] @ prepare_robot_m_deliver_relay_5[scientist,maintenance-tunnel]
m_prep_direct[?artifact_loc=maintenance-tunnel,?start_loc=stasis-lab,?r=scientist] @ prepare_robot[scientist,stasis-lab,maintenance-tunnel]
SHOP_methodm_prep_direct_2_precondition[scientist,stasis-lab]
SHOP_methodm_get_to_direct_19_precondition[stasis-lab,maintenance-tunnel]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[scientist]
move-to-unpressurized-room[scientist,stasis-lab,maintenance-tunnel]
m_load_slot_1[?at=scientific,?a=mart-nord-core-drill,?loc=maintenance-tunnel,?r=scientist] @ load_artifact[scientist,mart-nord-core-drill,scientific,maintenance-tunnel]
pick-up-slot-1[mart-nord-core-drill,scientific,maintenance-tunnel,scientist]
SHOP_methodm_get_to_direct_19_precondition[maintenance-tunnel,stasis-lab]
SHOP_methodm_step_pressurized_21_precondition[stasis-lab]
move-to-pressurized-room[scientist,maintenance-tunnel,stasis-lab]
m_unload_s1_std[?a=mart-nord-core-drill,?r=scientist,?loc=stasis-lab] @ unload_artifact[scientist,mart-nord-core-drill,stasis-lab]
SHOP_methodm_unload_s1_std_10_precondition[stasis-lab]
release-artifact-slot-1[scientist,mart-nord-core-drill,stasis-lab]
m_deliver_relay[?handoff_loc=maintenance-tunnel,?target=stasis-lab,?at=technological,?r_fetch=technician,?artifact_loc=hall-b,?a=rover-wheel,?r_deliver=scientist] @ deliver[rover-wheel,technological,stasis-lab]
SHOP_methodm_deliver_relay_1_precondition[maintenance-tunnel,technological,technician,rover-wheel,hall-b,stasis-lab,scientist]
m_deliver_relay_m_deliver_relay_2[?r_fetch=technician,?fetch_start=entrance,?artifact_loc=hall-b] @ prepare_robot_m_deliver_relay_2[technician,hall-b]
m_prep_fetch_pod[?start_loc=entrance,?p=pod1,?pod_loc=maintenance-tunnel,?r=technician,?artifact_loc=hall-b] @ prepare_robot[technician,entrance,hall-b]
SHOP_methodm_prep_fetch_pod_3_precondition[technician,entrance,pod1,maintenance-tunnel]
SHOP_methodm_get_to_direct_19_precondition[entrance,maintenance-tunnel]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[technician]
move-to-unpressurized-room[technician,entrance,maintenance-tunnel]
pick-up-pod-slot-1[technician,maintenance-tunnel,pod1]
SHOP_methodm_get_to_direct_19_precondition[maintenance-tunnel,hall-b]
SHOP_methodm_step_seismic_23_precondition[hall-b]
wait-seismic-room-safe[technician,hall-b,maintenance-tunnel]
m_load_into_pod[?loc=hall-b,?r=technician,?at=technological,?p=pod1,?a=rover-wheel] @ load_artifact[technician,rover-wheel,technological,hall-b]
put-in-pod[rover-wheel,technological,hall-b,technician,pod1]
m_transport_direct[?curr_loc=hall-b,?a=rover-wheel,?r=technician,?target_loc=maintenance-tunnel,?at=technological] @ transport_to_target[technician,rover-wheel,technological,hall-b,maintenance-tunnel]
SHOP_methodm_get_to_direct_19_precondition[hall-b,maintenance-tunnel]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[technician]
move-to-unpressurized-room[technician,hall-b,maintenance-tunnel]
m_unload_pod_drop_std[?p=pod1,?a=rover-wheel,?r=technician,?loc=maintenance-tunnel] @ unload_artifact[technician,rover-wheel,maintenance-tunnel]
SHOP_methodm_unload_pod_drop_std_15_precondition[maintenance-tunnel]
release-artifact-from-pod-slot-1[technician,rover-wheel,maintenance-tunnel,pod1]
drop-pod-slot-1[technician,pod1,maintenance-tunnel]
m_deliver_relay_m_deliver_relay_5[?handoff_loc=maintenance-tunnel,?r_deliver=scientist,?deliver_start=stasis-lab] @ prepare_robot_m_deliver_relay_5[scientist,maintenance-tunnel]
m_prep_fetch_pod[?start_loc=stasis-lab,?p=pod1,?pod_loc=maintenance-tunnel,?r=scientist,?artifact_loc=maintenance-tunnel] @ prepare_robot[scientist,stasis-lab,maintenance-tunnel]
SHOP_methodm_prep_fetch_pod_3_precondition[scientist,stasis-lab,pod1,maintenance-tunnel]        
SHOP_methodm_get_to_direct_19_precondition[stasis-lab,maintenance-tunnel]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[scientist]
move-to-unpressurized-room[scientist,stasis-lab,maintenance-tunnel]
pick-up-pod-slot-1[scientist,maintenance-tunnel,pod1]
m_get_to_stay[?r=scientist,?from=maintenance-tunnel,?to=maintenance-tunnel] @ get_to[scientist,maintenance-tunnel,maintenance-tunnel]
stay-at[scientist,maintenance-tunnel]
m_load_into_pod[?loc=maintenance-tunnel,?r=scientist,?at=technological,?p=pod1,?a=rover-wheel] @ load_artifact[scientist,rover-wheel,technological,maintenance-tunnel]
put-in-pod[rover-wheel,technological,maintenance-tunnel,scientist,pod1]
SHOP_methodm_get_to_direct_19_precondition[maintenance-tunnel,stasis-lab]
SHOP_methodm_step_pressurized_21_precondition[stasis-lab]
move-to-pressurized-room[scientist,maintenance-tunnel,stasis-lab]
m_unload_pod_keep_std[?a=rover-wheel,?r=scientist,?loc=stasis-lab,?p=pod1] @ unload_artifact[scientist,rover-wheel,stasis-lab]
SHOP_methodm_unload_pod_keep_std_14_precondition[stasis-lab]
release-artifact-from-pod-slot-1[scientist,rover-wheel,stasis-lab,pod1]
m_deliver_task[?start_loc=maintenance-tunnel,?target=cryo-chamber,?at=scientific,?artifact_loc=hall-a,?r=curator,?a=mart-north-pole-ice-sample] @ deliver[mart-north-pole-ice-sample,scientific,cryo-chamber]
SHOP_methodm_deliver_task_0_precondition[mart-north-pole-ice-sample,hall-a,curator,scientific,cryo-chamber]
m_prep_fetch_pod[?start_loc=maintenance-tunnel,?p=pod2,?pod_loc=anti-vibration-pods-room,?r=curator,?artifact_loc=hall-a] @ prepare_robot[curator,maintenance-tunnel,hall-a]
SHOP_methodm_prep_fetch_pod_3_precondition[curator,maintenance-tunnel,pod2,anti-vibration-pods-room]
SHOP_methodm_get_to_direct_19_precondition[maintenance-tunnel,anti-vibration-pods-room]
SHOP_methodm_step_pressurized_21_precondition[anti-vibration-pods-room]
move-to-pressurized-room[curator,maintenance-tunnel,anti-vibration-pods-room]
pick-up-pod-slot-1[curator,anti-vibration-pods-room,pod2]
SHOP_methodm_get_to_transit_20_precondition[anti-vibration-pods-room,maintenance-tunnel,hall-a] 
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,anti-vibration-pods-room,maintenance-tunnel]
SHOP_methodm_step_pressurized_21_precondition[hall-a]
move-to-pressurized-room[curator,maintenance-tunnel,hall-a]
m_load_into_pod[?loc=hall-a,?r=curator,?at=scientific,?p=pod2,?a=mart-north-pole-ice-sample] @ load_artifact[curator,mart-north-pole-ice-sample,scientific,hall-a]
put-in-pod[mart-north-pole-ice-sample,scientific,hall-a,curator,pod2]
m_transport_direct[?curr_loc=hall-a,?a=mart-north-pole-ice-sample,?r=curator,?target_loc=cryo-chamber,?at=scientific] @ transport_to_target[curator,mart-north-pole-ice-sample,scientific,hall-a,cryo-chamber]
SHOP_methodm_get_to_transit_20_precondition[hall-a,maintenance-tunnel,cryo-chamber]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,hall-a,maintenance-tunnel]
SHOP_methodm_step_pressurized_21_precondition[cryo-chamber]
move-to-pressurized-room[curator,maintenance-tunnel,cryo-chamber]
m_unload_pod_keep_cryo[?a=mart-north-pole-ice-sample,?r=curator,?loc=cryo-chamber,?p=pod2] @ unload_artifact[curator,mart-north-pole-ice-sample,cryo-chamber]
SHOP_methodm_unload_pod_keep_cryo_16_precondition[cryo-chamber]
release-artifact-cryo-from-pod-slot-1[curator,mart-north-pole-ice-sample,cryo-chamber,pod2]     
m_deliver_task[?start_loc=cryo-chamber,?target=hall-a,?at=scientific,?artifact_loc=hall-b,?r=curator,?a=mart-sand-sample] @ deliver[mart-sand-sample,scientific,hall-a]
SHOP_methodm_deliver_task_0_precondition[mart-sand-sample,hall-b,curator,scientific,hall-a]     
m_prep_direct[?artifact_loc=hall-b,?start_loc=cryo-chamber,?r=curator] @ prepare_robot[curator,cryo-chamber,hall-b]
SHOP_methodm_prep_direct_2_precondition[curator,cryo-chamber]
SHOP_methodm_get_to_transit_20_precondition[cryo-chamber,maintenance-tunnel,hall-b]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,cryo-chamber,maintenance-tunnel]
SHOP_methodm_step_seismic_23_precondition[hall-b]
wait-seismic-room-safe[curator,hall-b,maintenance-tunnel]
m_load_into_pod[?loc=hall-b,?r=curator,?at=scientific,?p=pod2,?a=mart-sand-sample] @ load_artifact[curator,mart-sand-sample,scientific,hall-b]
put-in-pod[mart-sand-sample,scientific,hall-b,curator,pod2]
m_transport_direct[?curr_loc=hall-b,?a=mart-sand-sample,?r=curator,?target_loc=hall-a,?at=scientific] @ transport_to_target[curator,mart-sand-sample,scientific,hall-b,hall-a]
SHOP_methodm_get_to_transit_20_precondition[hall-b,maintenance-tunnel,hall-a]
SHOP_methodm_step_unpressurized_22_precondition[maintenance-tunnel]
activate-seal[curator]
move-to-unpressurized-room[curator,hall-b,maintenance-tunnel]
SHOP_methodm_step_pressurized_21_precondition[hall-a]
move-to-pressurized-room[curator,maintenance-tunnel,hall-a]
m_unload_pod_keep_std[?a=mart-sand-sample,?r=curator,?loc=hall-a,?p=pod2] @ unload_artifact[curator,mart-sand-sample,hall-a]
SHOP_methodm_unload_pod_keep_std_14_precondition[hall-a]
release-artifact-from-pod-slot-1[curator,mart-sand-sample,hall-a,pod2]
"""
# --- CONFIGURAZIONI GLOBALI ---
HIDE_SHOP_PRECONDITIONS = True  # Nasconde i check di sistema per pulire l'albero

# Mappatura dei Livelli della Gerarchia
TASK_LEVELS = {
    'deliver': 1, 'deliver_two': 1,
    'prepare_robot': 2, 'load_artifact': 2, 'transport_to_target': 2, 'unload_artifact': 2,
    'get_to': 3, 'step': 4
}

def build_tree(max_depth):
    # Setup del Grafo
    dot = graphviz.Digraph(comment='PANDA HTN Plan', format='png')
    
    # Layout Orizzontale e HD
    dot.attr(
        rankdir='TB',       # Top to Bottom
        dpi='300',          # Risoluzione HD
        nodesep='0.6',      # Spazio orizzontale
        ranksep='1.0',      # Spazio verticale
        outputorder='edgesfirst'
    )
    
    dot.attr('node', fontname='Helvetica', fontsize='12', style='filled, rounded', shape='box', margin='0.2,0.1')

    root_id = "root"
    dot.node(root_id, f"MISSION\nSTART", fillcolor="#333333", fontcolor="white", shape='ellipse')

    stack = [(root_id, 0)]
    node_count = 0

    lines = HTN_LOG.strip().split('\n')

    for line in lines:
        line = line.strip()
        if not line: continue
        line = re.sub(r'^\d+:\s*', '', line)

        node_count += 1
        curr_id = f"node_{node_count}"

        # ----------------------------------------------------
        # CASO 1: TASK ASTRATTI (Metodi intermedi)
        # ----------------------------------------------------
        if ' @ ' in line:
            method_part, task_part = line.split(' @ ')
            
            # 1. Estrai il nome pulito del task
            match = re.match(r'^([a-zA-Z0-9_-]+)', task_part)
            task_name = match.group(1) if match else task_part

            # 2. FILTRO: Salta i task di compilazione interna (richiesta precedente)
            if "_m_" in task_name: 
                continue 

            # 3. NUOVO: Estrai i Parametri del Task
            # La stringa è tipo: "deliver[p1,p2,p3]"
            task_params = ""
            if '[' in task_part and ']' in task_part:
                # Prende tutto ciò che c'è tra la prima [ e l'ultima ]
                start = task_part.find('[') + 1
                end = task_part.rfind(']')
                task_params = task_part[start:end]

            # Estrai il nome del metodo
            method_name = method_part.split('[')[0]
            
            level = TASK_LEVELS.get(task_name, 2)
            
            # ---> FILTRO DI PROFONDITÀ <---
            if level > max_depth:
                continue
            
            # Colori in base al livello
            color = "#add8e6" if level == 1 else "#98fb98" if level == 2 else "#ffffe0"
            
            # 4. Costruzione della Label con Parametri
            if task_params:
                # Opzionale: sostituisce le virgole con "a capo" per non allargare troppo il box
                formatted_params = task_params.replace(',', ',\n') 
                label = f"{task_name.upper()}\n----------------\n{formatted_params}\n----------------\n({method_name})"
            else:
                label = f"{task_name.upper()}\n({method_name})"

            while stack and stack[-1][1] >= level:
                stack.pop()
                
            parent_id = stack[-1][0]
            
            dot.node(curr_id, label, fillcolor=color)
            dot.edge(parent_id, curr_id, color="#666666")
            stack.append((curr_id, level))

        # ----------------------------------------------------
        # CASO 2: AZIONI PRIMITIVE (Foglie dell'albero)
        # ----------------------------------------------------
        else:
            level = 99
            
            # ---> FILTRO DI PROFONDITÀ <---
            if level > max_depth:
                continue
                
            if HIDE_SHOP_PRECONDITIONS and line.startswith("SHOP_"):
                continue

            while stack and stack[-1][1] >= level:
                stack.pop()
                
            parent_id = stack[-1][0]
            
            if line.startswith("SHOP_"):
                color = "#e0e0e0"
                shape = "box"
                label = "Check Precondition"
            else:
                color = "#ffcc99" 
                shape = "ellipse"
                action_name = line.split('[')[0].split('(')[0]
                params = line.split('(')[-1].replace(')', '') if '(' in line else ""
                label = f"{action_name.upper()}\n[{params}]"
                
            dot.node(curr_id, label, fillcolor=color, shape=shape)
            dot.edge(parent_id, curr_id, color="#666666")

    # Rendering
    output_file = f"HTN_Plan_Depth_{max_depth}"
    dot.render(output_file, view=True)
    print(f"Grafico generato con successo: {output_file}.png (Livello massimo: {max_depth})")

if __name__ == "__main__":
    # Inizializza l'argparse per leggere i parametri da terminale
    parser = argparse.ArgumentParser(description="Genera un albero grafico da un trace HTN di PANDA.")
    parser.add_argument(
        '-d', '--max-depth', 
        type=int, 
        default=99, 
        help="Il livello massimo di profondità da stampare (es: 1=MacroTask, 2=Fasi, 99=Primitive)"
    )
    
    args = parser.parse_args()
    build_tree(max_depth=args.max_depth)