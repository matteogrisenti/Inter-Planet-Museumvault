Capisco perfettamente la situazione. Hai completato la parte di modellazione teorica (i file PDDL 2.4) e ora devi "portare in vita" questo dominio all'interno di ROS2 usando **PlanSys2**, come richiesto dal punto 2.5 ("Problem 5") dell'assignment.

L'obiettivo è creare una simulazione in cui il planner (PlanSys2) genera il piano e poi chiama dei nodi ROS2 (codice C++) per eseguire le azioni. Poiché è una simulazione ("fake actions"), questi nodi C++ non muoveranno veri robot, ma semplicemente "fingeranno" di lavorare (aspettando per un po' di tempo e stampando messaggi a video), esattamente come negli esempi `move_action_node.cpp` che hai allegato.

Ecco la guida passo-passo per svolgere il compito basandoci sui file che hai fornito.

### 1. Creazione del Package ROS2

Per prima cosa, devi creare un contenitore (package) per il tuo progetto all'interno del tuo workspace ROS2.

Apri il terminale e vai nella cartella `src` del tuo workspace (es. `ros2_planning_ws/src`):

```bash
ros2 pkg create --build-type ament_cmake my_mars_solution --dependencies rclcpp plansys2_bringup plansys2_msgs plansys2_domain_expert plansys2_executor plansys2_planner plansys2_problem_expert plansys2_pddl_parser

```

*Sostituisci `my_mars_solution` con il nome che preferisci.*

All'interno della cartella creata, dovrai creare queste sottocartelle:

* `pddl/` (dove metterai il file `domain.pddl`)
* `src/` (dove metterai i file `.cpp`)
* `launch/` (dove metterai il file `.py`)

### 2. Il file PDDL (Il Dominio)

Copia il tuo file `domain.pddl` (quello che mi hai mandato come Source 1) nella cartella `pddl/` appena creata.

**Nota importante:** PlanSys2 è molto rigido sulla sintassi. Assicurati che il nome dell'azione nel PDDL (es. `move-to-pressurized-room`) corrisponda esattamente a quello che useremo nei nodi C++.

### 3. I Nodi delle Azioni (C++)

Qui devi creare i "fake action nodes". L'idea è che **per ogni azione definita nel tuo dominio**, deve esistere un nodo ROS2 capace di eseguirla.

Nel tuo dominio hai molte azioni (`move-to-pressurized-room`, `pick-up-slot-1`, `cool-artifact...`, ecc.). Per l'assignment, puoi creare un unico codice C++ generico e lanciarlo più volte con nomi diversi, oppure creare file diversi.

Usiamo come base il file `move_action_node.cpp` che mi hai allegato e creiamo un file generico. Chiamiamolo `src/action_node.cpp`.

```cpp
#include <memory>
#include <algorithm>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class GenericAction : public plansys2::ActionExecutorClient
{
public:
  // Il costruttore prende il nome dell'azione come argomento
  GenericAction(const std::string& action_name, float seconds)
  : plansys2::ActionExecutorClient(action_name, 500ms) // 500ms è il rate di controllo
  {
    total_steps_ = seconds / 0.5; // Calcola quanti cicli fare in base alla durata
    progress_step_ = 1.0 / total_steps_;
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += progress_step_;
      send_feedback(progress_, "Action running");
    } else {
      finish(true, 1.0, "Action completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    // Stampa il nome del nodo per capire quale azione sta girando
    std::cout << this->get_name() << " ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
  float progress_step_;
  int total_steps_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Recuperiamo il nome dell'azione e la durata dai parametri
  // Se non specificati, usiamo valori di default
  std::string action_name = "action_generica";
  
  // Nota: Questo è un trucco semplice per l'assignment. 
  // In un sistema reale useremmo i parametri ROS2.
  if (argc > 1) {
      action_name = argv[1];
  }

  auto node = std::make_shared<GenericAction>(action_name, 5.0); // 5 secondi durata default

  // Questo trigger è fondamentale per attivare il nodo in PlanSys2
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

```

*Copia questo codice in un file `src/action_node.cpp`.*

### 4. Configurare CMakeLists.txt

Devi dire a ROS2 come compilare quel file C++. Apri `CMakeLists.txt` nella root del tuo package e modificalo così:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_mars_solution)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_bringup REQUIRED)
find_package(plansys2_executor REQUIRED)

# Creiamo un eseguibile generico che useremo per tutte le azioni
add_executable(action_executor_node src/action_node.cpp)
ament_target_dependencies(action_executor_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

install(TARGETS
  action_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

### 5. Il Launch File (Python)

Questo è il file che avvia tutto. Deve lanciare l'infrastruttura di PlanSys2 e poi istanziare un nodo per ogni azione definita nel `domain.pddl`.

Crea `launch/plansys2_mars_launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_mars_solution' # Sostituisci con il nome del tuo package
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Avvia PlanSys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': os.path.join(pkg_share, 'pddl', 'domain.pddl')
          }.items())

    # 2. Definisci i nodi per le azioni
    # Devi creare un nodo per OGNI azione durativa definita nel tuo domain.pddl
    # Il campo 'arguments' passa il nome che il nodo userà per registrarsi
    
    move_pressurized_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='move_to_pressurized_room', # Nome univoco del nodo ROS
        arguments=['move-to-pressurized-room'], # Nome esatto dell'azione nel PDDL
        output='screen')

    move_unpressurized_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='move_to_unpressurized_room',
        arguments=['move-to-unpressurized-room'],
        output='screen')
        
    pick_up_cmd = Node(
        package=pkg_name,
        executable='action_executor_node',
        name='pick_up_slot_1',
        arguments=['pick-up-slot-1'],
        output='screen')

    # ... Devi aggiungere qui sotto tutte le altre azioni (drop, cool, etc.) ...
    # activate-seal, pick-up-empty-pod-slot-1, put-in-pod-slot-1, etc.

    ld = LaunchDescription()
    ld.add_action(plansys2_cmd)
    ld.add_action(move_pressurized_cmd)
    ld.add_action(move_unpressurized_cmd)
    ld.add_action(pick_up_cmd)
    # ld.add_action(...) aggiungi gli altri

    return ld

```

### 6. Compilazione ed Esecuzione

1. Compila il workspace:
```bash
cd ~/ros2_planning_ws
colcon build --packages-select my_mars_solution
source install/setup.bash

```


2. Lancia il sistema:
```bash
ros2 launch my_mars_solution plansys2_mars_launch.py

```



### 7. Interazione (Il Terminale PlanSys2)

Ora che il sistema è attivo (vedrai scorrere log), apri un **secondo terminale**. Qui è dove definisci il problema, esattamente come negli esempi di comandi che mi hai incollato.

Avvia la shell di PlanSys2:

```bash
ros2 run plansys2_terminal plansys2_terminal

```

Ora sei dentro la console (>). Qui devi "tradurre" il tuo file `problem.pddl` in comandi.

**Esempio di traduzione per il tuo scenario:**

1. **Istanze (Objects):**
```text
set instance curator robot
set instance entrance location
set instance hall-a location
set instance mart-nord-core-drill artifact
set instance scientific artifact-type

```


*(Devi farlo per tutti gli oggetti del problem.pddl)*
2. **Predicati (Init):**
```text
set predicate (connected entrance hall-a)
set predicate (connected hall-a entrance)
set predicate (robot-at curator entrance)
set predicate (hands-empty-slot-1 curator)
set predicate (artifact-at mart-nord-core-drill hall-a)
set predicate (can-access curator entrance)
set predicate (can-access curator hall-a)
set predicate (is-pressurized entrance)
set predicate (is-safe entrance)

```


*(Continua con tutti i predicati iniziali)*
3. **Goal:**
```text
set goal (and (artifact-at mart-nord-core-drill entrance))

```


*(Imposta un goal semplice per testare, poi metti quello completo)*
4. **Esecuzione:**
```text
get plan
run

```



Se tutto funziona, vedrai nel primo terminale (quello del launch) i nodi C++ stampare "move-to-pressurized-room ... [20%]" ecc., simulando l'esecuzione del piano temporale.

### Riassunto di cosa devi fare per finire l'Assignment

1. Prendi il codice C++ generico che ti ho scritto e mettilo nel package.
2. Nel file Launch Python, aggiungi una riga `Node(...)` per **ogni singola azione** che hai nel file `domain.pddl`. Se ne dimentichi una, PlanSys2 si lamenterà che non trova chi esegue quell'azione.


3. Lancia tutto.
4. Scrivi uno script di testo (o fallo a mano) con tutti i comandi `set instance` e `set predicate` che rispecchiano il tuo `problem.pddl`  per poterli incollare velocemente nel terminale di PlanSys2.


5. Fai gli screenshot dell'esecuzione per il report finale.