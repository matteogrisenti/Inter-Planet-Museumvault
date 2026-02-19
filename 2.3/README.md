java -jar PANDA.jar -parser hddl domain.hddl problem.hddl


Esperimento 1: La Baseline (A* con euristica HTN)
Usa A* abbinato alla tdg-m (Task Decomposition Graph - makespan/max), un'euristica progettata specificamente per guidare i planner gerarchici valutando il costo della scomposizione dei task.

Comando:

Bash

java -jar PANDA.jar -parser hddl -searchAlgorithm astar -heuristic tdg-m domain.hddl 
minimal-problem.hddl
Aspettativa: Trova il piano ottimo, ma potrebbe generare molti nodi (come nel tuo test precedente).

Esperimento 2: L'Approccio Avido (Greedy Search)
La ricerca Greedy punta dritta verso l'obiettivo seguendo l'euristica, ignorando il "costo" speso finora. Spesso abbatte drasticamente i nodi generati in domini HTN.

Comando:

Bash

java -jar PANDA.jar -parser hddl -searchAlgorithm greedy -heuristic tdg-m domain.hddl minimal-problem.hddl

Aspettativa: Tempi di calcolo molto più brevi e crollo del numero di nodi, anche se il piano trovato potrebbe essere di qualche step più lungo (es. qualche movimento robot non perfettamente ottimizzato).

Esperimento 3: Cambio di Euristica (Relax)
Qui testiamo l'euristica relax (basata sul rilassamento dei vincoli, simile alla Fast-Forward) sempre con l'algoritmo Greedy. Questo serve a capire quale euristica "legge" meglio il tuo dominio.

Comando:

Bash

java -jar PANDA.jar -parser hddl -searchAlgorithm greedy -heuristic relax domain.hddl minimal-problem.hddl

Aspettativa: Confronta i nodi esplorati con l'Esperimento 2. Quella che genera meno nodi è l'euristica vincente per la tua architettura.

Esperimento 4: L'Approccio Fulmineo (DFS)
La DFS (Depth-First Search) esplora l'albero in profondità. Siccome il tuo HTN ora ha un caso base perfetto (la stanza vuota) e non ha più loop infiniti, la DFS potrebbe scendere istantaneamente lungo il primo ramo valido senza caricare in memoria migliaia di nodi paralleli.

Comando:

Bash
java -jar PANDA.jar -parser hddl -searchAlgorithm DFS domain.hddl minimal-problem.hddl
Aspettativa: Questa è la candidata numero uno per la velocità estrema. Pochi millisecondi, pochissimi nodi in memoria. Svantaggio: prenderà la prima soluzione che funziona, che potrebbe essere lunga e "disordinata".

Esperimento 5: Il Compromesso per HTN Profondi (Depth-A*)
PANDA offre una variante speciale chiamata depth-astar, che è un ibrido progettato apposta per non saturare la RAM, usando la formula f(n) = profondità + euristica.

Comando:

Bash
java -jar PANDA.jar -parser hddl -searchAlgorithm depth-astar -heuristic tdg-c domain.hddl minimal-problem.hddl
Aspettativa: Un'ottima via di mezzo. Meno RAM saturata rispetto all'A* classico (Exp 1), ma produce piani più intelligenti ed efficienti rispetto alla semplice DFS (Exp 4).



java -jar PANDA.jar -parser hddl -searchAlgorithm greedy -heuristic tdg-m domain.hddl minimal-problem.hddl raw_plan.dot