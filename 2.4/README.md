In questa fase non possiamo usare azioni non-deterministiche, quindi utilizziamo il predicato `is-safe` in maniera leggermente diversa rispetto alla versione 2.1. Invece di essere statico inizialmente, sfruttiamo i Timed Initial Literals per dire quando ci sarà il terremoto e quando no, e quindi `is-safe` sarà vero o falso a seconda del tempo. In questo modo, quando il robot prova ad entrare in una stanza, deve controllare se è sicura o meno, e se non lo è, non può entrarci. Questo ci permette di mantenere la logica di sicurezza senza dover ricorrere a meccaniche non-deterministiche.

Per cui l'azione try-to-enter-sismic-room non serve più dato che basterà utilizzare l'azione move-to-pressurized-room, che ha già come precondizione il fatto che la stanza sia sicura, e se non lo è, semplicemente non potrà essere eseguita e quindi il planner dovrà trovare un'altra strategia (es. aspettare che il terremoto finisca).

Inoltre possiamo togliere lo stato (is-seismic ?l) dato che non è più necessario dato che basta vedere (is-safe ?l) e non ci sono più azioni specifiche se la stanza è sismica o meno.


dato che over all rallenta, le cose che non possono cambiare tipo "upressurized" o "pressurized" e così via sono messe "at start"

alcune prioprietà sono fisse quindi possiamo usare at start.

nell'implementazione base (adattando le actions in durative-actions e facendo le ottimizzazioi sopra riusciamo a risolvere il problema con max 6-7 artefatti).

Ho provato a fare queste ottimizzazioni:
- introdurre fragile (c'era prima ma l'ho tolto per semplificare, ora lo reintroduco per ridurre il branching) nel pickup con pod che obbliga a caricare solo oggetti fragili
- se tiro via sealing mode che si disattiva appena entro e se tiro proprio via la sealing mode disattivabile e parto con essa attiva già dall'inizio il problema ci mette un sacco a risolversi
- provato a rendere specifiche le azioni di pickup per ogni tipo di artefatto, ma non sembra migliorare molto, anzi peggiora un po' (forse perché aumenta il numero di azioni) arriva sempre a max 6 artefatti (come prima, ma ci mette un po' di più per trovare la soluzione)

---

command:
```bash
optic -N -E -W5,1 domain.pddl problem.pddl | tee output_plan.txt 
```