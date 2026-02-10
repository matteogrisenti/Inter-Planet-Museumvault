# idee estensioni

# robot con permessi ✅
- robot admin che può accedere a tutte le stanze (x1)
- robot tecnico che può accedere a tutte le stanze tranne la stasis-lab (x2)
- robot scientifico che può accedere solo alla stasis-lab e il corridoio (x1)

# il drone ✅
- drone viene usato per accedere a stanze pericolose (sismiche) può portare solo oggetti non-fragili (x1)
non ha problemi di depressurizzazione

-> aggiungere permessi tipo: 
```can-access ?r - robot ?l location```
e questo viene controllato durante il movimento come precondizione

* mettere tanti artefatti (x5 per robot)

# robot possono portare solo certi artefatti
ci sono anche dei permessi sul tipo di artefatto
- can carry ?r - robot ?a - artefact

-> robot admin può portare tutti gli artefatti
-> robot tecnico può portare artefatti di tipo tecnologico
-> robot scientifico può portare artefatti di tipo scientifico

=> gli artefatti devono avere categorie (tecnologico, scientifico, top-secret)

# robot tecnico
può prendere 2 oggetti alla volta, il secondo non deve essere fragile
(però direi che può portare solo 1 pod alla volta. Non avrebbe molto senso che uno ne porti 2 insieme) Utile per portare un oggetto fragile nel pod + uno non fragile nel secondo slot

# Osservazioni e Variazioni rispetto alla versione 2.1

## 1. Introduzione di Ruoli Specializzati (Multi-Robot)
Mentre la versione 2.1 utilizzava un singolo robot generico, la versione 2.2 introduce un'architettura multi-agente con ruoli specializzati. Le definizioni dei problemi ora includono robot di tipo:
- **Admin**: Dotato di privilegi elevati per l'accesso e la manipolazione di artefatti critici.
- **Tecnico**: Progettato per il supporto logistico pesante.
- **Scienziato**: Focalizzato sull'interazione con il laboratorio e artefatti scientifici.

Questa variazione richiede al pianificatore di assegnare i task non solo in base alla posizione, ma anche alle *capacità intrinseche* dell'agente (predicati `can-access` e `can-pickup`).

## 2. Capacità di Carico Avanzata (Tecnico)
È stata introdotta una meccanica specifica per il robot Tecnico:
- **Doppio Slot**: Possibilità di trasportare due oggetti contemporaneamente (`can-carry-two`).
- **Vincolo di Fragilità**: Il secondo slot è limitato a oggetti non fragili, imponendo una strategia di carico mista (es. 1 Pod con oggetto fragile + 1 oggetto robusto extra).

## 3. Supporto Droni
Il dominio 2.2 integra i **Droni** come entità distinte:
- **Accesso Aree a Rischio**: I droni sono pensati per operare in aree sismiche o pericolose dove i robot potrebbero subire danni.
- **Mobilità Aerea**: Possono avere regole di connessione differenti (es. volare sopra ostacoli o accedere direttamente a zone specifiche).
- **Limitazioni**: Hanno capacità di carico ridotta (spesso 1 oggetto, no pod pesanti).

## 4. Nuova Logica di Pressurizzazione e Sicurezza
Una differenza critica riguarda la gestione della modalità di sigillatura (`sealing-mode`):
- **Automatismo (Miglioramento)**: Nella versione 2.2 rivista, l'ingresso in una stanza pressurizzata (`move-to-pressurized-room`) comporta la **disattivazione automatica** della modalità di sigillatura.  
- **Vantaggio**: Questo risolve un problema logico presente nelle versioni precedenti dove il robot era costretto a "togliere il sigillo" (`deactivate-seal`) mentre si trovava ancora nel vuoto del tunnel per poter soddisfare le precondizioni di ingresso. La nuova logica assume che la transizione (airlock) gestisca la pressurizzazione, permettendo al robot di entrare in sicurezza e trovarsi poi desigillato all'interno.

## Assunzioni del Modello per il Report
1.  **Airlock Impliciti**: Si assume che ogni connessione tra una zona `unpressurized` (Tunnel) e una `pressurized` (Hall) sia dotata di un sistema di airlock automatico che permette il passaggio sicuro senza esporre gli ambienti interni al vuoto.
2.  **Resilienza dei Droni**: I droni sono considerati immuni agli effetti di depressurizzazione o sono intrinsecamente progettati per il vuoto, non necessitando della meccanica `sealing-mode` dei robot antropomorfi.
3.  **Specializzazione dell'Equipaggiamento**: Le restrizioni di accesso (`can-access`) non sono solo fisiche (chiavi) ma possono rappresentare protocolli di sicurezza o rischi ambientali specifici per quel tipo di hardware (es. radiazioni nello Stasis Lab che solo lo Scienziato può gestire). 