# idee estensioni

# robot con permessi
- robot admin che può accedere a tutte le stanze (x1)
- robot tecnico che può accedere a tutte le stanze tranne la stasis-lab (x3)
- robot scientifico che può accedere solo alla stasis-lab e il corridoio (x2)

# il drone
- drone può accedere alla stanza sismica sempre (x1)

-> aggiungere permessi tipo: 
```can-access ?r - robot ?l location```
e questo viene controllato durante il movimento come precondizione

* mettere tanti artefatti (x5 per robot)

# robot possono portare solo certi artefatti
ci sono anche dei permessi sul tipo di artefatto
- can carry ?r - robot ?a - artefact

-> robot manager può portare tutti gli artefatti
-> robot tecnico può portare artefatti di tipo tecnologico
-> robot scientifico può portare artefatti di tipo scientifico

=> gli artefatti devono avere categorie (tecnologico, scientifico, top-secret)

# robot tecnico
può prendere 2 oggetti alla volta

