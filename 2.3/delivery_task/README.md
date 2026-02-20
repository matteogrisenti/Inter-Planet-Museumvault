Execution Commands

1) java -jar PANDA.jar -parser hddl -searchAlgorithm astar -heuristic tdg-m domain.hddl minimal-problem.hddl raw_plan.dot

2) Generation of the hand craft graph: 
   * dot -Tpdf delivery_task_1.dot -o delivery_task_1.pdf
   * dot -Tpdf delivery_task_2.dot -o delivery_task_2.pdf