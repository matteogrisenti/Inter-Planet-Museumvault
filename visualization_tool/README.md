# Inter-Planet Museum Vault - Visualization Tool

This directory contains the visualization engine. It processes raw planner outputs (PDDL `sas_plan` traces, HDDL outputs, and OPTIC textual outputs) into accessible animated maps, policy graphs, and temporal diagrams.

## Usage Requirements
All commands below should be executed from within the `visualization_tool/` directory (unless otherwise specified).

---

## 2.1 Vanilla & Concise Implementations
For the standard sequential plans, you can launch the Map Animator using Mode 1 (Full Animation).

**Command Example:**
```bash
python3 main.py ../2.1/vanilla_implementation/base_case --mode 1
```
*(You can run this similarly for `scaled_case` or any cases in `coincise_implementation`)*

---

## 2.2 Action Costs & Drone Models
These tasks support sequential map animations and state-space policy representations.

**Map Animation Examples:**
```bash
python3 main.py ../2.2/action_cost --mode 1
python3 main.py ../2.2/drone --mode 1
```

**Policy Graph Visualization Example:**
```bash
python3 main.py ../2.2 --mode 6
```
*(This requires graphviz installed to parse the default `.fsap` policy logs)*

---

## 2.3 Hierarchical Task Networks (HTN)
Since section 2.3 utilizes HDDL, its execution graph is parsed via a dedicated script inside the `2.3` directory. This script translates the `raw_plan.dot` into a highly readable logical Task Decomposition Tree.

**Command Example:**
```bash
# Navigate to the 2.3 search folder
cd ../2.3/delivery_search 

# Generate the hierarchical layout graph
python3 parser.py raw_plan.dot tree.dot

# Compile to PDF
dot -Tpdf tree.dot -o HTN_Tree.pdf
```

---

## 2.4 Temporal Planning
Temporal plans are processed using `combined_optic_viz.py`. This distinct tool generates a split-screen layout featuring the map on top and a Gantt-style timeline below to represent overlapping actions accurately.

**Basic Temporal Example:**
```bash
python3 combined_optic_viz.py ../2.4/output_plan6.txt ../2.4/viz/out6.gif ../2.4/problem.pddl
```

**Temporal Drone Example:**
```bash
python3 combined_optic_viz.py ../2.4/drone/output_plan5.txt ../2.4/drone/viz/out5.gif ../2.4/drone/problem.pddl
```
