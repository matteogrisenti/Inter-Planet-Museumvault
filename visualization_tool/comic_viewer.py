import tkinter as tk
from tkinter import ttk, PhotoImage
import json
import os
from pathlib import Path
from PIL import Image, ImageTk  # Requires: pip install pillow

class ComicBookViewer:
    def __init__(self, root, book_dir: Path):
        self.root = root
        self.root.title("PDDL Comic Book Viewer")
        self.root.geometry("1400x800")
        self.book_dir = book_dir
        
        # Load all step files
        self.steps = sorted([f for f in os.listdir(book_dir) if f.endswith('.json')])
        self.current_step_idx = 0
        self.cache = {} # Store loaded data

        # Styles
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("Treeview", rowheight=25, font=('Consolas', 10))
        style.configure("TLabel", font=('Arial', 11))
        style.configure("Bold.TLabel", font=('Arial', 12, 'bold'))

        self._setup_ui()
        self._load_step(0)

    def _setup_ui(self):
        # MAIN SPLIT (PanedWindow)
        self.paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # === LEFT PANEL (Image & Nav) ===
        self.left_frame = ttk.Frame(self.paned, width=500)
        self.paned.add(self.left_frame, weight=1)

        # Image Container
        self.img_label = ttk.Label(self.left_frame, text="Loading...", anchor="center")
        self.img_label.pack(fill=tk.BOTH, expand=True, pady=10)

        # Navigation Buttons
        btn_frame = ttk.Frame(self.left_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        
        self.btn_prev = ttk.Button(btn_frame, text="<< Previous", command=self._prev_step)
        self.btn_prev.pack(side=tk.LEFT, padx=20, ipadx=10)
        
        self.lbl_step = ttk.Label(btn_frame, text="Step 0 / 0", font=('Arial', 12, 'bold'))
        self.lbl_step.pack(side=tk.LEFT, expand=True)
        
        self.btn_next = ttk.Button(btn_frame, text="Next >>", command=self._next_step)
        self.btn_next.pack(side=tk.RIGHT, padx=20, ipadx=10)

        # === RIGHT PANEL (Data) ===
        self.right_frame = ttk.Frame(self.paned, width=800)
        self.paned.add(self.right_frame, weight=2) # 60% approx logic

        # Notebook (Tabs)
        self.notebook = ttk.Notebook(self.right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # 1. Action Screen
        self.tab_action = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_action, text="  Action Info  ")
        self._build_action_tab(self.tab_action)

        # 2. Fixed Atoms
        self.tab_fixed = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_fixed, text="  Fixed Atoms  ")
        self.tree_fixed = self._build_atom_tree(self.tab_fixed)

        # 3. Initial Atoms (Pre-Action)
        self.tab_initial = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_initial, text="  Initial State  ")
        self.tree_initial = self._build_atom_tree(self.tab_initial)

        # 4. Terminal Atoms (Post-Action)
        self.tab_terminal = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_terminal, text="  Terminal State  ")
        self.tree_terminal = self._build_atom_tree(self.tab_terminal)

    def _build_action_tab(self, parent):
        frame = ttk.Frame(parent, padding=20)
        frame.pack(fill=tk.BOTH, expand=True)

        # Action Name
        ttk.Label(frame, text="Action Name:", style="Bold.TLabel").grid(row=0, column=0, sticky='w', pady=5)
        self.act_name = ttk.Label(frame, text="--", font=('Consolas', 14, 'bold'), foreground="#2E86C1")
        self.act_name.grid(row=0, column=1, sticky='w', pady=5)

        # Parameters
        ttk.Label(frame, text="Parameters:", style="Bold.TLabel").grid(row=1, column=0, sticky='w', pady=5)
        self.act_params = ttk.Label(frame, text="--", font=('Consolas', 12))
        self.act_params.grid(row=1, column=1, sticky='w', pady=5)

        ttk.Separator(frame, orient='horizontal').grid(row=2, column=0, columnspan=2, sticky='ew', pady=15)

        # Effects Grid
        eff_frame = ttk.Frame(frame)
        eff_frame.grid(row=3, column=0, columnspan=2, sticky='nsew')
        
        # Add List
        ttk.Label(eff_frame, text="Effects (+ ADD):", foreground="green", style="Bold.TLabel").pack(anchor='w')
        self.txt_add = tk.Text(eff_frame, height=8, width=50, font=('Consolas', 10))
        self.txt_add.pack(fill=tk.X, pady=(0, 10))

        # Del List
        ttk.Label(eff_frame, text="Effects (- DEL):", foreground="red", style="Bold.TLabel").pack(anchor='w')
        self.txt_del = tk.Text(eff_frame, height=8, width=50, font=('Consolas', 10))
        self.txt_del.pack(fill=tk.X)

    def _build_atom_tree(self, parent):
        # Scrollbar
        tree_frame = ttk.Frame(parent)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        scroll = ttk.Scrollbar(tree_frame)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        tree = ttk.Treeview(tree_frame, yscrollcommand=scroll.set, columns=("Atom"), show="tree")
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.config(command=tree.yview)
        
        # Define Tags for coloring
        tree.tag_configure('robot', foreground='#E67E22')  # Orange
        tree.tag_configure('location', foreground='#2E86C1') # Blue
        tree.tag_configure('item', foreground='#27AE60')    # Green
        
        return tree

    def _categorize_atoms(self, atom_list):
        """Groups atoms for the Pull-down list (Treeview)."""
        groups = {
            "Robots": [],
            "Locations & Map": [],
            "Artifacts & Pods": [],
            "Environment & Other": []
        }
        
        for atom in atom_list:
            pred = atom[0]
            atom_str = f"({pred} {' '.join(atom[1:])})"
            
            if pred in ['robot-at', 'hands-empty', 'carrying', 'sealing-mode', 'can-access', 'carrying-full-pod', 'carrying-empty-pod']:
                groups['Robots'].append(atom_str)
            elif pred in ['connected', 'is-safe', 'is-seismic', 'is-pressurized', 'is-unpressurized', 'is-standard-room', 'is-chill-room']:
                groups['Locations & Map'].append(atom_str)
            elif pred in ['artifact-at', 'pod-at', 'contains-empty-pod', 'contains-full-pod', 'pod-contains', 'fragile', 'no-fragile', 'warm', 'cold', 'is-type']:
                groups['Artifacts & Pods'].append(atom_str)
            else:
                groups['Environment & Other'].append(atom_str)
                
        return groups

    def _update_tree(self, tree, atom_list):
        tree.delete(*tree.get_children())
        groups = self._categorize_atoms(atom_list)
        
        # Insert Groups
        for group_name, items in groups.items():
            if not items: continue
            
            # Determine icon/tag
            tag = 'other'
            if 'Robots' in group_name: tag = 'robot'
            elif 'Locations' in group_name: tag = 'location'
            elif 'Artifacts' in group_name: tag = 'item'
            
            # Parent Node
            parent = tree.insert("", "end", text=f"{group_name} ({len(items)})", open=True)
            
            # Children
            for item in sorted(items):
                tree.insert(parent, "end", text=item, tags=(tag,))

    def _load_step(self, idx):
        if idx < 0 or idx >= len(self.steps): return
        
        self.current_step_idx = idx
        
        # Load JSON
        json_path = self.book_dir / self.steps[idx]
        with open(json_path, 'r') as f:
            data = json.load(f)
            
        # 1. Update Image
        img_path = self.book_dir / data['image_file']
        try:
            # Resize image to fit panel
            pil_img = Image.open(img_path)
            # Basic responsiveness (fixed width for now)
            w_percent = (480 / float(pil_img.size[0]))
            h_size = int((float(pil_img.size[1]) * float(w_percent)))
            pil_img = pil_img.resize((480, h_size), Image.Resampling.LANCZOS)
            
            self.tk_img = ImageTk.PhotoImage(pil_img) # Keep ref
            self.img_label.config(image=self.tk_img, text="")
        except Exception as e:
            self.img_label.config(text=f"Error loading image: {e}")

        # 2. Update Header
        self.lbl_step.config(text=f"Step {idx} / {len(self.steps)-1}")
        
        # 3. Update Action Tab
        act = data['action']
        self.act_name.config(text=act['name'].upper())
        self.act_params.config(text=" ".join(act['parameters']))
        
        self.txt_add.delete(1.0, tk.END)
        for a in act['add']: self.txt_add.insert(tk.END, f"({ ' '.join(a) })\n")
        
        self.txt_del.delete(1.0, tk.END)
        for d in act['del']: self.txt_del.insert(tk.END, f"({ ' '.join(d) })\n")

        # 4. Update Trees
        self._update_tree(self.tree_fixed, data['fixed_atoms'])
        self._update_tree(self.tree_initial, data['initial_atoms'])
        self._update_tree(self.tree_terminal, data['terminal_atoms'])

        # Nav State
        self.btn_prev.state(['disabled'] if idx == 0 else ['!disabled'])
        self.btn_next.state(['disabled'] if idx == len(self.steps)-1 else ['!disabled'])

    def _prev_step(self):
        self._load_step(self.current_step_idx - 1)

    def _next_step(self):
        self._load_step(self.current_step_idx + 1)

if __name__ == "__main__":
    # Standalone launcher
    import sys
    
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        path = Path("viz/comic_book")
        
    if not path.exists():
        print(f"❌ Path not found: {path}")
        print("Run main.py with option 4 first!")
    else:
        root = tk.Tk()
        app = ComicBookViewer(root, path)
        root.mainloop()