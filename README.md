# AI Decision Lab

[![Deploy GitHub Pages](https://github.com/systemslibrarian/ai-decision-lab/actions/workflows/deploy-pages.yml/badge.svg)](https://github.com/systemslibrarian/ai-decision-lab/actions/workflows/deploy-pages.yml)

[Live Site](https://systemslibrarian.github.io/ai-decision-lab/)

**Visualizing Search, Optimization, and Robot Motion Planning**

AI Decision Lab is an interactive browser-based simulator that demonstrates how pathfinding, optimization, and motion planning work together in robotics and AI systems.

It combines:

- **DFS** for deep uninformed exploration
- **BFS** for shortest-path search on unweighted grids
- **Weighted A\*** for informed path planning with terrain costs
- **Path smoothing** for motion optimization
- **Robot animation** for path execution
- **Gradient-style descent visualization** for continuous optimization concepts

## Why this project exists

Modern intelligent systems often combine:

- **Discrete search** for planning
- **Continuous optimization** for refinement
- **Motion constraints** for realistic execution

This project makes those ideas visible and interactive.

## Features

- Interactive grid editor
- Place start and goal nodes
- Draw and erase walls
- Paint weighted terrain
- Run DFS, BFS, and Weighted A*
- Optimize paths with smoothing
- Animate robot traversal
- Compare search with gradient-style optimization
- Split-screen visualization
- Keyboard shortcuts
- Live replanning toggle

## Concepts illustrated

### Search
- DFS explores deeply without guaranteeing the best route
- BFS finds shortest paths in unweighted grids
- A* uses path cost + heuristic guidance
- Weighted A* models more realistic terrain-aware navigation

### Optimization
- Path smoothing reduces jagged movement
- Turn penalties model motion constraints
- Gradient descent visualization shows iterative improvement on a continuous surface

## Tech stack

- HTML
- CSS
- Vanilla JavaScript
- Canvas API

No frameworks, no build step, no backend.

## Run locally

Open `index.html` in your browser.

Or use a simple local server:

```bash
python -m http.server 8000
```

Then visit:

```text
http://localhost:8000
```

## Controls

### Mouse
- Place start
- Place goal
- Draw walls
- Erase cells
- Paint terrain cost

### Buttons
- Run A* Search
- Run BFS
- Run DFS
- Optimize Route
- Animate Robot
- Run Gradient
- Random Obstacles
- Clear Path
- Clear Grid
- Compare A* vs DFS
- Live Replan

### Keyboard shortcuts
- `1` Place Start
- `2` Place Goal
- `3` Place Walls
- `4` Erase
- `5` Paint Weight
- `A` Run A*
- `B` Run BFS
- `D` Run DFS
- `O` Optimize Path
- `R` Animate Robot
- `G` Run Gradient

## Project structure

```text
ai-decision-lab/
├── index.html
├── styles.css
├── script.js
├── README.md
├── LICENSE
├── .gitignore
├── assets/
│   └── favicon.svg
└── docs/
    └── concept-notes.md
```

## Deployment

This project deploys easily to:

- GitHub Pages
- Netlify
- Vercel
- any static host

## Portfolio value

This project demonstrates:
- algorithm visualization
- UI/UX for technical education
- robotics-relevant thinking
- AI/optimization concepts
- clean browser-based implementation

## Future upgrades

- moving obstacles
- preset warehouse / maze maps
- chart of cost over optimization iterations
- mobile-first toolbar layout
- explanation overlays for interviews and demos

## License

MIT
