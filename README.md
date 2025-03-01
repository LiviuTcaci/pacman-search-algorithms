# pacman-search-algorithms

A Pacman AI agent implementation using various search algorithms (DFS, BFS, UCS, A*) to navigate through mazes, find paths to goals, and collect food efficiently.

## Project Overview

This project implements different search algorithms for Pacman to find optimal paths through maze environments. The implementation is based on the UC Berkeley CS188 Artificial Intelligence course assignment (Project 1: Search).

Original assignment: https://inst.eecs.berkeley.edu/~cs188/fa24/projects/proj1

The main components include:

- **Basic Search Algorithms**: DFS, BFS, UCS, A*
- **Special Search Problems**: Finding all corners, eating all dots
- **Custom Heuristics**: For solving complex pathfinding challenges efficiently

## Search Algorithms Implemented

1. **Depth-First Search (DFS)**: Uses a stack to explore paths as far as possible before backtracking
2. **Breadth-First Search (BFS)**: Uses a queue to find the shortest path in terms of actions
3. **Uniform-Cost Search (UCS)**: Prioritizes paths with lower total cost
4. **A\* Search**: Combines UCS with heuristics to find optimal paths efficiently

## Problem Formulations

- **PositionSearchProblem**: Find path to a specific position
- **CornersProblem**: Find shortest path visiting all four corners
- **FoodSearchProblem**: Find optimal path to collect all food dots
- **AnyFoodSearchProblem**: Find path to closest food dot

## How to Run

### Basic Commands

```bash
# Run DFS on tinyMaze
python pacman.py -l tinyMaze -p SearchAgent

# Run BFS on mediumMaze
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs

# Run UCS on mediumMaze
python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs

# Run A* with Manhattan heuristic on bigMaze
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
```

### Special Agents

```bash
# Find all corners using BFS
python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem

# Find all corners using A* with corners heuristic
python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5

# Eat all food using A* with food heuristic
python pacman.py -l trickySearch -p AStarFoodSearchAgent

# Find closest food dots greedily
python pacman.py -l bigSearch -p ClosestDotSearchAgent -z .5
```

## Project Structure

- **search.py**: Contains implementations of search algorithms
- **searchAgents.py**: Contains search problems and heuristics
- **pacman.py**: Main file for running Pacman games
- **game.py**: Logic for the Pacman world 
- **util.py**: Data structures for implementing search algorithms
- **layouts/**: Directory with various maze configurations

## Heuristics

The project implements several admissible heuristics:

1. **Corners Heuristic**: Estimates cost to visit all remaining corners
2. **Food Heuristic**: Estimates cost to collect all remaining food dots
   - Uses maximum of (distance to farthest food, maximum distance between any two food dots)

## Performance

- **Corners Problem**: A* with corners heuristic expands 1,136 nodes (vs. ~2,000 for BFS)
- **Food Search Problem**: A* with food heuristic expands 8,763 nodes (vs. 16,000+ for UCS)

## Testing

Run the autograder to verify the implementations:

```bash
python autograder.py
```

All tests have been successfully passed across all questions.

## Credits

The Pacman AI projects were developed at UC Berkeley as teaching tools for AI courses. The original assignment can be found at https://inst.eecs.berkeley.edu/~cs188/fa24/projects/proj1.
