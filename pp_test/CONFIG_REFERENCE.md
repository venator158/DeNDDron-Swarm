# config.yaml Quick Reference

## Minimal Configuration

```yaml
world:
  bounds:
    min: [-20.0, 0.0, -20.0]
    max: [20.0, 20.0, 20.0]

obstacles:
  count: 0
  default_size: [2.0, 2.0, 2.0]

agents:
  default_radius: 0.5
  default_speed: 2.0
  agent_list:
    - id: 0
      start: [0.0, 0.0, 0.0]
      goal: [10.0, 10.0, 0.0]
```

## Field Reference

### world.bounds
- `min`: `[x, y, z]` - Minimum corner of the world (required)
- `max`: `[x, y, z]` - Maximum corner of the world (required)

### obstacles
- `count`: integer - Total number of obstacles (required)
- `default_size`: `[x, y, z]` - Default size for obstacles (required)
- `positions`: list - Explicit obstacle placements (optional)
  - `center`: `[x, y, z]` - Center position (required if in list)
  - `size`: `[x, y, z]` - Size override (optional, uses default_size if omitted)

**Note**: If `count` > number of defined positions, remaining obstacles are randomly generated

### agents
- `default_radius`: float - Default collision radius (required)
- `default_speed`: float - Default movement speed (required)
- `agent_list`: list - List of agents (required, must have at least 1)
  - `id`: integer - Unique agent identifier (required)
  - `start`: `[x, y, z]` - Starting position (required)
  - `goal`: `[x, y, z]` - Goal position (required)
  - `color`: `[r, g, b]` - RGB color, 0.0-1.0 (optional, auto-generated if omitted)
  - `radius`: float - Collision radius override (optional)
  - `speed`: float - Movement speed override (optional)

## Examples

### Two Agents Crossing Paths
```yaml
world:
  bounds:
    min: [-10.0, 0.0, -10.0]
    max: [10.0, 10.0, 10.0]

obstacles:
  count: 1
  default_size: [2.0, 2.0, 2.0]
  positions:
    - center: [0.0, 5.0, 0.0]

agents:
  default_radius: 0.5
  default_speed: 2.0
  agent_list:
    - id: 0
      start: [-5.0, 2.0, 0.0]
      goal: [5.0, 2.0, 0.0]
      color: [1.0, 0.0, 0.0]
    - id: 1
      start: [0.0, 2.0, -5.0]
      goal: [0.0, 2.0, 5.0]
      color: [0.0, 0.0, 1.0]
```

### Large Swarm (10 Agents, Random Obstacles)
```yaml
world:
  bounds:
    min: [-30.0, 0.0, -30.0]
    max: [30.0, 30.0, 30.0]

obstacles:
  count: 20  # All random since positions not specified
  default_size: [3.0, 3.0, 3.0]

agents:
  default_radius: 0.5
  default_speed: 2.5
  agent_list:
    - {id: 0, start: [-20, 2, -20], goal: [20, 20, 20]}
    - {id: 1, start: [20, 2, -20], goal: [-20, 20, 20]}
    - {id: 2, start: [-20, 2, 20], goal: [20, 20, -20]}
    - {id: 3, start: [20, 2, 20], goal: [-20, 20, -20]}
    - {id: 4, start: [0, 2, -20], goal: [0, 20, 20]}
    - {id: 5, start: [0, 2, 20], goal: [0, 20, -20]}
    - {id: 6, start: [-20, 2, 0], goal: [20, 20, 0]}
    - {id: 7, start: [20, 2, 0], goal: [-20, 20, 0]}
    - {id: 8, start: [-10, 2, -10], goal: [10, 20, 10]}
    - {id: 9, start: [10, 2, 10], goal: [-10, 20, -10]}
```

### Dense Obstacle Course
```yaml
world:
  bounds:
    min: [-15.0, 0.0, -15.0]
    max: [15.0, 15.0, 15.0]

obstacles:
  count: 15
  default_size: [2.0, 3.0, 2.0]
  positions:
    # Create a maze-like structure
    - {center: [-5, 3, 0], size: [1, 6, 8]}
    - {center: [5, 3, 0], size: [1, 6, 8]}
    - {center: [0, 3, -5], size: [8, 6, 1]}
    - {center: [0, 3, 5], size: [8, 6, 1]}
    # Remaining 11 obstacles will be randomly placed

agents:
  default_radius: 0.3
  default_speed: 1.5
  agent_list:
    - id: 0
      start: [-10, 2, -10]
      goal: [10, 10, 10]
      color: [1.0, 0.0, 0.0]
    - id: 1
      start: [10, 2, -10]
      goal: [-10, 10, 10]
      color: [0.0, 1.0, 0.0]
    - id: 2
      start: [-10, 2, 10]
      goal: [10, 10, -10]
      color: [0.0, 0.0, 1.0]
```

## Tips

- **Coordinate System**: Y is vertical (up), X and Z are horizontal
- **Units**: All distances are in arbitrary units (typically meters)
- **Colors**: RGB values from 0.0 (none) to 1.0 (full)
- **Spacing**: Keep agents separated by at least 2× their radius initially
- **Obstacles**: Leave clear paths between start and goal when possible
- **Random Generation**: Obstacles avoid agent start/goal positions by 4 units
- **Performance**: Tested with up to 100 agents, more may slow rendering

## Validation

The config loader will fail with an error if:
- Required fields are missing
- Vector3 fields don't have exactly 3 values
- File doesn't exist or has invalid YAML syntax
- No agents are defined
- Bounds min >= max in any dimension

Check console output for detailed error messages!
