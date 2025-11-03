# ROS2 Turtle Catch Game 🐢

A fun and interactive ROS2 project that implements a turtle catching game using the turtlesim simulator. The master turtle hunts and catches other randomly spawned turtles in the environment.

## Project Overview

This project demonstrates various ROS2 concepts including:
- Node creation and communication
- Publisher-Subscriber pattern
- Service-Client architecture
- Custom message and service definitions
- Launch file configuration
- Parameter management
- P-controller implementation for turtle movement

### Components

1. **Turtle Controller** (`turtle_controller.py`)
   - Controls the master turtle's movement
   - Implements P-controller for smooth navigation
   - Catches target turtles when in range
   - Customizable parameters for movement control

2. **Turtle Spawner** (`turtle_spawner.py`)
   - Spawns new turtles at random positions
   - Manages turtle lifecycle
   - Controls random movement of spawned turtles
   - Handles turtle catching service

3. **Turtle Monitor** (`turtle_monitor.py`)
   - Tracks all active turtles in the environment
   - Maintains real-time position updates
   - Publishes list of alive turtles
   - Manages turtle pose subscriptions

4. **Custom Interfaces** (`turtle_interfaces`)
   - Custom message definitions:
     - `Turtle.msg`: Individual turtle information
     - `TurtleArray.msg`: Collection of active turtles
   - Custom service:
     - `CatchTurtle.srv`: Service for catching turtles

## Features

- 🎮 Automated turtle hunting behavior
- 🎯 Closest turtle targeting option
- 🎨 Dynamic pen color changes
- 📊 Configurable game parameters
- 🔄 Continuous turtle spawning
- 🎲 Random turtle movement
- 📍 Position tracking and monitoring

## Getting Started

### Prerequisites

- ROS2 (Tested on Jazzy)
- Python 3.12
- turtlesim package

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd turtlesim_project
   ```

2. Build the packages:
   ```bash
   colcon build
   ```

3. Source the setup files:
   ```bash
   source install/setup.bash
   ```

### Running the Game

Launch the complete game using:
```bash
ros2 launch turtle_bringup turtle_catch.launch.xml
```

## Configuration

The game behavior can be customized through parameters in `turtle_bringup/config/turtle_config.yaml`:

### Turtlesim Parameters
- `background_r/g/b`: Background color (RGB values 0-255)

### Spawner Parameters
- `spawn_frequency`: Time interval between spawning turtles
- `turtle_name_prefix`: Prefix for naming new turtles
- `turtle_pen_off`: Enable/disable pen for spawned turtles

### Controller Parameters
- `catch_closest_turtle_first`: Target selection strategy
- `k_linear`: Linear velocity control constant
- `k_angular`: Angular velocity control constant
- `catch_radius`: Minimum distance for catching turtles
- `master_pen_off`: Enable/disable pen for master turtle
- `master_pen_width`: Pen width for master turtle

## Project Structure
```
turtlesim_project/
├── src/
│   ├── turtle_bringup/         # Launch files and configuration
│   │   ├── launch/
│   │   └── config/
│   ├── turtle_catch/           # Main game logic
│   │   └── turtle_catch/
│   │       ├── turtle_controller.py
│   │       ├── turtle_spawner.py
│   │       └── turtle_monitor.py
│   └── turtle_interfaces/      # Custom messages and services
│       ├── msg/
│       └── srv/
```

## Technical Details

### Control Algorithm
The master turtle uses a P-controller for movement:
- Linear velocity proportional to distance
- Angular velocity proportional to angle error
- Smooth deceleration near target

### Communication Architecture
- Publishers/Subscribers for continuous data flow
- Services for discrete actions (catching)
- Custom messages for specialized data types

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

[Your Name]

## Acknowledgments

- ROS2 Community
- turtlesim package developers