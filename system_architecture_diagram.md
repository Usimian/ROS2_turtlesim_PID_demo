# ROS2 Turtlesim PID Demo - System Architecture

## Block Diagram

```
┌──────────────────────────────────────────────────────────────────────────────────────┐
│                                    HOST SYSTEM                                       │
├──────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌─────────────┐    WebSocket     ┌──────────────────┐    WebSocket    ┌─────────┐   │
│  │     LLM     │◄────────────────►│  ros-mcp-server  │◄───────────────►│         │   │
│  │  (Claude)   │   MCP Protocol   │   (Python)       │   JSON-RPC      │         │   │
│  │             │                  │                  │                 │         │   │
│  └─────────────┘                  │ Tools:           │                 │         │   │
│                                   │ - call_service   │                 │         │   │
│                                   │ - get_services   │                 │         │   │
│                                   │ - get_topics     │                 │         │   │
│                                   └──────────────────┘                 │         │   │
│                                                                        │rosbridge│   │
│  ┌─────────────────────────────────────────────────────────────────────│websocket│   │
│  │                           ROS2 NETWORK                              │  server │   │
│  │                                                                     │         │   │
│  │                                                                     │ - rosapi│   │
│  │  ┌──────────────────┐    ROS2 Services     ┌──────────────────────┐ │  node   │   │
│  │  │mcp_bridge_service│◄────────────────────►│                      │ └─────────┘   │
│  │  │                  │                      │                      │               │
│  │  │ Services:        │                      │                      │               │
│  │  │ /turtle_move_*   │    ROS2 Actions      │ turtle_pid_controller│               │
│  │  │ /turtle_rotate_* │◄────────────────────►│                      │               │
│  │  │ /turtle_status   │                      │ Action Servers:      │               │
│  │  │                  │                      │ - GoToPose           │               │
│  │  │ Queue System:    │                      │ - RotateToPose       │               │
│  │  │ - command_queue  │                      │                      │               │
│  │  │ - sequential     │                      │ PID Controllers:     │               │
│  │  │   execution      │                      │ - Linear (x,y)       │               │
│  │  └──────────────────┘                      │ - Angular (θ)        │               │
│  │                                            └──────────────────────┘               │
│  │                                                       │                           │
│  │                                                   ROS2 Topics                     │
│  │                                                       │                           │
│  │                                                       ▼                           │
│  │  ┌──────────────────────────────────────────────────────────────────────────────┐ │
│  │  │                              turtlesim_node                                  │ │
│  │  │                                                                              │ │
│  │  │  Publishers:                          Subscribers:                           │ │
│  │  │  - /turtle1/pose                      - /turtle1/cmd_vel                     │ │
│  │  │    (turtle position & orientation)      (velocity commands)                  │ │
│  │  │                                                                              │ │
│  │  │  Services:                                                                   │ │
│  │  │  - /turtle1/teleport_absolute                                                │ │
│  │  │  - /turtle1/teleport_relative                                                │ │
│  │  │  - /turtle1/set_pen                                                          │ │
│  │  │                                                                              │ │
│  │  │                          🐢 TURTLE SIMULATION                                │ │
│  │  └──────────────────────────────────────────────────────────────────────────────┘ │
│  └───────────────────────────────────────────────────────────────────────────────────┤
└──────────────────────────────────────────────────────────────────────────────────────┘
```

## Communication Flow

### 1. LLM → Robot Command Flow
```
LLM Command: "Move forward 2 meters"
     │
     ▼ (WebSocket + MCP Protocol)
ros-mcp-server: call_service
     │
     ▼ (WebSocket + JSON-RPC)
rosbridge_websocket: Service Bridge
     │
     ▼ (ROS2 Service Call)
mcp_bridge_service: /turtle_move_relative
     │
     ▼ (Queue + ROS2 Action)
turtle_pid_controller: GoToPose Action
     │
     ▼ (ROS2 Topic)
turtlesim_node: /turtle1/cmd_vel
     │
     ▼
🐢 Turtle moves with PID precision
```

### 2. Robot Status → LLM Flow
```
🐢 Turtle position changes
     │
     ▼ (ROS2 Topic)
turtle_pid_controller: /turtle1/pose subscription
     │
     ▼ (Action Feedback)
mcp_bridge_service: Action feedback
     │
     ▼ (ROS2 Service Response)
rosbridge_websocket: Service response
     │
     ▼ (WebSocket + JSON-RPC)
ros-mcp-server: Service result
     │
     ▼ (WebSocket + MCP Protocol)
LLM: Command completion status
```

## Component Details

### 🧠 LLM (Large Language Model)
- **Type**: Claude/GPT via MCP client
- **Role**: High-level command generation
- **Output**: Natural language → Structured service calls
- **Interface**: MCP Protocol over WebSocket

### 🌐 ros-mcp-server
- **Language**: Python
- **Role**: MCP ↔ ROS2 bridge
- **Key Tools**:
  - `call_service`: Execute ROS2 services
  - `get_services`: List available services
  - `get_topics`: List available topics
- **Interface**: WebSocket to LLM, WebSocket to rosbridge

### 🔌 rosbridge_websocket
- **Package**: `rosbridge_server`
- **Role**: WebSocket ↔ ROS2 bridge
- **Components**:
  - `rosbridge_websocket`: WebSocket server
  - `rosapi_node`: ROS2 introspection
- **Protocol**: JSON-RPC over WebSocket

### 🎯 mcp_bridge_service
- **Language**: Python (rclpy)
- **Role**: Service interface + Command queue
- **Services Provided**:
  - `/turtle_move_relative` (distance)
  - `/turtle_move_position` (absolute position)
  - `/turtle_rotate_relative` (relative angle)
  - `/turtle_rotate_angle` (absolute angle)
  - `/turtle_status` (current pose)
- **Key Feature**: Sequential command queue

### 🎮 turtle_pid_controller
- **Language**: Python (rclpy)
- **Role**: Precision movement control
- **Action Servers**:
  - `GoToPose`: Linear movement with PID
  - `RotateToPose`: Angular movement with PID
- **PID Parameters**:
  - Linear: Kp=6.0, Ki=0.1, Kd=0.5
  - Angular: Kp=3.0, Ki=0.1, Kd=0.5

### 🐢 turtlesim_node
- **Package**: `turtlesim`
- **Role**: Robot simulation
- **Key Topics**:
  - `/turtle1/cmd_vel` (input): Velocity commands
  - `/turtle1/pose` (output): Current position/orientation

## Message Types

### Service Messages
```
cpp_node/srv/TurtleDistanceCommand
├── Request: distance, max_speed, tolerance
└── Response: success, message, actual_distance, execution_time

cpp_node/srv/TurtleRotateRelative  
├── Request: angle, angular_speed, tolerance
└── Response: success, message, final_heading, actual_rotation, execution_time
```

### Action Messages
```
cpp_node/action/GoToPose
├── Goal: desired_x_pos, desired_y_pos
├── Result: success
└── Feedback: current_x_pos, current_y_pos

cpp_node/action/RotateToPose
├── Goal: desired_angle, max_angular_speed, relative
├── Result: success, message, final_angle, angular_distance, execution_time
└── Feedback: current_angle, remaining_angle, angular_velocity
```

### Standard ROS2 Messages
```
geometry_msgs/msg/Twist
├── linear: {x, y, z}
└── angular: {x, y, z}

turtlesim/msg/Pose
├── x, y: position
├── theta: orientation
└── linear_velocity, angular_velocity
```

## Key Features

### 🔄 Command Queue System
- **Purpose**: Prevent overlapping commands
- **Implementation**: FIFO queue in mcp_bridge_service
- **Behavior**: Services return immediately, execute sequentially

### 🎯 PID Control
- **Linear Movement**: Precise position control
- **Angular Movement**: Precise rotation control
- **Feedback**: Real-time pose monitoring

### 🌐 Multi-Protocol Bridge
- **MCP Protocol**: LLM ↔ ros-mcp-server
- **WebSocket/JSON-RPC**: ros-mcp-server ↔ rosbridge
- **ROS2 DDS**: rosbridge ↔ robot nodes

## Startup Sequence

### Terminal 1: Core Simulation
```bash
ros2 run turtlesim turtlesim_node
```

### Terminal 2: Communication Bridge
```bash
cd ~/ROS2_turtlesim_PID_demo
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Terminal 3: Robot Control
```bash
cd ~/ROS2_turtlesim_PID_demo  
source install/setup.bash
ros2 launch turtle_demo_controller turtle_demo_with_mcp.launch.py
```

### Terminal 4: MCP Server
```bash
cd ~/ros-mcp-server
python server.py
```

This architecture provides a robust, scalable system for LLM-controlled robot movement with precision PID control and reliable command sequencing.
