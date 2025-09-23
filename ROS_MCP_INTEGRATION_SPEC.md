# ROS-MCP-Server Integration Specification

## 🎯 Project Goal

Integrate the existing working PID turtle control system with ros-mcp-server to enable natural language control via LLMs (Claude, ChatGPT, etc.).

## 🏗️ Architecture Overview

```
LLM (Claude/ChatGPT)
    ↕️ (MCP Protocol)
ros-mcp-server (external - unchanged)
    ↕️ (WebSocket + JSON)
rosbridge_server (external - required)
    ↕️ (ROS2 service calls)
NEW: MCP Bridge Service Node ← THIS IS WHAT YOU'LL BUILD
    ↕️ (internal action calls)
Existing PID System (unchanged)
    ↕️
Turtlesim
```

## 📋 Requirements

### What to Build
Create a **ROS2 service node** that acts as a bridge between ros-mcp-server and the existing PID action system.

### Core Functionality
1. **Distance Movement Service**: Move turtle X meters in current heading direction
2. **Position Movement Service**: Move turtle to absolute X,Y coordinates  
3. **Status Query Service**: Get current turtle position and status

### Integration Points
- **Input**: ROS2 services called by ros-mcp-server via rosbridge
- **Output**: Action client calls to existing `/go_to_pose` action server
- **Leverage**: Existing `distance_controller.py` logic for endpoint calculations

## 🛠️ Implementation Details

### 1. Service Message Definitions

Create in `src/cpp_node/srv/`:

#### `TurtleDistanceCommand.srv`
```
# Request
float64 distance        # meters (+ forward, - backward)
float64 max_speed      # optional max speed (default: 1.0)
float64 tolerance      # optional tolerance (default: 0.1)
---
# Response  
bool success
string message
float64 actual_distance
float64 execution_time
```

#### `TurtlePositionCommand.srv`
```
# Request
float64 target_x       # target X coordinate
float64 target_y       # target Y coordinate  
float64 max_speed      # optional max speed (default: 1.0)
float64 tolerance      # optional tolerance (default: 0.1)
---
# Response
bool success
string message  
float64 final_x
float64 final_y
float64 execution_time
```

#### `TurtleStatus.srv`
```
# Request
# (empty)
---
# Response
bool success
float64 current_x
float64 current_y
float64 current_theta
string status_message
```

### 2. MCP Bridge Service Node

Create `src/turtle_demo_controller/turtle_demo_controller/mcp_bridge_service.py`:

#### Key Components
- **Service servers** for the three service types above
- **Action client** to communicate with existing `/go_to_pose` action
- **Pose subscriber** to get current turtle position from `/turtle1/pose`
- **Distance calculation logic** (reuse from `distance_controller.py`)

#### Core Logic Flow

**For Distance Commands:**
1. Get current turtle pose (x, y, θ)
2. Calculate endpoint: `target_x = x + distance × cos(θ)`, `target_y = y + distance × sin(θ)`
3. Apply boundary clamping (0.5 to 10.5 range)
4. Send goal to `/go_to_pose` action
5. Wait for completion and return result

**For Position Commands:**
1. Validate target coordinates (within bounds)
2. Send goal directly to `/go_to_pose` action
3. Wait for completion and return result

**For Status Queries:**
1. Return current pose from `/turtle1/pose` topic
2. Include action server status if available

### 3. Service Names

Use these service names for ros-mcp-server integration:
- `/turtle_distance_command` - Distance movement
- `/turtle_position_command` - Position movement  
- `/turtle_status` - Status query

### 4. CMakeLists.txt Updates

Add to `src/cpp_node/CMakeLists.txt`:
```cmake
# Add service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  "srv/TurtleDistanceCommand.srv"
  "srv/TurtlePositionCommand.srv" 
  "srv/TurtleStatus.srv"
  DEPENDENCIES geometry_msgs
)
```

### 5. Launch File Updates

Update `src/turtle_demo_controller/launch/turtle_with_distance_control.launch.py`:
```python
# Add the MCP bridge service node
Node(
    package='turtle_demo_controller',
    executable='mcp_bridge_service',
    name='mcp_bridge_service',
    output='screen'
)
```

### 6. Setup.py Updates

Add to `src/turtle_demo_controller/setup.py`:
```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'mcp_bridge_service = turtle_demo_controller.mcp_bridge_service:main',
    ],
},
```

## 🧪 Testing Strategy

### 1. Unit Testing
Test each service independently:
```bash
# Test distance command
ros2 service call /turtle_distance_command cpp_node/srv/TurtleDistanceCommand "{distance: 2.0}"

# Test position command  
ros2 service call /turtle_position_command cpp_node/srv/TurtlePositionCommand "{target_x: 3.0, target_y: 7.0}"

# Test status query
ros2 service call /turtle_status cpp_node/srv/TurtleStatus "{}"
```

### 2. Integration Testing
1. Launch the complete system
2. Use ros-mcp-server to send natural language commands
3. Verify commands are translated to service calls correctly

### 3. Expected LLM Commands
The system should handle commands like:
- "Move the turtle 2 meters forward"
- "Move the turtle to position 3, 7"  
- "What is the turtle's current position?"
- "Move the turtle 1.5 meters backward"

## 📁 File Structure After Implementation

```
src/
├── cpp_node/
│   ├── action/
│   │   └── GoToPose.action                    # (existing)
│   ├── srv/
│   │   ├── TurtleDistanceCommand.srv          # (new)
│   │   ├── TurtlePositionCommand.srv          # (new)
│   │   └── TurtleStatus.srv                   # (new)
│   ├── CMakeLists.txt                         # (update)
│   └── package.xml                            # (existing)
└── turtle_demo_controller/
    ├── turtle_demo_controller/
    │   ├── turtle_controller.py               # (existing - PID controller)
    │   ├── distance_controller.py             # (existing - reference for logic)
    │   ├── mcp_bridge_service.py              # (new - main implementation)
    │   └── ...
    ├── launch/
    │   └── turtle_with_distance_control.launch.py  # (update)
    ├── setup.py                               # (update)
    └── ...
```

## 🔧 Implementation Notes

### Error Handling
- Handle action server unavailable gracefully
- Provide meaningful error messages in service responses
- Validate input parameters (bounds checking, reasonable values)

### Performance Considerations
- Reuse action client connection (don't create new client per request)
- Cache current pose to avoid blocking on pose topic
- Set reasonable timeouts for action calls

### Boundary Safety
- Clamp all target coordinates to turtlesim bounds (0.5 to 10.5)
- Reject unreasonable distance values (e.g., > 20 meters)
- Validate speed and tolerance parameters

### Logging
- Use ROS2 logging for debugging
- Log service calls and their outcomes
- Include timing information for performance monitoring

## 🎯 Success Criteria

1. ✅ **Service calls work**: All three services respond correctly
2. ✅ **Action integration works**: Services successfully call existing PID action
3. ✅ **Distance calculation works**: Forward/backward movement in current heading  
4. ✅ **Position movement works**: Direct coordinate targeting
5. ✅ **ros-mcp-server integration**: LLM commands translate to turtle movement
6. ✅ **Reliability**: No hanging or failure issues with consecutive calls
7. ✅ **Maintains existing functionality**: Original action/client system still works

## 📞 Integration with ros-mcp-server

Once implemented, ros-mcp-server will call these services using commands like:

```python
# Distance movement
call_service('/turtle_distance_command', 'cpp_node/srv/TurtleDistanceCommand', {'distance': 2.0})

# Position movement
call_service('/turtle_position_command', 'cpp_node/srv/TurtlePositionCommand', {'target_x': 3.0, 'target_y': 7.0})

# Status query
call_service('/turtle_status', 'cpp_node/srv/TurtleStatus', {})
```

This creates a clean, maintainable bridge between the AI system and your proven turtle control implementation!
