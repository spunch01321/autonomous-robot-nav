# Algorithm Deep Dive

## A* Pathfinding

### Why A*?
- **Optimal**: Guaranteed shortest path
- **Efficient**: Heuristic guides search
- **Complete**: Always finds solution if one exists

### Implementation Details

#### Heuristic Function
```javascript
// Manhattan distance (admissible heuristic)
h(n) = |n.x - goal.x| + |n.y - goal.y|
```

#### Cost Function
```javascript
// Actual cost from start
g(n) = distance_from_start

// Total estimated cost
f(n) = g(n) + h(n)
```

#### Grid Resolution Trade-offs
| Grid Size | Planning Time | Path Quality | Memory |
|-----------|---------------|--------------|---------|
| 10x10     | 2ms          | Poor         | Low     |
| 20x20     | 10ms         | Good         | Medium  |
| 40x40     | 50ms         | Excellent    | High    |

**Chosen: 20x20** - Best balance for real-time performance

### Obstacle Inflation
```javascript
// Safety margin around obstacles
inflated_radius = actual_radius + robot_radius + safety_margin
```

## PID Control

### Control Theory

#### Proportional Term (P)
- Responds to current error
- Higher Kp = faster response, potential overshoot
- Formula: `P = Kp * error`

#### Integral Term (I)
- Eliminates steady-state error
- Higher Ki = eliminates offset, can cause oscillation
- Formula: `I = Ki * Σ(error * dt)`

#### Derivative Term (D)
- Predicts future error
- Higher Kd = more damping, slower response
- Formula: `D = Kd * (error / dt)`

### Tuning Process
1. Start with P-only control
2. Increase Kp until oscillation begins
3. Reduce Kp by 50%
4. Add small D term to reduce overshoot
5. Add tiny I term if steady-state error exists

**Our Values**:

Kp = 2.0  # Responsive without oscillation
Ki = 0.1  # Small correction for drift
Kd = 0.5  # Smooth damping

## Sensor Fusion

### Complementary Filtering
```python
# Combine LIDAR (accurate distance) and Vision (object classification)
def fused_detection(lidar_data, vision_data):
    detections = []
    
    for vision_obj in vision_data:
        # Find closest LIDAR ray
        nearest_ray = find_nearest_ray(lidar_data, vision_obj.angle)
        
        # Fuse measurements
        detection = {
            'position': lidar_position(nearest_ray),
            'type': vision_obj.classification,
            'confidence': combine_confidence(lidar, vision)
        }
        
        detections.append(detection)
    
    return detections
```

### Confidence Weighting
- LIDAR: High distance accuracy, no classification
- Vision: Lower distance accuracy, object classification
- Combined confidence: `0.7 * lidar + 0.3 * vision`

## Collision Detection

### Ray Casting
```python
def check_collision(ray_start, ray_angle, obstacle):
    # Vector from ray to obstacle center
    dx = obstacle.x - ray_start.x
    dy = obstacle.y - ray_start.y
    
    # Perpendicular distance
    distance = abs(dx * sin(ray_angle) - dy * cos(ray_angle))
    
    return distance < obstacle.radius
```

### Continuous Collision Detection
- Sample path at 0.5m intervals
- Check robot footprint at each sample
- Reject paths with any collision

## Optimization Techniques

### Early Termination
```javascript
// Stop planning once "good enough" path found
if (path_cost < threshold * optimal_cost) {
    return path;
}
```

### Path Smoothing
```javascript
// Remove unnecessary waypoints
function smooth_path(path) {
    smoothed = [path[0]];
    
    for (i = 1; i < path.length - 1; i++) {
        if (!is_collinear(path[i-1], path[i], path[i+1])) {
            smoothed.push(path[i]);
        }
    }
    
    smoothed.push(path[path.length - 1]);
    return smoothed;
}
```