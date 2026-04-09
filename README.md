costmap_converter ROS Package
=============================

A ros package that includes plugins and nodes to convert occupied costmap2d cells to primitive types.


### Global Plan Filtering

The costmap converter supports filtering obstacle extraction to only process costmap cells near the global plan. On large maps with large occupied areas far from the robot's intended path, this significantly reduces processing time by skipping irrelevant obstacles.

#### How it works

When `plan_filter_distance` is set to a positive value and a global plan is provided, `updateCostmap2D()` only iterates cells within a bounding-box corridor of that radius around each plan waypoint, using a visited bitmask to avoid processing overlapping regions. All downstream processing (DBSCAN clustering, convex hull, polygon simplification) then operates on the reduced point set.

When `plan_filter_distance` is `0.0` (default) or no plan has been set, the full costmap is processed as before — no overhead is added.

#### Configuration

| Parameter | Type | Default | Description |
|---|---|---|---|
| `plan_filter_distance` | double | `0.0` | Only process cells within this distance (meters) of the global plan. `0` = disabled. |
| `global_plan_topic` | string | `/plan` | Topic to subscribe to for the global plan (standalone node only). |

The `plan_filter_distance` parameter is dynamically reconfigurable at runtime.

#### API

Callers that load the converter as a plugin (e.g. TEB local planner) can pass the plan directly:

```cpp
converter->setGlobalPlan(plan.poses);
```

The standalone node subscribes to the configured `global_plan_topic` automatically.

#### Performance

Benchmark on a 14m x 14m map at 0.05m resolution (280x280 = 78,400 cells, 60% occupied):

| Mode | Time per iteration | Obstacle points processed |
|---|---|---|
| Full map (no filter) | 48.6 ms | 46,993 |
| Filtered (2m corridor) | 13.2 ms | 13,505 |
| **Speedup** | **3.7x** | **71% fewer points** |


### Contributors

- Christoph Rösmann
- Franz Albers (*CostmapToDynamicObstacles* plugin)
- Otniel Rinaldo


### License

The *costmap_converter* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *MultitargetTracker*, GNU GPLv3, https://github.com/Smorodov/Multitarget-tracker
   (partially required for the *CostmapToDynamicObstacles* plugin)

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.



