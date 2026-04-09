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

Benchmark on a 14m x 14m map at 0.05m resolution (280x280 = 78,400 cells, 60% occupied), using AMR production parameters (`cluster_max_distance=0.3`, `cluster_min_pts=1`, `cluster_max_pts=10`, `convex_hull_min_pt_separation=0.01`):

| Environment | Full map | Filtered (2m corridor) | Speedup |
|---|---|---|---|
| CI (x86_64) | 38.2 ms (46,993 pts) | 10.6 ms (13,505 pts) | **3.6x** |
| AMR47 (embedded) | 88.4 ms (46,993 pts) | 25.1 ms (13,505 pts) | **3.5x** |

The filter reduces processed obstacle points by 71%, with the largest absolute savings on resource-constrained embedded hardware where the full map scan approaches 90ms per cycle.


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



