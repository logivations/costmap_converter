#include <random>
#include <memory>
#include <chrono>
#include <gtest/gtest.h>

#include <costmap_converter/costmap_to_polygons.h>

namespace {
geometry_msgs::msg::Point32 create_point(double x, double y)
{
  geometry_msgs::msg::Point32 result;
  result.x = x;
  result.y = y;
  result.z = 0.;
  return result;
}
} // end namespace

// make things accesible in the test
class CostmapToPolygons : public costmap_converter::CostmapToPolygonsDBSMCCH
{
  public:
    const std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>& points() const {return occupied_cells_;}
    costmap_converter::CostmapToPolygonsDBSMCCH::Parameters& parameters() {return parameter_;}
    costmap_converter::CostmapToPolygonsDBSMCCH::Parameters& bufferedParameters() {return parameter_buffered_;}
    using costmap_converter::CostmapToPolygonsDBSMCCH::addPoint;
    using costmap_converter::CostmapToPolygonsDBSMCCH::regionQuery;
    using costmap_converter::CostmapToPolygonsDBSMCCH::dbScan;
    using costmap_converter::CostmapToPolygonsDBSMCCH::convexHull2;
    using costmap_converter::CostmapToPolygonsDBSMCCH::simplifyPolygon;
    using costmap_converter::CostmapToPolygonsDBSMCCH::updateCostmap2D;
};

class CostmapToPolygonsDBSMCCHTest : public ::testing::Test
{
  protected:
    void SetUp() override {
      // parameters
      costmap_to_polygons.parameters().max_distance_ = 0.5;
      costmap_to_polygons.parameters().max_pts_ = 100;
      costmap_to_polygons.parameters().min_pts_ = 2;
      costmap_to_polygons.parameters().min_keypoint_separation_ = 0.1;

      costmap.reset(new nav2_costmap_2d::Costmap2D(100, 100, 0.1, -5., -5.));
      costmap_to_polygons.setCostmap2D(costmap.get());

      std::random_device rand_dev;
      std::mt19937 generator(rand_dev());
      std::uniform_real_distribution<> random_angle(-M_PI, M_PI);
      std::uniform_real_distribution<> random_dist(0.,   costmap_to_polygons.parameters().max_distance_);

      costmap_to_polygons.addPoint(0., 0.);
      costmap_to_polygons.addPoint(1.3, 1.3);

      // adding random points
      double center_x = costmap_to_polygons.points()[0].x;
      double center_y = costmap_to_polygons.points()[0].y;
      for (int i = 0; i < costmap_to_polygons.parameters().max_pts_ - 1; ++i)
      {
        double alpha = random_angle(generator);
        double dist  = random_dist(generator);
        double wx = center_x + std::cos(alpha) * dist;
        double wy = center_y + std::sin(alpha) * dist;
        costmap_to_polygons.addPoint(wx, wy);
      }

      // some noisy points not belonging to a cluster
      costmap_to_polygons.addPoint(-1, -1);
      costmap_to_polygons.addPoint(-2, -2);
      
      // adding random points
      center_x = costmap_to_polygons.points()[1].x;
      center_y = costmap_to_polygons.points()[1].y;
      for (int i = 0; i < costmap_to_polygons.parameters().max_pts_/2; ++i)
      {
        double alpha = random_angle(generator);
        double dist  = random_dist(generator);
        double wx = center_x + std::cos(alpha) * dist;
        double wy = center_y + std::sin(alpha) * dist;
        costmap_to_polygons.addPoint(wx, wy);
      }
    }

    void regionQueryTrivial(int curr_index, std::vector<int>& neighbor_indices)
    {
      neighbor_indices.clear();
      const auto& query_point = costmap_to_polygons.points()[curr_index];
      for (int i = 0; i < int(costmap_to_polygons.points().size()); ++i)
      {
        if (i == curr_index)
          continue;
        const auto& kp = costmap_to_polygons.points()[i];
        double dx = query_point.x - kp.x;
        double dy = query_point.y - kp.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < costmap_to_polygons.parameters().max_distance_)
          neighbor_indices.push_back(i);
      }
    }

    CostmapToPolygons costmap_to_polygons;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
};

TEST_F(CostmapToPolygonsDBSMCCHTest, regionQuery)
{
  std::vector<int> neighbors, neighbors_trivial;
  costmap_to_polygons.regionQuery(0, neighbors);
  regionQueryTrivial(0, neighbors_trivial);
  ASSERT_EQ(costmap_to_polygons.parameters().max_pts_ - 1, int(neighbors.size()));
  ASSERT_EQ(neighbors_trivial.size(), neighbors.size());
  std::sort(neighbors.begin(), neighbors.end());
  ASSERT_EQ(neighbors_trivial, neighbors);

  costmap_to_polygons.regionQuery(1, neighbors);
  regionQueryTrivial(1, neighbors_trivial);
  ASSERT_EQ(costmap_to_polygons.parameters().max_pts_/2, int(neighbors.size()));
  ASSERT_EQ(neighbors_trivial.size(), neighbors.size());
  std::sort(neighbors.begin(), neighbors.end());
  ASSERT_EQ(neighbors_trivial, neighbors);
}

TEST_F(CostmapToPolygonsDBSMCCHTest, dbScan)
{
  std::vector< std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> > clusters;
  costmap_to_polygons.dbScan(clusters);
  
  ASSERT_EQ((unsigned)3, clusters.size());
  ASSERT_EQ((unsigned)2, clusters[0].size()); // noisy points not belonging to a cluster
  ASSERT_EQ((unsigned)costmap_to_polygons.parameters().max_pts_, clusters[1].size()); // first cluster at (0,0)
  ASSERT_EQ((unsigned)costmap_to_polygons.parameters().max_pts_/2 + 1, clusters[2].size()); // second cluster at (1,1)
}

TEST(CostmapToPolygonsDBSMCCH, EmptyMap)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap =
    std::make_shared<nav2_costmap_2d::Costmap2D>(nav2_costmap_2d::Costmap2D(100, 100, 0.1, -5., -5.));
  CostmapToPolygons costmap_to_polygons;
  costmap_to_polygons.setCostmap2D(costmap.get());

  std::vector< std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> > clusters;
  costmap_to_polygons.dbScan(clusters);
  ASSERT_EQ((unsigned)1, clusters.size());    // noise cluster exists
  ASSERT_EQ((unsigned)0, clusters[0].size()); // noise clsuter is empty
}

TEST(CostmapToPolygonsDBSMCCH, SimplifyPolygon)
{
  const double simplification_threshold = 0.1;
  CostmapToPolygons costmap_to_polygons;
  costmap_to_polygons.parameters().min_keypoint_separation_ = simplification_threshold;
  
  geometry_msgs::msg::Polygon polygon;
  polygon.points.push_back(create_point(0., 0.));
  polygon.points.push_back(create_point(1., 0.));

  // degenerate case with just two points
  geometry_msgs::msg::Polygon original_polygon = polygon;
  costmap_to_polygons.simplifyPolygon(polygon);
  ASSERT_EQ((unsigned)2, polygon.points.size());
  for (size_t i = 0; i < polygon.points.size(); ++i)
  {
    ASSERT_FLOAT_EQ(original_polygon.points[i].x, polygon.points[i].x);
    ASSERT_FLOAT_EQ(original_polygon.points[i].y, polygon.points[i].y);  
  }

  // degenerate case with three points
  polygon.points.push_back(create_point(1., simplification_threshold / 2.));
  original_polygon = polygon;
  costmap_to_polygons.simplifyPolygon(polygon);
  ASSERT_EQ((unsigned)3, polygon.points.size());
  for (size_t i = 0; i < polygon.points.size(); ++i)
  {
    ASSERT_FLOAT_EQ(original_polygon.points[i].x, polygon.points[i].x);
    ASSERT_FLOAT_EQ(original_polygon.points[i].y, polygon.points[i].y);  
  }

  polygon.points.push_back(create_point(1., 1.));
  original_polygon = polygon;
  // remove the point that has to be removed by the simplification
  original_polygon.points.erase(original_polygon.points.begin()+2);
  costmap_to_polygons.simplifyPolygon(polygon);
  ASSERT_EQ((unsigned)3, polygon.points.size());
  for (size_t i = 0; i < polygon.points.size(); ++i)
  {
    ASSERT_FLOAT_EQ(original_polygon.points[i].x, polygon.points[i].x);
    ASSERT_FLOAT_EQ(original_polygon.points[i].y, polygon.points[i].y);
  }
}

TEST(CostmapToPolygonsDBSMCCH, SimplifyPolygonPerfectLines)
{
  const double simplification_threshold = 0.1;
  CostmapToPolygons costmap_to_polygons;
  costmap_to_polygons.parameters().min_keypoint_separation_ = simplification_threshold;
  
  geometry_msgs::msg::Polygon polygon;
  for (int i = 0; i <= 100; ++i)
    polygon.points.push_back(create_point(i*1., 0.));
  geometry_msgs::msg::Point32 lastPoint = polygon.points.back();
  for (int i = 0; i <= 100; ++i)
    polygon.points.push_back(create_point(lastPoint.x, lastPoint.y + i * 1.));
  lastPoint = polygon.points.back();
  for (int i = 0; i <= 100; ++i)
    polygon.points.push_back(create_point(lastPoint.x + i * 1., lastPoint.y));
  lastPoint = polygon.points.back();
  for (int i = 0; i <= 100; ++i)
    polygon.points.push_back(create_point(lastPoint.x, lastPoint.y + i * 1.));
  lastPoint = polygon.points.back();
  for (int i = 0; i <= 100; ++i)
    polygon.points.push_back(create_point(lastPoint.x + i * 1., lastPoint.y));

  costmap_to_polygons.simplifyPolygon(polygon);
  ASSERT_EQ((unsigned)6, polygon.points.size());
  ASSERT_FLOAT_EQ(  0., polygon.points[0].x);
  ASSERT_FLOAT_EQ(  0., polygon.points[0].y);
  ASSERT_FLOAT_EQ(100., polygon.points[1].x);
  ASSERT_FLOAT_EQ(  0., polygon.points[1].y);
  ASSERT_FLOAT_EQ(100., polygon.points[2].x);
  ASSERT_FLOAT_EQ(100., polygon.points[2].y);
  ASSERT_FLOAT_EQ(200., polygon.points[3].x);
  ASSERT_FLOAT_EQ(100., polygon.points[3].y);
  ASSERT_FLOAT_EQ(200., polygon.points[4].x);
  ASSERT_FLOAT_EQ(200., polygon.points[4].y);
  ASSERT_FLOAT_EQ(300., polygon.points[5].x);
  ASSERT_FLOAT_EQ(200., polygon.points[5].y);
}

// ===================== Plan Filter Tests =====================

class PlanFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 100x100 cells, 0.1m resolution, origin at (-5,-5) => spans (-5,-5) to (5,5)
    costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, -5., -5.);

    // Area A: cluster near the origin (0,0)
    unsigned int mx, my;
    costmap->worldToMap(0.0, 0.0, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap->setCost(mx + 1, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap->setCost(mx, my + 1, nav2_costmap_2d::LETHAL_OBSTACLE);

    // Area B: cluster far away at (4,4)
    costmap->worldToMap(4.0, 4.0, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap->setCost(mx + 1, my, nav2_costmap_2d::LETHAL_OBSTACLE);

    converter.setCostmap2D(costmap.get());

    // Set reasonable defaults for the parameters
    CostmapToPolygons::Parameters params;
    params.max_distance_ = 0.4;
    params.min_pts_ = 2;
    params.max_pts_ = 30;
    params.min_keypoint_separation_ = 0.1;
    params.plan_filter_distance_ = 0.0;
    converter.parameters() = params;
    converter.bufferedParameters() = params;
  }

  geometry_msgs::msg::PoseStamped makePose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0;
    return ps;
  }

  CostmapToPolygons converter;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
};

TEST_F(PlanFilterTest, DisabledByDefault)
{
  // plan_filter_distance = 0 (default), no plan set
  // All 5 obstacle cells should be found
  converter.updateCostmap2D();
  ASSERT_EQ(5u, converter.points().size());
}

TEST_F(PlanFilterTest, EmptyPlanFallsBackToFullMap)
{
  // Enable filter distance but don't set a plan
  converter.parameters().plan_filter_distance_ = 1.0;
  converter.updateCostmap2D();

  // Should still find all 5 obstacle cells (empty plan => full map)
  ASSERT_EQ(5u, converter.points().size());
}

TEST_F(PlanFilterTest, FilterExcludesDistantObstacles)
{
  // Set plan near origin only
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  plan.push_back(makePose(0.0, 0.0));
  converter.setGlobalPlan(plan);

  // Enable filter: 1m radius around plan points
  converter.parameters().plan_filter_distance_ = 1.0;
  converter.updateCostmap2D();

  // Only Area A's 3 cells should be found; Area B at (4,4) is >1m away
  ASSERT_EQ(3u, converter.points().size());
}

TEST_F(PlanFilterTest, FilterIncludesBothWhenPlanCoversAll)
{
  // Set plan that passes near both areas
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  plan.push_back(makePose(0.0, 0.0));
  plan.push_back(makePose(4.0, 4.0));
  converter.setGlobalPlan(plan);

  // 1m radius covers both clusters
  converter.parameters().plan_filter_distance_ = 1.0;
  converter.updateCostmap2D();

  ASSERT_EQ(5u, converter.points().size());
}

TEST_F(PlanFilterTest, PlanPointsOutsideCostmapAreSkipped)
{
  // Plan point far outside the costmap bounds
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  plan.push_back(makePose(100.0, 100.0));
  converter.setGlobalPlan(plan);

  converter.parameters().plan_filter_distance_ = 1.0;
  converter.updateCostmap2D();

  // No cells in range of the out-of-bounds plan point => 0 obstacles
  ASSERT_EQ(0u, converter.points().size());
}

TEST_F(PlanFilterTest, LargeRadiusCoversEntireMap)
{
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  plan.push_back(makePose(0.0, 0.0));
  converter.setGlobalPlan(plan);

  // Radius large enough to cover entire 10m x 10m map from center
  converter.parameters().plan_filter_distance_ = 20.0;
  converter.updateCostmap2D();

  ASSERT_EQ(5u, converter.points().size());
}

TEST_F(PlanFilterTest, NoDuplicatesFromOverlappingPlanPoints)
{
  // Multiple plan points near the same area should not cause duplicate cells
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  plan.push_back(makePose(0.0, 0.0));
  plan.push_back(makePose(0.05, 0.0));
  plan.push_back(makePose(0.1, 0.0));
  converter.setGlobalPlan(plan);

  converter.parameters().plan_filter_distance_ = 1.0;
  converter.updateCostmap2D();

  // Still exactly 3 cells from Area A, no duplicates
  ASSERT_EQ(3u, converter.points().size());
}

// ===================== Performance Test =====================

TEST(PlanFilterPerf, LargeMap60PercentObstacle)
{
  // 14m x 14m at 0.05m resolution = 280x280 = 78400 cells
  const double width = 14.0;
  const double height = 14.0;
  const double resolution = 0.05;
  const unsigned int cells_x = static_cast<unsigned int>(width / resolution);
  const unsigned int cells_y = static_cast<unsigned int>(height / resolution);
  const double origin_x = 0.0;
  const double origin_y = 0.0;

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
      cells_x, cells_y, resolution, origin_x, origin_y);

  // Fill 60% of cells with LETHAL_OBSTACLE using a deterministic seed
  std::mt19937 rng(42);
  std::uniform_real_distribution<> dist(0.0, 1.0);
  unsigned int obstacle_count = 0;
  for (unsigned int i = 0; i < cells_x; ++i)
  {
    for (unsigned int j = 0; j < cells_y; ++j)
    {
      if (dist(rng) < 0.6)
      {
        costmap->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        ++obstacle_count;
      }
    }
  }

  // Build a global plan as a straight line through the middle of the map
  // from (1, 7) to (13, 7), with poses every 0.1m
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  for (double x = 1.0; x <= 13.0; x += 0.1)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = height / 2.0;
    ps.pose.position.z = 0;
    plan.push_back(ps);
  }

  const int iterations = 20;

  // --- Benchmark WITHOUT plan filter (full map scan) ---
  CostmapToPolygons converter_full;
  converter_full.parameters().max_distance_ = 0.4;
  converter_full.parameters().min_pts_ = 2;
  converter_full.parameters().max_pts_ = 30;
  converter_full.parameters().min_keypoint_separation_ = 0.1;
  converter_full.parameters().plan_filter_distance_ = 0.0;
  converter_full.setCostmap2D(costmap.get());

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int iter = 0; iter < iterations; ++iter)
  {
    converter_full.updateCostmap2D();
    converter_full.compute();
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms_full = std::chrono::duration<double, std::milli>(t1 - t0).count() / iterations;
  size_t points_full = converter_full.points().size();

  // --- Benchmark WITH plan filter (2m corridor around plan) ---
  CostmapToPolygons converter_filtered;
  converter_filtered.parameters().max_distance_ = 0.4;
  converter_filtered.parameters().min_pts_ = 2;
  converter_filtered.parameters().max_pts_ = 30;
  converter_filtered.parameters().min_keypoint_separation_ = 0.1;
  converter_filtered.parameters().plan_filter_distance_ = 2.0;
  converter_filtered.setCostmap2D(costmap.get());
  converter_filtered.setGlobalPlan(plan);

  auto t2 = std::chrono::high_resolution_clock::now();
  for (int iter = 0; iter < iterations; ++iter)
  {
    converter_filtered.updateCostmap2D();
    converter_filtered.compute();
  }
  auto t3 = std::chrono::high_resolution_clock::now();
  double ms_filtered = std::chrono::duration<double, std::milli>(t3 - t2).count() / iterations;
  size_t points_filtered = converter_filtered.points().size();

  double speedup = ms_full / ms_filtered;

  printf("\n");
  printf("=== Plan Filter Performance Test ===\n");
  printf("Map: %ux%u cells (%.0fm x %.0fm @ %.2fm), %.1f%% obstacle (%u cells)\n",
         cells_x, cells_y, width, height, resolution,
         100.0 * obstacle_count / (cells_x * cells_y), obstacle_count);
  printf("Plan: %zu poses, filter distance: 2.0m\n", plan.size());
  printf("Iterations: %d\n", iterations);
  printf("\n");
  printf("  Full map:   %8.2f ms/iter  (%zu obstacle points)\n", ms_full, points_full);
  printf("  Filtered:   %8.2f ms/iter  (%zu obstacle points)\n", ms_filtered, points_filtered);
  printf("  Speedup:    %.1fx\n", speedup);
  printf("====================================\n\n");

  // The filtered version should process fewer points
  EXPECT_LT(points_filtered, points_full);
  // The filtered version should be faster
  EXPECT_LT(ms_filtered, ms_full);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
