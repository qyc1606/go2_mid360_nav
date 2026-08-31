#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <vector>


struct QueueNode
{
  int id;
  double f;

  bool operator<(const QueueNode& other) const
  {
    // std::priority_queue 默认最大堆；
    // 这里反转，使 f 最小的节点优先。
    return f > other.f;
  }
};


class GlobalPlanner
{
public:
  GlobalPlanner()
      : nh_(),
        pnh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        have_map_(false)
  {
    // ============================================================
    // Parameters
    // ============================================================

    pnh_.param<std::string>(
        "map_topic",
        map_topic_,
        "/map_2d");

    pnh_.param<std::string>(
        "goal_topic",
        goal_topic_,
        "/move_base_simple/goal");

    pnh_.param<std::string>(
        "map_frame",
        map_frame_,
        "map");

    pnh_.param<std::string>(
        "base_frame",
        base_frame_,
        "base_link");

    pnh_.param<int>(
        "occupied_threshold",
        occupied_threshold_,
        50);

    pnh_.param<double>(
        "sparse_spacing_m",
        sparse_spacing_m_,
        1.50);

    pnh_.param<double>(
        "start_search_radius_m",
        start_search_radius_m_,
        1.00);

    pnh_.param<double>(
        "goal_search_radius_m",
        goal_search_radius_m_,
        0.50);


    // ============================================================
    // ROS I/O
    // ============================================================

    map_sub_ = nh_.subscribe(
        map_topic_,
        1,
        &GlobalPlanner::mapCallback,
        this);

    goal_sub_ = nh_.subscribe(
        goal_topic_,
        10,
        &GlobalPlanner::goalCallback,
        this);


    path_pub_ = nh_.advertise<nav_msgs::Path>(
        "/global_path",
        1,
        true);

    sparse_pub_ = nh_.advertise<nav_msgs::Path>(
        "/sparse_waypoints",
        1,
        true);


    // ============================================================
    // Startup diagnostics
    // ============================================================

    ROS_INFO(
        "go2_global_planner started");

    ROS_INFO(
        "map_topic=%s",
        map_topic_.c_str());

    ROS_INFO(
        "goal_topic=%s",
        goal_topic_.c_str());

    ROS_INFO(
        "map_frame=%s",
        map_frame_.c_str());

    ROS_INFO(
        "base_frame=%s",
        base_frame_.c_str());

    ROS_INFO(
        "occupied_threshold=%d",
        occupied_threshold_);

    ROS_INFO(
        "sparse_spacing_m=%.3f",
        sparse_spacing_m_);

    ROS_INFO(
        "start_search_radius_m=%.3f",
        start_search_radius_m_);

    ROS_INFO(
        "goal_search_radius_m=%.3f",
        goal_search_radius_m_);
  }


private:

  // ==============================================================
  // Grid utilities
  // ==============================================================

  int index(
      int x,
      int y) const
  {
    return
        y * static_cast<int>(map_.info.width) +
        x;
  }


  bool inside(
      int x,
      int y) const
  {
    return
        x >= 0 &&
        y >= 0 &&
        x < static_cast<int>(map_.info.width) &&
        y < static_cast<int>(map_.info.height);
  }


  int cellValue(
      int x,
      int y) const
  {
    if (!inside(x, y))
    {
      return -999;
    }

    return static_cast<int>(
        map_.data[index(x, y)]);
  }


  bool isFree(
      int x,
      int y) const
  {
    if (!inside(x, y))
    {
      return false;
    }


    const int8_t value =
        map_.data[index(x, y)];


    // UNKNOWN = -1，不允许直接规划。
    if (value < 0)
    {
      return false;
    }


    // 小于 occupied_threshold 才认为可通行。
    return
        value < occupied_threshold_;
  }


  bool worldToGrid(
      double wx,
      double wy,
      int& gx,
      int& gy) const
  {
    if (map_.info.resolution <= 0.0)
    {
      return false;
    }


    gx =
        static_cast<int>(
            std::floor(
                (wx -
                 map_.info.origin.position.x) /
                map_.info.resolution));


    gy =
        static_cast<int>(
            std::floor(
                (wy -
                 map_.info.origin.position.y) /
                map_.info.resolution));


    return inside(gx, gy);
  }


  void gridToWorld(
      int gx,
      int gy,
      double& wx,
      double& wy) const
  {
    wx =
        map_.info.origin.position.x +
        (static_cast<double>(gx) + 0.5) *
        map_.info.resolution;


    wy =
        map_.info.origin.position.y +
        (static_cast<double>(gy) + 0.5) *
        map_.info.resolution;
  }


  double heuristic(
      int x0,
      int y0,
      int x1,
      int y1) const
  {
    const double dx =
        static_cast<double>(x0 - x1);

    const double dy =
        static_cast<double>(y0 - y1);


    return
        std::sqrt(
            dx * dx +
            dy * dy);
  }


  // ==============================================================
  // Search nearest FREE cell
  // ==============================================================

  bool findNearestFree(
      int input_x,
      int input_y,
      double search_radius_m,
      int& output_x,
      int& output_y,
      double& projection_distance_m) const
  {
    projection_distance_m = 0.0;


    // 原位置本身就是 FREE。
    if (isFree(input_x, input_y))
    {
      output_x = input_x;
      output_y = input_y;

      return true;
    }


    if (map_.info.resolution <= 0.0)
    {
      return false;
    }


    const int max_radius_cells =
        std::max(
            1,
            static_cast<int>(
                std::ceil(
                    search_radius_m /
                    map_.info.resolution)));


    bool found = false;


    double best_distance_sq =
        std::numeric_limits<double>::infinity();


    int best_x = input_x;
    int best_y = input_y;


    for (int dx = -max_radius_cells;
         dx <= max_radius_cells;
         ++dx)
    {
      for (int dy = -max_radius_cells;
           dy <= max_radius_cells;
           ++dy)
      {
        const int nx =
            input_x + dx;

        const int ny =
            input_y + dy;


        if (!inside(nx, ny))
        {
          continue;
        }


        if (!isFree(nx, ny))
        {
          continue;
        }


        const double distance_sq_cells =
            static_cast<double>(
                dx * dx +
                dy * dy);


        const double distance_m =
            std::sqrt(distance_sq_cells) *
            map_.info.resolution;


        if (distance_m >
            search_radius_m)
        {
          continue;
        }


        if (distance_sq_cells <
            best_distance_sq)
        {
          best_distance_sq =
              distance_sq_cells;

          best_x = nx;
          best_y = ny;

          found = true;
        }
      }
    }


    if (!found)
    {
      return false;
    }


    output_x = best_x;
    output_y = best_y;


    projection_distance_m =
        std::sqrt(best_distance_sq) *
        map_.info.resolution;


    return true;
  }


  // ==============================================================
  // Map callback
  // ==============================================================

  void mapCallback(
      const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    map_ =
        *msg;


    have_map_ =
        true;


    ROS_INFO(
        "Received map %u x %u, "
        "resolution=%.3f, "
        "origin=(%.3f, %.3f)",
        map_.info.width,
        map_.info.height,
        map_.info.resolution,
        map_.info.origin.position.x,
        map_.info.origin.position.y);
  }


  // ==============================================================
  // Goal callback
  // ==============================================================

  void goalCallback(
      const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    // ------------------------------------------------------------
    // 1. Map check
    // ------------------------------------------------------------

    if (!have_map_)
    {
      ROS_WARN(
          "No /map_2d received yet");

      return;
    }


    // ------------------------------------------------------------
    // 2. Goal frame check
    // ------------------------------------------------------------

    if (!goal->header.frame_id.empty() &&
        goal->header.frame_id != map_frame_)
    {
      ROS_WARN(
          "Goal frame [%s] does not match map frame [%s]",
          goal->header.frame_id.c_str(),
          map_frame_.c_str());

      return;
    }


    // ------------------------------------------------------------
    // 3. Get current robot pose:
    //
    // map -> base_link
    // ------------------------------------------------------------

    geometry_msgs::TransformStamped map_to_base;


    try
    {
      map_to_base =
          tf_buffer_.lookupTransform(
              map_frame_,
              base_frame_,
              ros::Time(0),
              ros::Duration(0.5));
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN(
          "Cannot lookup %s -> %s: %s",
          map_frame_.c_str(),
          base_frame_.c_str(),
          e.what());

      return;
    }


    const double start_world_x =
        map_to_base.transform.translation.x;

    const double start_world_y =
        map_to_base.transform.translation.y;


    const double goal_world_x =
        goal->pose.position.x;

    const double goal_world_y =
        goal->pose.position.y;


    ROS_INFO(
        "Planning request: "
        "start=(%.3f, %.3f), "
        "goal=(%.3f, %.3f)",
        start_world_x,
        start_world_y,
        goal_world_x,
        goal_world_y);


    // ------------------------------------------------------------
    // 4. Convert start and goal to map grid
    // ------------------------------------------------------------

    int start_x = 0;
    int start_y = 0;

    int goal_x = 0;
    int goal_y = 0;


    if (!worldToGrid(
            start_world_x,
            start_world_y,
            start_x,
            start_y))
    {
      ROS_WARN(
          "Start is outside map");

      return;
    }


    if (!worldToGrid(
            goal_world_x,
            goal_world_y,
            goal_x,
            goal_y))
    {
      ROS_WARN(
          "Goal is outside map");

      return;
    }


    ROS_INFO(
        "Raw grid cells: "
        "start=(%d,%d,value=%d), "
        "goal=(%d,%d,value=%d)",
        start_x,
        start_y,
        cellValue(start_x, start_y),
        goal_x,
        goal_y,
        cellValue(goal_x, goal_y));


    // ------------------------------------------------------------
    // 5. Project start to nearest known FREE cell
    // ------------------------------------------------------------

    if (!isFree(
            start_x,
            start_y))
    {
      const int original_start_x =
          start_x;

      const int original_start_y =
          start_y;


      const int original_value =
          cellValue(
              original_start_x,
              original_start_y);


      double projection_distance_m =
          0.0;


      if (!findNearestFree(
              original_start_x,
              original_start_y,
              start_search_radius_m_,
              start_x,
              start_y,
              projection_distance_m))
      {
        ROS_WARN(
            "Start cell value=%d is not free "
            "and no FREE cell was found "
            "within %.3f m",
            original_value,
            start_search_radius_m_);

        return;
      }


      double projected_world_x =
          0.0;

      double projected_world_y =
          0.0;


      gridToWorld(
          start_x,
          start_y,
          projected_world_x,
          projected_world_y);


      ROS_WARN(
          "Start cell value=%d is not free. "
          "Projected start by %.3f m "
          "to FREE cell "
          "grid=(%d,%d), "
          "world=(%.3f,%.3f)",
          original_value,
          projection_distance_m,
          start_x,
          start_y,
          projected_world_x,
          projected_world_y);
    }


    // ------------------------------------------------------------
    // 6. Project goal to nearest known FREE cell
    // ------------------------------------------------------------

    if (!isFree(
            goal_x,
            goal_y))
    {
      const int original_goal_x =
          goal_x;

      const int original_goal_y =
          goal_y;


      const int original_value =
          cellValue(
              original_goal_x,
              original_goal_y);


      double projection_distance_m =
          0.0;


      if (!findNearestFree(
              original_goal_x,
              original_goal_y,
              goal_search_radius_m_,
              goal_x,
              goal_y,
              projection_distance_m))
      {
        ROS_WARN(
            "Goal cell value=%d is not free "
            "and no FREE cell was found "
            "within %.3f m",
            original_value,
            goal_search_radius_m_);

        return;
      }


      double projected_world_x =
          0.0;

      double projected_world_y =
          0.0;


      gridToWorld(
          goal_x,
          goal_y,
          projected_world_x,
          projected_world_y);


      ROS_WARN(
          "Goal cell value=%d is not free. "
          "Projected goal by %.3f m "
          "to FREE cell "
          "grid=(%d,%d), "
          "world=(%.3f,%.3f)",
          original_value,
          projection_distance_m,
          goal_x,
          goal_y,
          projected_world_x,
          projected_world_y);
    }


    // ------------------------------------------------------------
    // 7. A* initialization
    // ------------------------------------------------------------

    const int width =
        static_cast<int>(
            map_.info.width);


    const int height =
        static_cast<int>(
            map_.info.height);


    const int total_cells =
        width * height;


    const double INF =
        std::numeric_limits<double>::infinity();


    std::vector<double> g_score(
        total_cells,
        INF);


    std::vector<int> parent(
        total_cells,
        -1);


    std::vector<uint8_t> closed(
        total_cells,
        0);


    std::priority_queue<QueueNode> open;


    const int start_id =
        index(
            start_x,
            start_y);


    const int goal_id =
        index(
            goal_x,
            goal_y);


    g_score[start_id] =
        0.0;


    open.push(
        {
            start_id,
            heuristic(
                start_x,
                start_y,
                goal_x,
                goal_y)
        });


    // 8-connected grid
    const int dx[8] =
        {
            1,
            1,
            0,
            -1,
            -1,
            -1,
            0,
            1
        };


    const int dy[8] =
        {
            0,
            1,
            1,
            1,
            0,
            -1,
            -1,
            -1
        };


    bool found =
        false;


    // ------------------------------------------------------------
    // 8. A* search
    // ------------------------------------------------------------

    while (!open.empty())
    {
      const QueueNode current =
          open.top();


      open.pop();


      if (closed[current.id])
      {
        continue;
      }


      closed[current.id] =
          1;


      if (current.id ==
          goal_id)
      {
        found =
            true;

        break;
      }


      const int cy =
          current.id /
          width;


      const int cx =
          current.id %
          width;


      for (int k = 0;
           k < 8;
           ++k)
      {
        const int nx =
            cx + dx[k];

        const int ny =
            cy + dy[k];


        if (!isFree(
                nx,
                ny))
        {
          continue;
        }


        // --------------------------------------------------------
        // Diagonal corner-cut prevention
        // --------------------------------------------------------

        if (dx[k] != 0 &&
            dy[k] != 0)
        {
          if (!isFree(
                  cx + dx[k],
                  cy) ||
              !isFree(
                  cx,
                  cy + dy[k]))
          {
            continue;
          }
        }


        const int next_id =
            index(
                nx,
                ny);


        if (closed[next_id])
        {
          continue;
        }


        const double step_cost =
            (dx[k] != 0 &&
             dy[k] != 0)
                ? std::sqrt(2.0)
                : 1.0;


        const double tentative_g =
            g_score[current.id] +
            step_cost;


        if (tentative_g <
            g_score[next_id])
        {
          g_score[next_id] =
              tentative_g;


          parent[next_id] =
              current.id;


          const double f =
              tentative_g +
              heuristic(
                  nx,
                  ny,
                  goal_x,
                  goal_y);


          open.push(
              {
                  next_id,
                  f
              });
        }
      }
    }


    // ------------------------------------------------------------
    // 9. Search failed
    // ------------------------------------------------------------

    if (!found)
    {
      ROS_WARN(
          "A* failed to find a path: "
          "start_grid=(%d,%d), "
          "goal_grid=(%d,%d)",
          start_x,
          start_y,
          goal_x,
          goal_y);

      return;
    }


    // ------------------------------------------------------------
    // 10. Reconstruct path
    // ------------------------------------------------------------

    std::vector<int> cells;


    int current =
        goal_id;


    while (current != -1)
    {
      cells.push_back(
          current);


      if (current ==
          start_id)
      {
        break;
      }


      current =
          parent[current];
    }


    if (cells.empty() ||
        cells.back() !=
            start_id)
    {
      ROS_WARN(
          "A* path reconstruction failed");

      return;
    }


    std::reverse(
        cells.begin(),
        cells.end());


    // ------------------------------------------------------------
    // 11. Build /global_path
    // ------------------------------------------------------------

    nav_msgs::Path global_path;


    global_path.header.stamp =
        ros::Time::now();


    global_path.header.frame_id =
        map_frame_;


    global_path.poses.reserve(
        cells.size());


    for (const int cell_id :
         cells)
    {
      const int gy =
          cell_id /
          width;


      const int gx =
          cell_id %
          width;


      double wx =
          0.0;

      double wy =
          0.0;


      gridToWorld(
          gx,
          gy,
          wx,
          wy);


      geometry_msgs::PoseStamped pose;


      pose.header =
          global_path.header;


      pose.pose.position.x =
          wx;


      pose.pose.position.y =
          wy;


      pose.pose.position.z =
          0.0;


      pose.pose.orientation.x =
          0.0;

      pose.pose.orientation.y =
          0.0;

      pose.pose.orientation.z =
          0.0;

      pose.pose.orientation.w =
          1.0;


      global_path.poses.push_back(
          pose);
    }


    // ------------------------------------------------------------
    // 12. Publish full path
    // ------------------------------------------------------------

    path_pub_.publish(
        global_path);


    // ------------------------------------------------------------
    // 13. Build sparse waypoints
    // ------------------------------------------------------------

    nav_msgs::Path sparse_path;


    sparse_path.header =
        global_path.header;


    size_t stride =
        1;


    if (map_.info.resolution >
        0.0)
    {
      stride =
          std::max<size_t>(
              1,
              static_cast<size_t>(
                  std::round(
                      sparse_spacing_m_ /
                      map_.info.resolution)));
    }


    for (size_t i = 0;
         i < global_path.poses.size();
         i += stride)
    {
      sparse_path.poses.push_back(
          global_path.poses[i]);
    }


    // ------------------------------------------------------------
    // Always include final goal point
    // ------------------------------------------------------------

    if (!global_path.poses.empty())
    {
      if (sparse_path.poses.empty())
      {
        sparse_path.poses.push_back(
            global_path.poses.back());
      }
      else
      {
        const geometry_msgs::Point& p0 =
            sparse_path.poses.back().pose.position;


        const geometry_msgs::Point& p1 =
            global_path.poses.back().pose.position;


        const double distance =
            std::hypot(
                p0.x - p1.x,
                p0.y - p1.y);


        if (distance >
            1e-6)
        {
          sparse_path.poses.push_back(
              global_path.poses.back());
        }
      }
    }


    // ------------------------------------------------------------
    // 14. Publish sparse path
    // ------------------------------------------------------------

    sparse_pub_.publish(
        sparse_path);


    // ------------------------------------------------------------
    // 15. Final diagnostics
    // ------------------------------------------------------------

    ROS_INFO(
        "A* SUCCESS: "
        "path points=%zu, "
        "sparse waypoints=%zu, "
        "start_grid=(%d,%d), "
        "goal_grid=(%d,%d)",
        global_path.poses.size(),
        sparse_path.poses.size(),
        start_x,
        start_y,
        goal_x,
        goal_y);
  }


private:

  // ==============================================================
  // ROS
  // ==============================================================

  ros::NodeHandle nh_;

  ros::NodeHandle pnh_;


  tf2_ros::Buffer tf_buffer_;

  tf2_ros::TransformListener
      tf_listener_;


  ros::Subscriber map_sub_;

  ros::Subscriber goal_sub_;


  ros::Publisher path_pub_;

  ros::Publisher sparse_pub_;


  // ==============================================================
  // Map
  // ==============================================================

  nav_msgs::OccupancyGrid map_;

  bool have_map_;


  // ==============================================================
  // Configuration
  // ==============================================================

  std::string map_topic_;

  std::string goal_topic_;

  std::string map_frame_;

  std::string base_frame_;


  int occupied_threshold_;


  double sparse_spacing_m_;

  double start_search_radius_m_;

  double goal_search_radius_m_;
};


int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_global_planner");


  GlobalPlanner planner;


  ros::spin();


  return 0;
}