extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <string>
#include <assert.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <rerun.hpp>
#include <thread>
#include <pthread.h>
#include <cmath>
#include <time.h>
#include <sys/time.h>
#include <iomanip>
#include "SlamPose.h"
#include "LidarRawObjectArray.h"
#include "Localization.h"
#include <vector>


#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std;

struct road_lane
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
};
struct rerun3D
{
    float x;
    float y;
    float z;
};
// struct Pose2D {
//     double x;            // X坐标（m）
//     double y;            // Y坐标（m）
//     double theta;        // 航向角（弧度）
// };
// Pose2D pose;
CurPose_h pose;

road_lane jsonToLaneLine(const json& j) 
{
    road_lane line;
    line.x = j.at("x").get<std::vector<double>>();
    line.y = j.at("y").get<std::vector<double>>();
    line.s = j.at("s").get<std::vector<double>>();
    return line;
}

std::vector<rerun::Position3D> convertToPosition3D(const road_lane& lane) {
    std::vector<rerun::Position3D> positions;
    size_t size = std::min(lane.x.size(), lane.y.size());

    positions.reserve(size);  // 提前分配内存提升性能

    for (size_t i = 0; i < size; ++i) {
        rerun3D pos;
        pos.x = static_cast<float>(lane.x[i]);
        pos.y = static_cast<float>(lane.y[i]);
        pos.z = -2.0f; 
        positions.emplace_back(pos.x, pos.y, pos.z);
    }

    return positions;
}

bool read_map(const string globalmap_pcd, rerun::RecordingStream& rec)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(globalmap_pcd, *map_cloud);

    std::vector<rerun::Position3D> map_points;
    std::vector<rerun::Color> map_colors;  

    for (const auto& pt : map_cloud->points) {
        map_points.emplace_back(pt.x, pt.y, pt.z);
        map_colors.emplace_back(255, 0, 0);
    }

    rec.log("map_points_static", rerun::Points3D(map_points).with_colors(map_colors).with_radii({0.02f}));
    std::cout << "Loaded " << map_cloud->points.size()<< " map to Rerun." << std::endl;

    return true;
}

bool clouds2rerun(const char *bytes, int32_t size, rerun::RecordingStream& rec)
{
    if (size <= 0)
    {
        std::cerr << "Error: Point cloud size <= 0!" << std::endl;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr row_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    row_cloud->header.seq = *(std::uint32_t *)bytes;
    row_cloud->header.stamp = *(std::uint64_t *)(bytes + 8);
    row_cloud->header.frame_id = "rslidar";
    row_cloud->width = size;
    row_cloud->height = 1;
    row_cloud->is_dense = true;
    std::vector<rerun::Position3D> rerun_points;
    std::vector<rerun::Color> colors;
    for (size_t i = 0; i < size; i++) {
        pcl::PointXYZI tem_point;
        tem_point.x = *(float *)(bytes + 16 + 16 * i);
        tem_point.y = *(float *)(bytes + 16 + 4 + 16 * i);
        tem_point.z = *(float *)(bytes + 16 + 8 + 16 * i);
        tem_point.intensity = *(float *)(bytes + 16 + 12 + 16 * i);
        rerun_points.emplace_back(tem_point.x, tem_point.y, tem_point.z);
    }
    rec.log("live_points", rerun::Points3D(rerun_points).with_colors(0x00FF00FF).with_radii({0.02f}));

    rec.log(
        "live_points",
        rerun::Transform3D::from_translation_rotation(
            {pose.x, pose.y, 0.0f},
            rerun::RotationAxisAngle{
                {0.0f, 0.0f, 1.0f},
                rerun::Angle::degrees(pose.theta),
            }
        )
    );
    // std::cout << "Logging " << rerun_points.size() << " points to Rerun." << std::endl;

    return true;
}


void path2rerun(char *msg, size_t data_len, rerun::RecordingStream& rec)
{

    int num_points = data_len / sizeof(float); 
    float* float_array = reinterpret_cast<float*>(msg); 

    int num_xy_points = num_points / 2; 

    std::vector<float> x_v(float_array, float_array + num_xy_points);
    std::vector<float> y_v(float_array + num_xy_points, float_array + num_points); 

    vector<rerun::Position3D> points;
    points.reserve(num_xy_points);
    for (int i = 0; i < num_xy_points; ++i)
    {
        points.emplace_back(x_v[i], y_v[i], 0.0);
    }
    std::vector<rerun::LineStrip3D> line_strips = {rerun::LineStrip3D(points)};
    // std::cout << "x: " << pose.x << " Y ; " << pose.y << std::endl;
    rec.log("path_points", rerun::LineStrips3D(line_strips).with_colors(0x0000FFFF).with_radii({0.08f}));
    rec.log(
        "path_points",
        rerun::Transform3D::from_translation_rotation(
            {pose.x, pose.y, -1.0f},
            rerun::RotationAxisAngle{
                {0.0f, 0.0f, 1.0f},
                // rerun::Angle::degrees(pose.theta - 90.0f),//车辆坐标系的正前方是y坐标，为了统一到全局，需要逆时针旋转90°
                rerun::Angle::degrees(pose.theta - 90.0f),//车辆坐标系的正前方是y坐标，为了统一到全局，需要逆时针旋转90°
            }
        )
    );

    return;
}


void pose2rerun(char *msg, size_t len, rerun::RecordingStream& rec)
{

    CurPose_h *cur_pose = reinterpret_cast<CurPose_h *>(msg);
    // Pose2D *cur_pose = reinterpret_cast<Pose2D *>(msg);
    pose.x = cur_pose->x;
    pose.y = cur_pose->y;
    pose.theta = cur_pose->theta;
    pose.s = cur_pose->s;

    rec.log(
    "vehicle/position_log",
    rerun::TextLog("x: " + std::to_string(pose.x) + "  y: " + std::to_string(pose.y) + " theta: " +std::to_string(pose.theta) + " s: " + std::to_string(pose.s))
        .with_level(rerun::TextLogLevel::Info)
    );


    std::vector<rerun::Position3D> origins;
    std::vector<rerun::Vector3D> vectors;

    // double theta_pose = pose.theta;  
    float angle = pose.theta * (M_PI / 180.0f);  
    float length = 1.5f;  
 
    origins.push_back({pose.x, pose.y, -1.0});
    vectors.push_back({length * cosf(angle), length * sinf(angle), 0.0});

    rec.log(
        "arrows",
        rerun::Arrows3D::from_vectors(vectors).with_origins(origins).with_colors(0xFF00FFFF)
    );


    return ;
}

void road2rerun(char *msg, size_t data_len, rerun::RecordingStream& rec)
{
    std::string data_str(msg, data_len);
    json j_array  = json::parse(data_str);         
    road_lane refer_road_lane = jsonToLaneLine(j_array.at(0));
    auto refer_path = convertToPosition3D(refer_road_lane);
    if(refer_path.empty()) 
    {
        cerr << "can not read refer_path!!!" << endl;
        return ;
    }
    std::vector<rerun::LineStrip3D> refer_line_strips = {rerun::LineStrip3D(refer_path)};
    rec.log("refer_path", rerun::LineStrips3D(refer_line_strips).with_colors(0x00FFFFFF).with_radii({0.08f}));

    road_lane left_road_lane = jsonToLaneLine(j_array.at(1));
    auto left_path = convertToPosition3D(left_road_lane);
    if(left_path.empty()) 
    {
        cerr << "can not read left_path!!!" << endl;
        return ;
    }
    std::vector<rerun::LineStrip3D> left_line_strips = {rerun::LineStrip3D(left_path)};
    rec.log("left_path", rerun::LineStrips3D(left_line_strips).with_colors(0xF0D249FF).with_radii({0.08f}));

    road_lane right_road_lane = jsonToLaneLine(j_array.at(2));
    auto right_path = convertToPosition3D(right_road_lane);
    if(left_path.empty()) 
    {
        cerr << "can not read right_path!!!" << endl;
        return ;
    }
    std::vector<rerun::LineStrip3D> right_line_strips = {rerun::LineStrip3D(right_path)};
    rec.log("right_path", rerun::LineStrips3D(right_line_strips).with_colors(0xF0D249FF).with_radii({0.08f}));
    return ;
}


void object2rerun(char *data, size_t data_len, rerun::RecordingStream& rec)
{
    std::string data_str(data, data_len);
    json j = json::parse(data_str);
    vector<rerun::Vec3D> centers;
    std::vector<rerun::Vec3D> half_sizes;
    LidarRawObjectArray_h object_data;
    LidarRawObject_h lidar_object;
    for (const auto& obj : j["LidarRawObjects"]["objs"]) 
    {
        for (int i = 0; i < obj["bbox_point"]["x"].size(); ++i) 
        {
            lidar_object.bbox_point[i].x = obj["bbox_point"]["x"][i];
            lidar_object.bbox_point[i].y = obj["bbox_point"]["y"][i];
            lidar_object.bbox_point[i].z = obj["bbox_point"]["z"][i];
        }

        lidar_object.lwh.x() = obj["lwh"]["x"];
        lidar_object.lwh.y() = obj["lwh"]["y"];
        lidar_object.lwh.z() = obj["lwh"]["z"];

        lidar_object.x_pos = obj["x_pos"];
        lidar_object.y_pos = obj["y_pos"];
        lidar_object.z_pos = obj["z_pos"];
        // std::cout << "lidar_object.x_pos: " << lidar_object.x_pos << std::endl;
        object_data.objs.push_back(lidar_object);
    }
    // std::cout << "rerun size : " << object_data.objs.size() << std::endl;
    for (const auto &var : object_data.objs) 
    {
        centers.emplace_back(var.x_pos, var.y_pos, var.z_pos);

        half_sizes.emplace_back(var.lwh.x()/2.0f, var.lwh.y()/2.0f,var.lwh.z()/2.0f);
    }
    float theta_rad = pose.theta * M_PI / 180.0f;
    rerun::Quaternion rot_quat = rerun::Quaternion::from_xyzw(
        0.0f, 0.0f,
        std::sin(theta_rad / 2.0f),
        std::cos(theta_rad / 2.0f)
    );
    std::vector<rerun::Quaternion> quaternions(centers.size(), rot_quat);

    rec.log(
    "lidar/clusters",
    rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes)
        .with_quaternions(quaternions)
        .with_fill_mode(rerun::FillMode::MajorWireframe)
    );
    return ;
}


int run(void *dora_context, rerun::RecordingStream& rec)
{
    while (true)
    {
        void * event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            if (strncmp("pointcloud", data_id, 10) == 0)
            {
                //----------------------------------------------------------------------------------------------------
                    // struct timeval tv;
                    // gettimeofday(&tv, NULL);//获取时间
                int32_t point_len = (data_len - 16) / 16;          
                // auto clouds = bytes2cloud(data, point_len);
                // if (clouds == NULL)
                // {
                //     std::cerr << "Error: Failed to rec point cloud!" << std::endl;
                // }

                // auto trans_clouds = rslidar2baselink(clouds);

                // auto res = points2rerun(trans_clouds, rec);
                // if (!res)
                // {
                //     std::cerr << "Error: Failed to send point cloud to Rerun!" << std::endl;
                // }
                    // struct timeval tv_1;
                    // gettimeofday(&tv_1, NULL);//获取时间
                    // auto start = tv.tv_sec + tv.tv_usec * 1e-6;
                    // auto end = tv_1.tv_sec + tv_1.tv_usec * 1e-6;
                    // auto all_time = end - start;
                    // std::cout << "Time: " << all_time << std::endl;
                //----------------------------------------------------------------------------------------------------
                auto clouds = clouds2rerun(data, point_len, rec);
                if (!clouds)
                {
                    std::cerr << "Error: Failed to rec point cloud!" << std::endl;
                }
            }
            else if (strncmp("raw_path", data_id, 8) == 0)
            {
                path2rerun(data, data_len, rec);  
            }
            else if (strncmp("cur_pose_all", data_id, 12) == 0)
            {
                pose2rerun(data, data_len, rec);
            }
            else if (strncmp("road_lane", data_id, 9) == 0)
            {
                road2rerun(data, data_len, rec);
            }
            else if (strncmp("Filter_Object", data_id, 13) == 0)
            {
                object2rerun(data, data_len, rec);
            }

        }    
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }                
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);    
    }

    return 0;
}

int main()
{
    std::cout << "rerun_test" << std::endl;
    void* dora_context = init_dora_context_from_env();

    rerun::RecordingStream rec("lidarpoints_viewer");
    rec.spawn().exit_on_failure();

    // std::string globalmap_pcd ="./data/map.pcd";
    // auto map_load = read_map(globalmap_pcd, rec);
    // if (!map_load)
    // {
    //     std::cerr << "Error: Failed to open map!" << std::endl;
    // }

    // rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP); // Set an up-axis
    // rec.log(
    //     "world/xyz",
    //     rerun::Arrows3D::from_vectors({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}
    //     ).with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}})
    // );

    auto ret = run(dora_context, rec);
    free_dora_context(dora_context);
    std::cout << "END rerun_test" << std::endl;
    return ret;
}