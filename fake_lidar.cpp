#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <geometry_msgs/Point32.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <random>

// ROS Publisher
ros::Publisher cloud_pub;
ros::Publisher poly_pub;

// 전역 변수로 장애물 정보 및 바운딩 박스 설정
sensor_msgs::PointCloud global_cloud;
Vec2f origin(-10, -10);  // 바운딩 박스의 시작점 (예: -10, -10)
Vec2f range(20, 20);     // 바운딩 박스의 크기 (예: 20x20m)

// 가짜 LiDAR 데이터 생성 함수
void generate_fake_lidar_data() {
    global_cloud.points.clear();  // 기존 점들을 초기화
    
    // 가짜 LiDAR 데이터를 생성 (예: -10 ~ 10의 범위에서 100개의 랜덤 점 생성)
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-10.0, 10.0);  // -10 ~ 10 범위로 랜덤 생성
    
    for (int i = 0; i < 100; ++i) {
        geometry_msgs::Point32 pt;
        pt.x = distribution(generator);  // x좌표
        pt.y = distribution(generator);  // y좌표
        pt.z = 0;  // z좌표는 0으로 설정 (2D 공간)
        global_cloud.points.push_back(pt);
    }
}

// 장애물과 주행 가능한 영역을 계산하여 시각화
void updateVisualization() {
    if (global_cloud.points.empty()) return;

    // 장애물 데이터를 2D로 변환
    vec_Vec3f obs = DecompROS::cloud_to_vec(global_cloud);
    vec_Vec2f obs2d;
    for (const auto& it : obs)
        obs2d.push_back(it.topRows<2>());

    // Ellipsoid Decomposition 수행
    EllipsoidDecomp2D decomp_util;
    decomp_util.set_obs(obs2d);
    decomp_util.set_local_bbox(Vec2f(2, 2)); // 바운딩 박스를 2x2m로 설정
    decomp_util.dilate(obs2d);  // 장애물을 피하면서 다각형을 생성

    // PolyhedronArray 메시지 생성 및 퍼블리시
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh("~");

    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);  // 장애물 점군 퍼블리시
    poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);  // 주행 가능 영역 퍼블리시

    ros::Rate loop_rate(1.0); // 1Hz 주기로 실행
    while (ros::ok()) {
        generate_fake_lidar_data();  // 가짜 LiDAR 데이터 생성
        
        cloud_pub.publish(global_cloud);  // 생성한 PointCloud 퍼블리시
        
        updateVisualization(); // 시각화 업데이트
        
        loop_rate.sleep(); // 다음 루프까지 대기
    }

    return 0;
}

