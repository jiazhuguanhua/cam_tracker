#include <ros/ros.h>
#include <cam_tracker/Detection.h>
#include <cam_tracker/DetectionArray.h>

void detectionArrayCallback(const cam_tracker::DetectionArray::ConstPtr& msg)
{
    ROS_INFO("收到检测数组，包含 %d 个目标", (int)msg->detections.size());
    
    // 遍历所有检测结果
    for (size_t i = 0; i < msg->detections.size(); i++)
    {
        const cam_tracker::Detection& detection = msg->detections[i];
        ROS_INFO("目标 %zu: ID=%d, 类别=%s, 置信度=%.2f", 
                 i, detection.id, detection.class_name.c_str(), detection.confidence);
        ROS_INFO("  位置: (%.1f, %.1f), 尺寸: %.1f x %.1f", 
                 detection.center_x, detection.center_y, detection.width, detection.height);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_test_cpp");
    ros::NodeHandle nh;
    
    ROS_INFO("C++ 检测消息测试节点启动");
    
    // 订阅检测结果
    ros::Subscriber sub = nh.subscribe("/cam_tracker/detections", 10, detectionArrayCallback);
    
    // 创建发布者测试消息发布
    ros::Publisher pub = nh.advertise<cam_tracker::DetectionArray>("/test_detections", 10);
    
    ros::Rate rate(1.0);  // 1Hz
    
    while (ros::ok())
    {
        // 创建测试消息
        cam_tracker::DetectionArray test_msg;
        test_msg.header.stamp = ros::Time::now();
        test_msg.header.frame_id = "camera";
        test_msg.image_width = 640;
        test_msg.image_height = 480;
        test_msg.total_objects = 2;
        test_msg.processing_time = 0.05;
        
        // 添加第一个检测目标
        cam_tracker::Detection det1;
        det1.header = test_msg.header;
        det1.id = 1;
        det1.class_name = "person";
        det1.confidence = 0.85;
        det1.xyxy = {100, 150, 200, 350};  // x1, y1, x2, y2
        det1.center_x = 150;
        det1.center_y = 250;
        det1.width = 100;
        det1.height = 200;
        
        // 添加第二个检测目标
        cam_tracker::Detection det2;
        det2.header = test_msg.header;
        det2.id = 2;
        det2.class_name = "car";
        det2.confidence = 0.92;
        det2.xyxy = {300, 200, 450, 300};
        det2.center_x = 375;
        det2.center_y = 250;
        det2.width = 150;
        det2.height = 100;
        
        // 将检测结果添加到数组
        test_msg.detections.push_back(det1);
        test_msg.detections.push_back(det2);
        
        // 发布测试消息
        pub.publish(test_msg);
        ROS_INFO("发布了包含 %d 个目标的测试消息", (int)test_msg.detections.size());
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}