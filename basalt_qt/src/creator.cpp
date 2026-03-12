#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include <QApplication>
#include <QPushButton>
#include <QFont>
#include "interface.cpp"

#include "pthread.h"

using namespace std::chrono_literals;

static rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr p_pose_publisher;

void printCirclePositions(QGraphicsScene* scene){
    qDebug() << "---- Circle Positions ----";

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "world";

    for(auto item : scene->items())
    {
        CircleItem* circle = dynamic_cast<CircleItem*>(item);

        if(circle)
        {
            QPointF p = circle->pos();

            double gx = p.x() / CircleItem::gridSize;
            double gy = p.y() / CircleItem::gridSize;

            qDebug() << "Circle ID:" << circle->id
                     << "X:" << gx
                     << "Y:" << gy
                     << "Z:" << circle->zValueLogical;

	    geometry_msgs::msg::Pose pose;

	    pose.position.x = gx;
	    pose.position.y = gy;
	    pose.position.z = circle->zValueLogical;

	    // Use insert, because even if it is less efficient, order is reversed
	    pose_array.poses.insert(pose_array.poses.begin(), pose);

        }
    }

    p_pose_publisher->publish(pose_array);
}

class FormationCreator : public rclcpp::Node{
  public:
    FormationCreator(): Node("creator_node"){

      // Spawn qt thread
      pthread_t qt_thread;
      pthread_create(&qt_thread, NULL, qtThread, NULL);

      pose_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("formation/definition", 10);
      
      p_pose_publisher = pose_publisher;

      std::cout << "Formation Creator Node has been started." << std::endl;

    }

    static void* qtThread(void* arg){
      (void)arg; 

      std::cout << "Qt thread has been started." << std::endl;

      int   argc_ = 0;
      char *argv_[] = {nullptr};
      QApplication app(argc_, argv_);

      QFont defaultFont("Consolas", 20); 
      app.setFont(defaultFont);

      MainWindow w;
      w.printHandle = printCirclePositions;
      w.show();

      /*
      QPushButton button("Hello World!");
      button.resize(200, 100);
      
      // Add a lambda function to handle button clicks
      QObject::connect(&button, &QPushButton::clicked, []() {
	std::cout << "Button was clicked!" << std::endl;
	std_msgs::msg::String msg;
	msg.data = "Button was clicked!";
	p_string_publisher->publish(msg);
      });

      button.show();

      */
      void* ret = (void*)(intptr_t)app.exec();
      return ret;
    }

  private:

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationCreator>());
  rclcpp::shutdown();
  return 0;
}
