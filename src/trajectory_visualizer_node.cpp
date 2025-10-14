#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>

// rclcpp::Nodeを継承したTrajectoryVisualizerクラスを定義
class TrajectoryVisualizer : public rclcpp::Node
{
public:
  // コンストラクタ
  TrajectoryVisualizer() : Node("trajectory_visualizer")
  {
    // Subscriber: /v2x/cpm/trajectoryトピックを購読
    subscription_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "/v2x/cpm/trajectory", 10, std::bind(&TrajectoryVisualizer::trajectoryCallback, this, std::placeholders::_1));
    
    // Publisher: マーカーを/v2x/trajectory_markerトピックとして配信
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/v2x/trajectory_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Trajectory Visualizer Node has been started.");
  }

private:
  // /v2x/cpm/trajectoryトピックを受信したときに呼び出されるコールバック関数
  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // 1. 古いマーカーをすべて削除するための特殊なマーカー
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // 2. 経路を線で描画するためのマーカー (LINE_STRIP)
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = this->now();
    line_strip.ns = "v2x_trajectory_line"; // 名前空間
    line_strip.id = 0; // この名前空間内でユニークなID
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    
    // 線のスタイル設定
    line_strip.scale.x = 0.2; // 線の太さ (メートル単位)
    line_strip.color.r = 0.0f;
    line_strip.color.g = 1.0f; // 緑色
    line_strip.color.b = 0.0f;
    line_strip.color.a = 1.0; // 不透明

    // Trajectoryメッセージ内の全点の座標をマーカーに追加
    for (const auto& point : msg->points) {
      line_strip.points.push_back(point.pose.position);
    }

    // 経路点が1つ以上ある場合のみマーカー配列に追加
    if (!line_strip.points.empty()) {
        marker_array.markers.push_back(line_strip);
    }
    
    // 作成したMarkerArrayを配信
    publisher_->publish(marker_array);
  }

  // メンバ変数としてSubscriberとPublisherを保持
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

// main関数: ノードを初期化して実行
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryVisualizer>());
  rclcpp::shutdown();
  return 0;
}