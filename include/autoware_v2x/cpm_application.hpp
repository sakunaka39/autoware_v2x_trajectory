#ifndef CPM_APPLICATION_HPP_EUIC2VFR
#define CPM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_v2x/positioning.hpp"
#include <vanetza/asn1/cpm.hpp>
#include <optional>
#include <mutex>

namespace v2x_trajectory
{
    class V2XNode;
    class CpmApplication : public Application
    {
    public:
        CpmApplication(V2XNode *node, vanetza::Runtime &, bool is_sender);
        PortType port() override;
        void indicate(const DataIndication &, UpPacketPtr) override;
        void set_interval(vanetza::Clock::duration);
        
        // 自車位置や向きを更新するための関数
        void updateMGRS(double *, double *);
        void updateRP(double *, double *, double *);
        void updateGenerationDeltaTime(int *, long long *);
        void updateHeading(double *);

        void updateTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr&);

        struct Object {
            std::string uuid;
            int objectID; // 0-255 for CPM
            vanetza::Clock::time_point timestamp;
            rclcpp::Time timestamp_ros;
            double position_x;
            double position_y;
            double position_z;
            double orientation_x;
            double orientation_y;
            double orientation_z;
            double orientation_w;
            double twist_linear_x;
            double twist_linear_y;
            double twist_angular_x;
            double twist_angular_y;
            int shape_x;
            int shape_y;
            int shape_z;
            int xDistance;
            int yDistance;
            int xSpeed;
            int ySpeed;
            int yawAngle;
            vanetza::PositionFix position;
            int timeOfMeasurement;
            bool to_send;
            int to_send_trigger;
        };

    private:
        void schedule_timer();
        void on_timer(vanetza::Clock::time_point);
        void send();
        void createTables();
        void insertCpmToCpmTable(vanetza::asn1::Cpm, char*);

        V2XNode *node_;
        vanetza::Runtime &runtime_;
        vanetza::Clock::duration cpm_interval_;

        // 自車の状態を保持する構造体
        struct Ego_station {
            double mgrs_x;
            double mgrs_y;
            double latitude;
            double longitude;
            double altitude;
            double heading;
        };
        Ego_station ego_;

        // 経路情報を保持するメンバ変数を追加
        std::optional<autoware_auto_planning_msgs::msg::Trajectory> latest_trajectory_;
        std::mutex trajectory_mutex_; 
        
        int generationDeltaTime_;
        long long gdt_timestamp_;

        bool sending_;
        bool is_sender_;
        bool reflect_packet_;
        int cpm_num_;
        int received_cpm_num_;
    };
}

#endif /* CPM_APPLICATION_HPP_EUIC2VFR */
