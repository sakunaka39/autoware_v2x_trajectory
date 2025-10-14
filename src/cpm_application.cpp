#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/v2x_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <exception>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <string>
#include <algorithm>

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

#include <sqlite3.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace vanetza;
using namespace std::chrono;

using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

namespace v2x_trajectory {
  CpmApplication::CpmApplication(V2XNode *node, Runtime &rt, bool is_sender) :     
    node_(node),
    runtime_(rt),
    ego_(),
    generationDeltaTime_(0),
    sending_(false),
    is_sender_(is_sender),
    reflect_packet_(false),
    cpm_num_(0),
    received_cpm_num_(0)
  {
    RCLCPP_INFO(node_->get_logger(), "CpmApplication started. is_sender: %d", is_sender_);
    set_interval(milliseconds(100));
    createTables();
  }

  void CpmApplication::updateTrajectory(const Trajectory::ConstSharedPtr& msg) {
    // 書き込み時にミューテックスでロックする
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    latest_trajectory_ = *msg;
    RCLCPP_INFO(node_->get_logger(), "Trajectory updated with %zu points.", msg->points.size());
  }

  void CpmApplication::set_interval(Clock::duration interval) {
    cpm_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
  }

  void CpmApplication::schedule_timer() {
    runtime_.schedule(cpm_interval_, std::bind(&CpmApplication::on_timer, this, std::placeholders::_1), this);
  }

  void CpmApplication::on_timer(Clock::time_point) {
    schedule_timer();
    send();
  }

  CpmApplication::PortType CpmApplication::port() {
    //return btp::port_type(3939);
    return btp::ports::CPM;
  }

  void CpmApplication::indicate(const DataIndication &indication, UpPacketPtr packet) {

    asn1::PacketVisitor<asn1::Cpm> visitor;
    std::shared_ptr<const asn1::Cpm> cpm = boost::apply_visitor(visitor, *packet);

    if (cpm) {
      RCLCPP_INFO(node_->get_logger(), "[INDICATE] Received CPM #%d", received_cpm_num_);

      rclcpp::Time current_time = node_->now();
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] T_receive_r1 %ld", current_time.nanoseconds());

      asn1::Cpm message = *cpm;
      ItsPduHeader_t &header = message->header;

       std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
        std::chrono::system_clock::now().time_since_epoch()
      );
      node_->latency_log_file << "T_received," << header.stationID << "," << ms.count() << std::endl;


      // Calculate GDT and get GDT from CPM and calculate the "Age of CPM"
      GenerationDeltaTime_t gdt_cpm = message->cpm.generationDeltaTime;
      const auto time_now = duration_cast<milliseconds> (runtime_.now().time_since_epoch());
      uint16_t gdt = time_now.count();
      int gdt_diff = (65536 + (gdt - gdt_cpm) % 65536) % 65536;
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] GDT_CPM: %ld", gdt_cpm);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] GDT: %u", gdt);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] T_R1R2: %d", gdt_diff);


      CpmManagementContainer_t &management = message->cpm.cpmParameters.managementContainer;
      double lat = management.referencePosition.latitude / 1.0e7;
      double lon = management.referencePosition.longitude / 1.0e7;

      int zone;
      int grid_num_x = 4;
      int grid_num_y = 39;
      int grid_size = 100000;
      bool northp;
      double x, y;

      GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
      double x_mgrs = x - grid_num_x * grid_size;
      double y_mgrs = y - grid_num_y * grid_size;

      OriginatingVehicleContainer_t &ovc = message->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;

      // Calculate ego-vehicle orientation (radians) from heading (degree).
      // orientation: True-East, counter-clockwise angle. (0.1 degree accuracy)
      int heading = ovc.heading.headingValue;
      double orientation = (90.0 - (double) heading / 10.0) * M_PI / 180.0;
      if (orientation < 0.0) orientation += (2.0 * M_PI);
      // double orientation = heading / 10.0;
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] heading: %d", heading);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] orientation: %f", orientation);


      // Publish CPM Sender info to /v2x/cpm/sender through V2XNode function
      node_->publishCpmSenderObject(x_mgrs, y_mgrs, orientation);

      auto poc = message->cpm.cpmParameters.perceivedObjectContainer;
      RCLCPP_INFO(node_->get_logger(), "[INDICATE] Number of perceived objects in message: %ld", message->cpm.cpmParameters.numberOfPerceivedObjects);

      if (poc != NULL && poc->list.count > 0) {
          Trajectory received_trajectory;
          received_trajectory.header.stamp = node_->now();
          received_trajectory.header.frame_id = "map";

          for (int i = 0; i < poc->list.count; ++i) {
              PerceivedObject_t *p_obj = poc->list.array[i];
              TrajectoryPoint point;

              point.pose.position.x = static_cast<double>(p_obj->xDistance.value) / 10.0;
              point.pose.position.y = static_cast<double>(p_obj->yDistance.value) / 10.0;
              point.longitudinal_velocity_mps = static_cast<double>(p_obj->xSpeed.value) / 100.0;
              point.lateral_velocity_mps = static_cast<double>(p_obj->ySpeed.value) / 100.0;
              
              received_trajectory.points.push_back(point);
          }
          node_->publishTrajectory(received_trajectory);
          RCLCPP_INFO(node_->get_logger(), "Published received trajectory with %zu points", received_trajectory.points.size());
      }
      
      insertCpmToCpmTable(message, (char*) "cpm_received");

      if (reflect_packet_) {
        Application::DownPacketPtr packet{new DownPacket()};
        std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};

        payload->layer(OsiLayer::Application) = std::move(message);

        Application::DataRequest request;
        request.its_aid = aid::CP;
        request.transport_type = geonet::TransportType::SHB;
        request.communication_profile = geonet::CommunicationProfile::ITS_G5;

        Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);

        if (!confirm.accepted()) {
          throw std::runtime_error("[CpmApplication::indicate] Packet reflection failed");
        }
      }

      ++received_cpm_num_;

    } else {
      RCLCPP_INFO(node_->get_logger(), "[INDICATE] Received broken content");
    }
  }

  void CpmApplication::updateMGRS(double *x, double *y) {
    ego_.mgrs_x = *x;
    ego_.mgrs_y = *y;
    // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::updateMGRS] ego-vehicle.position: %.10f, %.10f", ego_.mgrs_x, ego_.mgrs_y);
  }

  void CpmApplication::updateRP(double *lat, double *lon, double *altitude) {
    ego_.latitude = *lat;
    ego_.longitude = *lon;
    ego_.altitude = *altitude;
  }

  void CpmApplication::updateGenerationDeltaTime(int *gdt, long long *gdt_timestamp) {
    generationDeltaTime_ = *gdt;
    gdt_timestamp_ = *gdt_timestamp; // ETSI-epoch milliseconds timestamp
  }

  void CpmApplication::updateHeading(double *yaw) {
    ego_.heading = *yaw;
  }

  void CpmApplication::send() {

    if (is_sender_) {

      sending_ = true;
      
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Sending CPM...");

      vanetza::asn1::Cpm message;

      // ITS PDU Header
      ItsPduHeader_t &header = message->header;
      header.protocolVersion = 1;
      header.messageID = 14;
      header.stationID = cpm_num_;

      CollectivePerceptionMessage_t &cpm = message->cpm;

      // Set GenerationDeltaTime
      cpm.generationDeltaTime = generationDeltaTime_ * GenerationDeltaTime_oneMilliSec;

      CpmManagementContainer_t &management = cpm.cpmParameters.managementContainer;
      management.stationType = StationType_passengerCar;
      management.referencePosition.latitude = ego_.latitude * 1e7;
      management.referencePosition.longitude = ego_.longitude * 1e7;

      management.referencePosition.altitude.altitudeValue = ego_.altitude * 100; // 単位をcmに変換
      management.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
      management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
      management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
      management.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingConfidence_unavailable;

      StationDataContainer_t *&sdc = cpm.cpmParameters.stationDataContainer;
      sdc = vanetza::asn1::allocate<StationDataContainer_t>();
      sdc->present = StationDataContainer_PR_originatingVehicleContainer;

      OriginatingVehicleContainer_t &ovc = sdc->choice.originatingVehicleContainer;
      ovc.speed.speedValue = 0;
      ovc.speed.speedConfidence = 1;

      // Calculate headingValue of ego-vehicle.
      // Convert ego-vehicle yaw to True-North clockwise angle (heading). 0.1 degree accuracy.
      int heading = std::lround(((-ego_.heading * 180.0 / M_PI) + 90.0) * 10.0);
      if (heading < 0) heading += 3600;
      ovc.heading.headingValue = heading;
      ovc.heading.headingConfidence = 1;

      // スレッドセーフな方法で経路情報をコピー
      std::optional<autoware_auto_planning_msgs::msg::Trajectory> trajectory_copy;
      {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          trajectory_copy = latest_trajectory_;
      } // ここでロックが自動的に解除される

      if (trajectory_copy && !trajectory_copy->points.empty()) {
        auto& points = trajectory_copy->points;

        // int num_objects = points.size();
        // パケットが大きくなりすぎないように、送信する点の数を 50 points に制限する
        const int max_points_to_send = 50;
        int num_objects = std::min(static_cast<int>(points.size()), max_points_to_send);

        cpm.cpmParameters.numberOfPerceivedObjects = num_objects;

        // PerceivedObjectContainerを割り当て
        auto& poc = cpm.cpmParameters.perceivedObjectContainer;
        poc = asn1::allocate<PerceivedObjectContainer_t>();

        for (int i = 0; i < num_objects; ++i) {
            PerceivedObject_t *p_obj = asn1::allocate<PerceivedObject_t>();

            // 経路上の点をオブジェクトとして設定
            p_obj->objectID = i; // 経路上のインデックスをIDとして利用
            p_obj->timeOfMeasurement = 0; // 'now'を意味する相対時間

            // MGRS座標系における相対位置を設定
            // 注意: 本来は自車からの相対座標を入れるべきだが、ここでは簡略化のためMGRS座標をそのまま利用
            p_obj->xDistance.value = static_cast<long>(points[i].pose.position.x * 10.0); // 10cm単位
            p_obj->xDistance.confidence = 1;
            p_obj->yDistance.value = static_cast<long>(points[i].pose.position.y * 10.0); // 10cm単位
            p_obj->yDistance.confidence = 1;

            p_obj->xSpeed.value = static_cast<long>(points[i].longitudinal_velocity_mps * 100.0); // 0.01m/s単位
            p_obj->xSpeed.confidence = 1;
            // 必須フィールド ySpeed を追加
            p_obj->ySpeed.value = static_cast<long>(points[i].lateral_velocity_mps * 100.0);
            p_obj->ySpeed.confidence = 1;

            p_obj->planarObjectDimension1 = asn1::allocate<ObjectDimension_t>();
            p_obj->planarObjectDimension2 = asn1::allocate<ObjectDimension_t>();
            p_obj->verticalObjectDimension = asn1::allocate<ObjectDimension_t>();
            p_obj->yawAngle = asn1::allocate<CartesianAngle_t>();
            
            p_obj->planarObjectDimension1->value = 1; // 0.1m, dummy value
            p_obj->planarObjectDimension1->confidence = ObjectDimensionConfidence_outOfRange;
            p_obj->planarObjectDimension2->value = 1; // 0.1m, dummy value
            p_obj->planarObjectDimension2->confidence = ObjectDimensionConfidence_outOfRange;
            p_obj->verticalObjectDimension->value = 1; // 0.1m, dummy value
            p_obj->verticalObjectDimension->confidence = ObjectDimensionConfidence_outOfRange;
            
            p_obj->yawAngle->value = 0; // 0.1 degree, dummy value
            p_obj->yawAngle->confidence = HeadingConfidence_outOfRange;

            // リストに追加
            asn_sequence_add(&poc->list, p_obj);
        }
        RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Sending CPM with %d trajectory points.", num_objects);
      } else {
        cpm.cpmParameters.perceivedObjectContainer = NULL;
        cpm.cpmParameters.numberOfPerceivedObjects = 0;
        RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Sending basic CPM (no trajectory).");
      }

      insertCpmToCpmTable(message, (char*) "cpm_sent");
      
      std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};

      payload->layer(OsiLayer::Application) = std::move(message);

      Application::DataRequest request;
      request.its_aid = aid::CP;
      request.transport_type = geonet::TransportType::SHB;
      request.communication_profile = geonet::CommunicationProfile::ITS_G5;

      Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);

      if (!confirm.accepted()) {
        throw std::runtime_error("[CpmApplication::send] CPM application data request failed");
      }

      sending_ = false;
      // rclcpp::Time current_time = node_->now();
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] [measure] T_depart_r1 %ld", current_time.nanoseconds());

      std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
        std::chrono::system_clock::now().time_since_epoch()
      );
      node_->latency_log_file << "T_depart," << cpm_num_ << "," << ms.count() << std::endl;

      ++cpm_num_;
    }
  }

  void CpmApplication::createTables() {
    sqlite3 *db = NULL;
    char* err = NULL;

    int ret = sqlite3_open("autoware_v2x.db", &db);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB File Open Error");
      return;
    }

    char* sql_command;
    
    sql_command = (char*) "create table if not exists cpm_sent(id INTEGER PRIMARY KEY, timestamp BIGINT, perceivedObjectCount INTEGER);";

    ret = sqlite3_exec(db, sql_command, NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (create table cpm_sent)");
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sql_command = (char*) "create table if not exists cpm_received(id INTEGER PRIMARY KEY, timestamp BIGINT, perceivedObjectCount INTEGER);";

    ret = sqlite3_exec(db, sql_command, NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (create table cpm_received)");
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sqlite3_close(db);
    RCLCPP_INFO(node_->get_logger(), "CpmApplication::createTables Finished");
  }

  void CpmApplication::insertCpmToCpmTable(vanetza::asn1::Cpm cpm, char* table_name) {
    sqlite3 *db = NULL;
    char* err = NULL;

    int ret = sqlite3_open("autoware_v2x.db", &db);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB File Open Error");
      return;
    }

    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    int perceivedObjectCount = 0;
    if (cpm->cpm.cpmParameters.numberOfPerceivedObjects) {
      perceivedObjectCount = cpm->cpm.cpmParameters.numberOfPerceivedObjects;
    }

    std::stringstream sql_command;
    
    sql_command << "insert into " << table_name << " (timestamp, perceivedObjectCount) values (" << timestamp << ", " << perceivedObjectCount << ");";

    ret = sqlite3_exec(db, sql_command.str().c_str(), NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (insertCpmToCpmTable)");
      RCLCPP_INFO(node_->get_logger(), sql_command.str().c_str());
      RCLCPP_INFO(node_->get_logger(), err);
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sqlite3_close(db);
    // RCLCPP_INFO(node_->get_logger(), "CpmApplication::insertCpmToCpmTable Finished");
  }
}