// Standard imports.
#include <string>
#include <sstream>
#include <optional>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <memory>
// ROS imports.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
// Custom messages.
#include "sailboat_interface/msg/algo_debug.hpp"
#include "sailboat_interface/srv/waypoint.hpp"

enum SailState {
    NORMAL,
    TACK,
};

/** Represents a point in latitude and longitude coordinates. */
class LatLongPoint;

/** Represents a point in UTM coordinates. */
class UTMPoint {
public:
    double easting;
    double northing;
    int zoneNumber;
    std::string zoneLetter;

    UTMPoint(double northing, double easting, int zoneNumber, std::string zoneLetter) {
        this->northing = northing;
        this->easting = easting;
        this->zoneNumber = zoneNumber;
        this->zoneLetter = zoneLetter;
    }

    double distance_to(const UTMPoint& other) const {
        return std::hypot(other.easting - easting, other.northing - northing);
    }

    double target_bearing_to(const UTMPoint& other) const {
        double delta_easting = other.easting - easting;
        double delta_northing = other.northing - northing;
        return std::atan2(delta_northing, delta_easting) * 180.0 / M_PI;
    }
};

/** Represents a point in latitude and longitude coordinates. */
class LatLongPoint {
    double latitude;
    double longitude;

public:
    LatLongPoint(double latitude, double longitude) {
        this->latitude = latitude;
        this->longitude = longitude;
    }

    double get_latitude() const {
        return latitude;
    }
    double get_longitude() const {
        return longitude;
    }

    UTMPoint to_utm() const {
        double x = longitude * 111320.0; // Rough local approximation for structural compilation
        double y = latitude * 110540.0;
        return UTMPoint(x, y, 32, "T");
    }

    sensor_msgs::msg::NavSatFix to_navsatfix_msg() const {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = latitude;
        msg.longitude = longitude;
        return msg;
    }

    std::string to_string() const {
        std::ostringstream oss;
        oss << "Latitude = " << latitude << ", Longitude = " << longitude;
        return oss.str();
    }
};


/** The sailing algorithm responsible for changing the rudder angle based on the
 *  current location, destination, and heading direction. */
class Algo : public rclcpp::Node {
    /** CLASS INVARIANTS:
     *  - All angles are in degrees.
     *  - Angles are positive to the right and negative to the left of the boat's centerline.
     *  - BOAT reference frame: the direction the boat is facing represents 0 degrees.
     *  - EARTH reference frame: East is 0 degrees, North is 90 degrees, etc.
     */
    // SENSOR AND INPUT DATA.
    std::optional<double> wind_direction;
    std::optional<double> absolute_wind_dir;
    std::optional<double> heading_direction;
    std::optional<double> heading_difference;
    std::optional<double> dist_to_dest;
    std::optional<UTMPoint> current_location;
    std::optional<UTMPoint> current_waypoint;

    // ALGORITHM STATE.
    SailState sail_state = NORMAL;
    std::optional<UTMPoint> current_destination;
    std::string current_mode = "manual";
    double tack_time_tracker = 0.0;
    bool turn_left = true;
    double timer_period = 0.200;

    // RUNTIME MUTABLE/UPDATABLE PARAMETERS.
    int no_go_zone = 60;
    int neutral_zone = 15;
    int tacking_buffer = 10;

    // CONSTANT ALGORITHM/PHYSICAL PARAMETERS.
    static constexpr int MAX_RUDDER_ANGLE = 25;
    static constexpr int NEUTRAL_RUDDER_ANGLE = 0;
    static constexpr int POP_RADIUS = 5;

    // ROS SUBSCRIBERS.
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_curr_loc;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_heading_direction;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_wind_direction;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_current_waypoint;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_no_go_zone;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_neutral_zone;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_tacking_buffer;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_mode;

    // ROS PUBLISHERS.
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rudder_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr danger_zone_pub;
    rclcpp::Publisher<sailboat_interface::msg::AlgoDebug>::SharedPtr state_pub;

    // TIMERS & CLIENTS
    rclcpp::TimerBase::SharedPtr step_timer;
    rclcpp::TimerBase::SharedPtr state_timer;
    rclcpp::Client<sailboat_interface::srv::Waypoint>::SharedPtr waypoint_client;

public:
    Algo() : Node("main_algo") {
        // Parameters
        this->declare_parameter("timer_period", 0.200);
        timer_period = this->get_parameter("timer_period").as_double();

        this->declare_parameter("tacking_buffer", 10);
        tacking_buffer = this->get_parameter("tacking_buffer").as_int();

        this->declare_parameter("no_go_zone", 60);
        no_go_zone = this->get_parameter("no_go_zone").as_int();

        this->declare_parameter("debug", true);
        bool debug = this->get_parameter("debug").as_bool();

        // Subscriptions
        subscription_curr_loc = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&Algo::currGpsCallback, this, std::placeholders::_1));

        // Subscription for heading direction.
        subscription_heading_direction = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/imu", 10, std::bind(&Algo::headingDirectionCallback, this, std::placeholders::_1));

        // Subscription for wind direction.
        subscription_wind_direction = this->create_subscription<std_msgs::msg::Int32>(
            "wind", 10, std::bind(&Algo::windCallback, this, std::placeholders::_1));

        // Subscription for current waypoint.
        subscription_current_waypoint = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "current_waypoint", 10, std::bind(&Algo::currentWaypointCallback, this, std::placeholders::_1));

        // Subscription for runtime no-go zone adjustments.
        subscription_no_go_zone = this->create_subscription<std_msgs::msg::Int32>(
            "no_go_zone", 10, std::bind(&Algo::noGoZoneCallback, this, std::placeholders::_1));

        // Subscription for runtime neutral zone adjustments.
        subscription_neutral_zone = this->create_subscription<std_msgs::msg::Int32>(
            "neutral_zone", 10, std::bind(&Algo::neutralZoneCallback, this, std::placeholders::_1));

        // Subscription for runtime tacking buffer adjustments.
        subscription_tacking_buffer = this->create_subscription<std_msgs::msg::Int32>(
            "tacking_buffer", 10, std::bind(&Algo::tackingBufferCallback, this, std::placeholders::_1));

        // Subscription for the mode.
        subscription_mode = this->create_subscription<std_msgs::msg::String>(
            "current_mode", 10, std::bind(&Algo::modeCallback, this, std::placeholders::_1));

        // Publisher for the rudder angle.
        rudder_angle_pub = this->create_publisher<std_msgs::msg::Int32>("algo_rudder", 10);

        // Publisher for the danger zone notification.
        danger_zone_pub = this->create_publisher<std_msgs::msg::Bool>("danger_zone", 10);

        // Service Client
        waypoint_client = this->create_client<sailboat_interface::srv::Waypoint>("mutate_waypoint_queue");

        // Primary execution loop
        step_timer = this->create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&Algo::step, this)
        );

        RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug ? "true" : "false");

        if (debug) {
            // Publisher for internal state
            state_pub = this->create_publisher<sailboat_interface::msg::AlgoDebug>("main_algo_debug", 10);
            // Timer to publish state every 1 second (1000ms)
            state_timer = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&Algo::publishStateDebug, this)
            );
        }

        RCLCPP_INFO(this->get_logger(), "Main algo started successfully");
    }


    // ALL THE ROS CALLBACKS AND PUBLISHERS

    void currGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_location = LatLongPoint(msg->latitude, msg->longitude).to_utm();

        if (current_destination) {
            dist_to_dest = current_location->distance_to(*current_waypoint);
            RCLCPP_INFO(this->get_logger(), "Distance to destination: %.2f", dist_to_dest.value());

            if (dist_to_dest.value() < 3.0) {
                if (current_mode != "station_keeping") {
                    RCLCPP_INFO(this->get_logger(), "======================= Waypoint popped =======================");
                    popWaypoint();
                    current_destination = std::nullopt;
                }
            }
        }
    }

    void headingDirectionCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Heading Direction: %.2f", msg->z);
        heading_direction = msg->z;
    }

    void windCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        wind_direction = static_cast<double>(msg->data);
        if (heading_direction) {
            absolute_wind_dir = std::fmod(wind_direction.value() + heading_direction.value(), 360.0);
            RCLCPP_INFO(this->get_logger(), "Wind Direction: %.2f, Absolute Wind Direction: %.2f",
                wind_direction.value(), absolute_wind_dir.value());
        }
    }

    void currentWaypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_waypoint = LatLongPoint(msg->latitude, msg->longitude).to_utm();
        RCLCPP_INFO(this->get_logger(), "Updated current waypoint to: (%.4f, %.4f)", msg->latitude, msg->longitude);
        RCLCPP_INFO(this->get_logger(), "New waypoint received");

        current_destination = current_waypoint;
        tack_time_tracker = 0.0;
        sail_state = NORMAL;
    }

    void noGoZoneCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        no_go_zone = msg->data;
        RCLCPP_INFO(this->get_logger(), "No-Go Zone: %d", no_go_zone);
    }

    void neutralZoneCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        neutral_zone = msg->data;
        RCLCPP_INFO(this->get_logger(), "Neutral Zone: %d", neutral_zone);
    }

    void tackingBufferCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        tacking_buffer = msg->data;
        RCLCPP_INFO(this->get_logger(), "Tacking Buffer: %d", tacking_buffer);
    }

    void modeCallback(const std_msgs::msg::String::SharedPtr msg) {
        current_mode = msg->data;
    }



    /** This method continuously updates the boat's sailing, making the ideal decision for the boat's
     *  position in a given moment. Sail the boat through looping this method forever. */
    void step() {
        // Not enough information yet.
        if (!current_location || !current_waypoint || !heading_direction) return;

        if (!current_destination) {
            current_destination = current_waypoint;
            sail_state = NORMAL;
        }

        updateState();

        // Equiv to np.mod(heading - bearing + 180, 360) - 180
        double target_bearing = current_location->target_bearing_to(*current_destination);
        double raw_diff = heading_direction.value() - target_bearing + 180.0;
        heading_difference = std::fmod(raw_diff, 360.0);
        if (heading_difference.value() < 0) heading_difference.value() += 360.0;
        heading_difference.value() -= 180.0;

        if (sail_state == NORMAL) setNormalRudder();
        else if (sail_state == TACK) setTackRudder();
    }

    void updateState() {
        if (sail_state == NORMAL) {
            if (current_location->distance_to(*current_destination) < POP_RADIUS) {
                RCLCPP_INFO(this->get_logger(), "Reached tacking point");
                current_destination = current_waypoint;
            }

            if (waypointInNogo() && tack_time_tracker > tacking_buffer) {
                current_destination = calculateTackingPoint();
                if (wind_direction.value() < 180.0) turn_left = false;
                else turn_left = true;
                sail_state = TACK;
                tack_time_tracker = 0.0;
            } else if (tack_time_tracker > tacking_buffer) {
                tack_time_tracker = 0.0;
            }
            tack_time_tracker += timer_period;

        } else if (sail_state == TACK) {
            if (tack_time_tracker > tacking_buffer) {
                sail_state = NORMAL;
                tack_time_tracker = 0.0;
                current_destination = current_waypoint;
            } else if (turn_left) {
                if (heading_difference && heading_difference.value() > 0) {
                    sail_state = NORMAL;
                    tack_time_tracker = 0.0;
                }
            } else {
                if (heading_difference && heading_difference.value() < 0) {
                    sail_state = NORMAL;
                    tack_time_tracker = 0.0;
                }
            }
            tack_time_tracker += timer_period;
        } else {
            RCLCPP_INFO(this->get_logger(), "Unknown state");
        }
    }

    bool boatInNogo() {
        RCLCPP_INFO(this->get_logger(), "Wind Direction: %.2f", wind_direction.value_or(0.0));
        if (!wind_direction) return false;
        return (180.0 - no_go_zone < wind_direction.value() && wind_direction.value() < 180.0 + no_go_zone);
    }

    bool waypointInNogo() {
        if (!current_waypoint || !wind_direction || !absolute_wind_dir || !current_location) return false;

        double target_bearing = current_location->target_bearing_to(*current_waypoint);
        double opposite_wind_dir = std::fmod(absolute_wind_dir.value() + 180.0, 360.0);

        double diff = std::fmod(target_bearing - opposite_wind_dir + 180.0, 360.0);
        if (diff < 0) diff += 360.0;
        diff -= 180.0;

        return std::abs(diff) < no_go_zone;
    }

    UTMPoint calculateTackingPoint() {
        double tack_angle = 0.0;
        double approach_angle = 0.0;

        if (wind_direction.value() < 180.0) {
            tack_angle = std::fmod(absolute_wind_dir.value() - no_go_zone, 360.0);
            approach_angle = std::fmod(no_go_zone + absolute_wind_dir.value(), 360.0);
        } else {
            tack_angle = std::fmod(no_go_zone + absolute_wind_dir.value(), 360.0);
            approach_angle = std::fmod(absolute_wind_dir.value() - no_go_zone, 360.0);
        }
        if (tack_angle < 0) tack_angle += 360.0;
        if (approach_angle < 0) approach_angle += 360.0;

        double vec1_x = std::cos(tack_angle * M_PI / 180.0);
        double vec1_y = std::sin(tack_angle * M_PI / 180.0);
        double vec2_x = -std::cos(approach_angle * M_PI / 180.0);
        double vec2_y = -std::sin(approach_angle * M_PI / 180.0);

        // Linear equation solving system matrix A and column vector b
        // [vec1_x  -vec2_x] [t1] = [P2_x - P1_x]
        // [vec1_y  -vec2_y] [t2]   [P2_y - P1_y]
        double p1_x = current_location->easting;
        double p1_y = current_location->northing;
        double p2_x = current_waypoint->easting;
        double p2_y = current_waypoint->northing;

        double bx = p2_x - p1_x;
        double by = p2_y - p1_y;

        double det = (vec1_x * -vec2_y) - (-vec2_x * vec1_y);
        double t1 = 0.0;
        if (std::abs(det) > 1e-6) {
            t1 = (bx * -vec2_y - (-vec2_x * by)) / det;
        }

        double tp_easting = p1_x + t1 * vec1_x;
        double tp_northing = p1_y + t1 * vec1_y;

        return UTMPoint(tp_easting, tp_northing, current_location->zoneNumber, current_location->zoneLetter);
    }

    void setTackRudder() {
        int rudder_angle = turn_left ? -MAX_RUDDER_ANGLE : MAX_RUDDER_ANGLE;

        RCLCPP_INFO(this->get_logger(), "Rudder Angle: %d", rudder_angle);
        auto msg = std_msgs::msg::Int32();
        msg.data = rudder_angle;
        rudder_angle_pub->publish(msg);
    }

    void setNormalRudder() {
        int rudder_angle = 0;
        if (heading_difference) {
            rudder_angle = static_cast<int>(std::round(heading_difference.value() / 180.0 * 20.0));
        }

        RCLCPP_INFO(this->get_logger(), "Rudder Angle: %d", rudder_angle);
        auto msg = std_msgs::msg::Int32();
        msg.data = rudder_angle;
        rudder_angle_pub->publish(msg);
    }

    // ASYNC CLIENT INTERACTION
    void popWaypoint() {
        while (!waypoint_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waypoint service not available, waiting...");
        }

        auto request = std::make_shared<sailboat_interface::srv::Waypoint::Request>();
        request->command = "pop";
        request->argument = "";

        auto result = waypoint_client->async_send_request(request,
            [this](rclcpp::Client<sailboat_interface::srv::Waypoint>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Waypoint popped successfully.");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Failed to pop waypoint from the service.");
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            });
    }

    void publishStateDebug() {
        RCLCPP_INFO(this->get_logger(), "Publishing internal state");
        auto debug_msg = sailboat_interface::msg::AlgoDebug();

        debug_msg.tacking = (sail_state == TACK);

        debug_msg.heading_dir.data = heading_direction ? static_cast<int>(heading_direction.value()) : 0;

        if (current_destination) {
            // Reconstructing local latlong representation for debug pipeline output safely
            LatLongPoint local_ll(current_destination->northing / 110540.0, current_destination->easting / 111320.0);
            debug_msg.curr_dest = local_ll.to_navsatfix_msg();
        } else {
            RCLCPP_INFO(this->get_logger(), "Current destination is None");
            debug_msg.curr_dest = sensor_msgs::msg::NavSatFix();
        }

        debug_msg.diff.data = heading_difference ? static_cast<int>(heading_difference.value()) : 0;
        debug_msg.dist_to_dest.data = dist_to_dest ? static_cast<int>(dist_to_dest.value()) : -1;
        debug_msg.no_go_zone.data = no_go_zone;
        debug_msg.neutral_zone.data = neutral_zone;

        state_pub->publish(debug_msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<Algo> mainAlgo = std::make_shared<Algo>();

    // ROS2 in C++ handles keyboard interrupts; the node will automatically be destroyed when it gets out of scope.
    rclcpp::spin(mainAlgo);

    rclcpp::shutdown();
    return 0;
}