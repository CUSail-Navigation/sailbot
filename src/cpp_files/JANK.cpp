// Standard imports.
#include <string>
#include <sstream>
#include <optional>
#include <cmath>
// ROS imports.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
// Custom messages.
#include "sailboat_interface/msg/algo_debug.hpp" //todo this is giving me errors
//#include "sailboat_interface/srv/[waypoinythingy]" //todo this is giving me errors

class LatLongPoint;
class NavSatFix;

/** Represents a point in UTM coordinates. */
class UTMPoint {
    double northing;
    double easting;
    int zoneNumber;
    std::string zoneLetter;

    public:
    UTMPoint(double northing, double easting, int zoneNumber, std::string zoneLetter) {
        this->northing = northing;
        this->easting = easting;
        this->zoneNumber = zoneNumber;
        this->zoneLetter = zoneLetter;
    }


    // /** Covert UTM Coordinates to latitude and longitude. */
    // LatLongPoint to_latlong() {
    //     return LatLongPoint(0.0, 0.0);
    // }

    /*
    /** Convert UTM Coordinates to NavSatFix message. /
    NavSatFix to_navsatfix_msg() {
        LatLongPoint lat_long = to_latlong();
        ;avSatFix msg;
        msg.setLatitude(lat_long.getlatitude());
        msg.setLongitude(lat_long.getLongitude());
        return msg;
    }
    */
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

    double get_latitude() {
        return latitude;
    }

    double get_longitude() {
        return longitude;
    }

    /* // TODO fix
    /** Convert latitude and longitude to UTM coordinates. /
    UTMPoint to_utm {
        x, y, zone_number, zone_letter = utm.from_latlon(self.latitude, self.longitude);
        return UTMPoint(x, y, zone_number, zone_letter);
    };
    */

    std::string to_string() {
        std::ostringstream oss;
        oss << "Latitude = " << latitude << ", Longitude = " << longitude;
        return oss.str();
    };
};

enum SailState { // TODO see if this can be removed, later. --> if it's redundant because it's obvious when we're tacking and we don't need to keep track of it
    NORMAL,
    TACK,
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

    // SENSOR AND INPUT DATA.                   // TODO write docs / class invariants / definitions for all variables
    std::optional<double> wind_direction;
    std::optional<double> wind_speed;           // TODO Work this in somewhere --> potentially for sail trim
    std::optional<double> heading_direction;    // convention: 0 is east, 90 is north, etc. todo see if this is still true
    std::optional<double> heading_difference;   // What is this for?? --> the difference to your target angle
    std::optional<double> dist_to_dest;
    std::optional<UTMPoint> current_location;
    std::optional<UTMPoint> current_waypoint;   // The manual waypoint set on the map. New version: this will be our end goal.

    // ALGORITHM STATE.     // TODO re-organize these sections because they don't seem super right.
    SailState state = NORMAL;
    // std::optional<UTMPoint> currentDestination;        // What the algorithm calculates as the next place to go to, to get to currentWaypoint. This was for the old version; don't need anymore.
    // std::optional<UTMPoint> tacking_point = None       // Do we need this
    std::string current_mode = "manual";

    // CONSTANT ALGORITHM/PHYSICAL PARAMETERS.
    /* Nomenclature:
     *  - NO_GO_ZONE (angle): irons; where the boat can't generate lift (left-right of the wind).
     *  - DANGER_ZONE (angle): smaller zone opposite irons; don't sail here for accidental jibes.
     *  - POP_RADIUS (meters): how close we need to get to a waypoint to "pop" it.
     */
    static constexpr int NO_GO_ZONE = 45;       // TODO nail down the value of this.
    static constexpr int DANGER_ZONE = 10;      // TODO nail down the value of this.
    static constexpr int TACKING_BUFFER = 30;   // Time (30 sec) to next tack
    static constexpr int NEUTRAL_ZONE = 10;     // Angle off the nose of the boat to prevent new tack. Don't need anymore?
    static constexpr int MAX_RUDDER_ANGLE = 25; // TODO nail down the value of this.
    static constexpr int NEUTRAL_RUDDER_ANGLE = 0;
    static constexpr int POP_RADIUS = 5;        // Do we need this??

    // ROS SUBSCRIBERS.
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_curr_loc;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_heading_direction;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_wind_direction;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_current_waypoint;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_no_go_zone;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_neutral_zone;   // TODO delete? 1) class variable 2) ros variable 3) ros subscription 4) ros callback method
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_tacking_buffer; // TODO delete? 1) class variable 2) ros variable 3) ros subscription 4) ros callback method
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_mode; // todo is this state? do we need?

    // ROS PUBLISHERS.
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rudder_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr danger_zone_pub;

    rclcpp::Publisher<sailboat_interface::msg::AlgoDebug>::SharedPtr state_pub;
    rclcpp::TimerBase::SharedPtr state_timer;

    public:
    Algo() : Node("main_algo") {
        // Stuff with the timer
        // All the stuff to subscribe and publish.

        // Subscription for current location.
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

        // Handle debug publishing.
        // Declare and get parameter
        this->declare_parameter("debug", true);
        bool debug = this->get_parameter("debug").as_bool();

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
        // Your logic here
    }

    void headingDirectionCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Your logic here
    }

    void windCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Your logic here
    }

    void currentWaypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // Your logic here
    }

    void noGoZoneCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Your logic here
    }

    void neutralZoneCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Your logic here
    }

    void tackingBufferCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Your logic here
    }

    void modeCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Your logic here
    }

    void publishStateDebug() {
       // Your logic here
    }



    /** This method continuously updates the boat's sailing, making the ideal decision for the boat's
     *  position in a given moment. Sail the boat through looping this method forever. */
    void step() {
        // Not enough information yet.
        if (!current_location || !current_waypoint || !heading_direction) return;

        updateState();

        switch (state) { // todo simplify so it only does something if the state changes?
            case TACK: setTackRudder(); // todo decide if we want a break here or if it should set normal after
            case NORMAL: setNormalRudder();
        }
    }

    /** Helper method to update the state of the algorithm. This function is called when the state changes. */
    void updateState() {
        if (state == NORMAL) {
            if (waypointInNogo()) {

            }
            else {
                // Sail directly to destination.
            }
        }
        else if (state == TACK) {

        }
        else {
            // self.get_logger().info("Unknown state")
        }
    }


    bool waypointInNogo() {// TODO implement this
        if (!(current_waypoint && wind_direction)) return false;


    }

    /** Initiates a tack by setting rudder angle to \code MAX_RUDDER_ANGLE\endcode on the correct side of the boat. */
    void setTackRudder() {
        int rudderAngle;

        // smth about when you're coming out of the tack
        if (heading_difference && std::abs(heading_difference.value()) <= NEUTRAL_ZONE) {
            rudderAngle = (int) (heading_difference.value() / 180 * 25);
        }
        // Decide which side.
        else {
            rudderAngle = heading_difference >= 0 ? MAX_RUDDER_ANGLE : -MAX_RUDDER_ANGLE;
        }

        // Publish.

        // todo implement this
    }

    void setNormalRudder() {}
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<Algo> mainAlgo = std::make_shared<Algo>();

    // ROS2 in C++ handles keyboard interrupts; the node will automatically be destroyed when it gets out of scope.
    rclcpp::spin(mainAlgo);

    rclcpp::shutdown();
    return 0;
}