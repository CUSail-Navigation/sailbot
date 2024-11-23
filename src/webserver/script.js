let ros;
let controlModeTopic;
const waypoints = [];
function connectToROS() {
    const rosbridgeAddress = "ws://localhost:9090";
    ros = new ROSLIB.Ros({
        url: rosbridgeAddress
    });
    ros.on('connection', function () {
        console.log('Connected to rosbridge server.');
        subscribeToTopics();
    });
    ros.on('error', function (error) {
        console.error('Error connecting to rosbridge server:', error);
    });
    ros.on('close', function () {
        console.log('Connection to rosbridge server closed.');
    });
}
function parseGpsData(message) {
    const latitude = message.latitude;
    const longitude = message.longitude;

    // Format the latitude and longitude to your desired precision
    const formattedLatitude = latitude.toFixed(6);
    const formattedLongitude = longitude.toFixed(6);

    // Update the DOM elements
    document.getElementById('latitude-value').innerText = formattedLatitude;
    document.getElementById('longitude-value').innerText = formattedLongitude;
}

function parseImuData(message) {
    parseQuaternionData(message.orientation);
    parseAngularVelocityData(message.angular_velocity);
}

function parseQuaternionData(message) {
    quaternionX = message.x;
    quaternionY = message.y;
    quaternionZ = message.z;
    quaternionW = message.w

    heading = quaternionToHeading(quaternionX, quaternionY, quaternionZ, quaternionW)

    formattedHeading = heading.toFixed(6);
    
    document.getElementById('heading-value').innerText = formattedHeading;
}

function parseAngularVelocityData(message) {
    angularVelocityZ = message.z; 

    angularVelocityZ = angularVelocityZ.toFixed(6);

    document.getElementById('angular-velocity-z-value').innerText = angularVelocityZ
}

/**
 * Converts a quaternion to a heading angle in degrees.
 * @param {number} x - The x component of the quaternion.
 * @param {number} y - The y component of the quaternion.
 * @param {number} z - The z component of the quaternion.
 * @param {number} w - The w component of the quaternion.
 * @returns {number} The heading angle in degrees.
 */

function quaternionToHeading(x, y, z, w) {
    // Compute the heading angle (yaw) from the quaternion
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const headingRadians = Math.atan2(siny_cosp, cosy_cosp);
    
    // Convert the heading from radians to degrees
    const headingDegrees = headingRadians * (180 / Math.PI);
    
    // Normalize to the range [0, 360)
    return (headingDegrees + 360) % 360;
}

function subscribeToTopics() {
    // Helper function to update DOM element with topic data
    function updateValue(elementId, value) {
        document.getElementById(elementId).innerText = value;
    }
    // Subscribe to /sailbot/algo_rudder
    const algoRudderTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/algo_rudder',
        messageType: 'std_msgs/Int32'
    });
    algoRudderTopic.subscribe(function (message) {
        updateValue('algo-rudder-value', message.data);
    });
    // Subscribe to /sailbot/algo_sail
    const algoSailTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/algo_sail',
        messageType: 'std_msgs/Int32'
    });
    algoSailTopic.subscribe(function (message) {
        updateValue('algo-sail-value', message.data);
    });
    // Subscribe to /sailbot/control_mode
    controlModeTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/control_mode',
        messageType: 'std_msgs/String'
    });
    controlModeTopic.subscribe(function (message) {
        updateValue('control-mode-value', message.data);
    });
    // Subscribe to /sailbot/radio_rudder
    const radioRudderTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/radio_rudder',
        messageType: 'std_msgs/Int32'
    });
    radioRudderTopic.subscribe(function (message) {
        updateValue('radio-rudder-value', message.data);
    });
    // Subscribe to /sailbot/radio_sail
    const radioSailTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/radio_sail',
        messageType: 'std_msgs/Int32'
    });
    radioSailTopic.subscribe(function (message) {
        updateValue('radio-sail-value', message.data);
    });
    // Subscribe to /sailbot/rudder_angle
    const rudderAngleTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/rudder_angle',
        messageType: 'std_msgs/Int32'
    });
    rudderAngleTopic.subscribe(function (message) {
        updateValue('rudder-angle-value', message.data);
    });
    // Subscribe to /sailbot/sail
    const sailTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/sail',
        messageType: 'std_msgs/Int32'
    });
    sailTopic.subscribe(function (message) {
        updateValue('sail-value', message.data);
    });
    const gpsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/gps',
        messageType: 'sensor_msgs/NavSatFix'
    });
    gpsTopic.subscribe(parseGpsData);

    const imuTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/imu',
        messageType: 'sensor_msgs/msg/Imu'
    });
    imuTopic.subscribe(parseImuData);
}
document.getElementById('submit-waypoint').addEventListener('click', function () {
    const latitude = document.getElementById('latitude').value;
    const longitude = document.getElementById('longitude').value;
    if (latitude && longitude) {
        const waypoint = `${latitude}, ${longitude}`;
        addWaypointToQueue(waypoint);
        waypoints.push(waypoint);
        displayWaypoints();
    }
    else {
        alert('Please enter both latitude and longitude.');
    }
    getWaypoints();
});
function addWaypointToQueue(waypoint) {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    const waypointQueueTopic = new ROSLIB.Topic({
        ros: ros,
        name: 'sailbot/waypoint_queue',
        messageType: 'std_msgs/String'
    });
    const waypointMessage = new ROSLIB.Message({ data: waypoint });
    waypointQueueTopic.publish(waypointMessage);
    console.log(`Published waypoint: ${waypoint}`);
}
function displayWaypoints() {
    const waypointListElement = document.getElementById('waypoint-list'); waypointListElement.innerHTML = '';
    waypoints.forEach(waypoint => {
        const waypointElement = document.createElement('div');
        waypointElement.textContent = waypoint; waypointListElement.appendChild(waypointElement);
    });
}
function getWaypoints() {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' }); ros.on('connection', function () {
        console.log('Connected to websocket server.');
    });
    const getWaypointClient = new ROSLIB.Service({
        ros: ros,
        name: '/sailbot/get_waypoint',
        serviceType: 'sailboat_interface/srv/Waypoint'
    });
    const request = new ROSLIB.ServiceRequest();
    getWaypointClient.callService(request, function (result) {
        if (result.success) {
            console.log(`Received waypoint: Latitude ${result.waypoint.latitude}, Longitude ${result.waypoint.longitude}`);
        } else { console.log('No waypoints in list'); }
    });
}
// Connect to ROS when the page loads
window.onload = function () {
    connectToROS();
};

// Function to toggle dropdown visibility
function toggleDropdown() {
    const dropdownContent = document.getElementById("dropdown-content");
    dropdownContent.style.display = dropdownContent.style.display === "block" ? "none" : "block";
}

// Function to update the dropdown button text based on selection
function selectMode(mode) {
    const modeButton = document.getElementById("mode-button");
    modeButton.innerText = mode; // Update button text
    toggleDropdown(); // Close the dropdown

    // Publish the selected mode to the /control_mode topic
    const message = new ROSLIB.Message({ data: mode.toLowerCase() });
    controlModeTopic.publish(message);
    console.log(`Published control mode: ${mode.toLowerCase()}`);
}

// Close dropdown if clicking outside of it
window.onclick = function (event) {
    if (!event.target.matches('.dropdown-button')) {
        const dropdowns = document.getElementsByClassName("dropdown-content");
        for (let i = 0; i < dropdowns.length; i++) {
            const openDropdown = dropdowns[i];
            if (openDropdown.style.display === "block") {
                openDropdown.style.display = "none";
            }
        }
    }
};