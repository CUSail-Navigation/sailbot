let ros;
let controlModeTopic;

function connectToROS(url) {
    ros = new ROSLIB.Ros({
        url: url
    });

    ros.on('connection', function () {
        console.log('Connected to rosbridge server at:', url);
        subscribeToTopics();
    });

    ros.on('error', function (error) {
        console.error('Error connecting to rosbridge server:', error);
    });

    ros.on('close', function () {
        console.log('Connection to rosbridge server closed.');
    });
}

// Add event listener for the button
document.getElementById('connect-to-ros').addEventListener('click', function () {
    const rosbridgeAddress = document.getElementById('ros-url').value;
    if (rosbridgeAddress) {
        connectToROS(rosbridgeAddress);
    } else {
        console.error('Please enter a valid ROS URL.');
    }
});

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
}
function submitWaypoint() {
    const latitude = parseFloat(document.getElementById('latitude').value);
    const longitude = parseFloat(document.getElementById('longitude').value);

    if (isNaN(latitude) || isNaN(longitude)) {
        alert("Please enter valid numeric latitude and longitude.");
        return;
    }

    console.log("Submitting waypoint:", latitude, longitude);

    // Call the ROS service to add the waypoint
    const waypointService = new ROSLIB.Service({
        ros: ros,
        name: '/add_waypoint',
        serviceType: 'sailboat_interface/srv/AddWaypoint'
    });

    const request = new ROSLIB.ServiceRequest({
        latitude: latitude,
        longitude: longitude
    });

    waypointService.callService(request, function (response) {
        console.log("called waypoint service")
        if (response.success) {
            console.log("Waypoint added successfully!");
        } else {
            console.log("Failed to add waypoint.");
        }
    });
}
document.addEventListener('DOMContentLoaded', () => {
    document.getElementById('submit-waypoint').addEventListener('click', submitWaypoint);
});

window.onload = function () {
    // Nothing for now
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



