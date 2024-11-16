
let ros;
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
    const controlModeTopic = new ROSLIB.Topic({
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
// Connect to ROS when the page loads
window.onload = function () {
    connectToROS();
};
// Function to toggle dropdown visibility
function toggleDropdown() {
    var dropdownContent = document.getElementById("dropdown-content");
    dropdownContent.style.display = dropdownContent.style.display === "block" ? "none" : "block";
}
// Close dropdown if clicking outside of it
window.onclick = function (event) {
    if (!event.target.matches('.dropdown-button')) {
        var dropdowns = document.getElementsByClassName("dropdown-content");
        for (var i = 0; i < dropdowns.length; i++) {
            var openDropdown = dropdowns[i];
            if (openDropdown.style.display === "block") {
                openDropdown.style.display = "none";
            }
        }
    }
}
