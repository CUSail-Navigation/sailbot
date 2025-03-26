let ros;
let controlModeTopic;
let waypointTopic;

console.log("script.js loaded successfully");

let waypoints = []; // Global array for storing waypoints
const waypointMarkers = {}; // Global dictionary for waypoint markers
let map; // Global variable for the map instance
let sailboatMarker; // Global variable for the sailboat marker

let buoys = []
const buoyMarkers = {};
let sailPlanCoordinates = []; // Global variable for sailboat path coordinates
let sailPath; // Global variable for the sailboat trail
let formattedHeading;
let waypointPath; // Global variable for waypoint trail
let waypointPlanCoordinates = []; // Global variable for waypoint path coordinates

// Initialize the Google Map
function initMap() {
    const defaultLocation = { lat: 0, lng: 0 }; // Default center

    // Create a new map instance
    map = new google.maps.Map(document.getElementById("map"), {
        center: defaultLocation, // Center the map at the default location
        zoom: 2, // Set an initial zoom level
    });

    google.maps.event.addListener(map, "mousemove", function (event) {
        document.getElementById("mouse-pos").innerText =
            `Mouse Latitude: ${event.latLng.lat().toFixed(3)}
            Mouse Longitude: ${event.latLng.lng().toFixed(3)}`;
    });



    sailPath = new google.maps.Polyline({
        path: sailPlanCoordinates,
        geodesic: true,
        strokeColor: "#FF0000",
        strokeOpacity: 1.0,
        strokeWeight: 2,
    });

    waypointPath = new google.maps.Polyline({
        path: waypointPlanCoordinates,
        geodesic: true,
        strokeColor: "#FF0000",
        strokeOpacity: 1.0,
        strokeWeight: 2,
    });

    waypointPath.setOptions({ strokeColor: "#911084" });

    sailPath.setMap(map);
    waypointPath.setMap(map);
};
window.initMap = initMap;

function updateTrail(latitude, longitude) {
    const timestamp = Date.now();
    sailPlanCoordinates.push({ lat: latitude, lng: longitude, timestamp });

    // Filter out points older than 60 seconds
    const oneMinuteAgo = Date.now() - 60000;
    sailPlanCoordinates = sailPlanCoordinates.filter(coord => coord.timestamp >= oneMinuteAgo);

    // Map the coordinates for the polyline (only lat & lng)
    const currentPath = sailPlanCoordinates.map(coord => ({ lat: coord.lat, lng: coord.lng }));

    // Update the polyline with the filtered, current path
    sailPath.setPath(currentPath);
}


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
        console.log(rosbridgeAddress);
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

    // Add or update the sailboat marker on the map
    const sailboatLocation = { lat: latitude, lng: longitude };

    if (!sailboatMarker) {
        // Create a new marker if it doesn't exist
        sailboatMarker = new google.maps.Marker({
            position: sailboatLocation,
            map: map,
            title: "Sailboat Location",
            icon: {
                url: "boat.png", // Custom marker icon (optional)
            }
        });
    } else {
        // Update the marker's position
        sailboatMarker.setPosition(sailboatLocation);
        updateTrail(latitude, longitude);
    }

    // Optionally center the map on the sailboat
    map.setCenter(sailboatLocation);
    map.setZoom(17);
}


function parseImuData(message) {
    parseHeading(message);
    // parseAngularVelocityData(message);
}

function parseHeading(message) {
    heading = message.z;

    formattedHeading = heading.toFixed(6);

    document.getElementById('heading-value').innerText = formattedHeading;
    updateHeadAngle(formattedHeading, 'heading-value-dial')

    if (sailboatMarker) {
        rotateMarkerIcon("boat.png", heading, function (rotatedImageUrl) {
            sailboatMarker.setIcon({
                url: rotatedImageUrl
            });
        });
    }
}

// Not in use after changing quaternion to vector3 type
// function parseAngularVelocityData(message) {
//     angularVelocityZ = message.z;

//     angularVelocityZ = angularVelocityZ.toFixed(6);

//     document.getElementById('angular-velocity-z-value').innerText = angularVelocityZ
// }

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

let waypointService;
let BASE_THROTTLE_RATE = 1000;

function subscribeToTopics() {
    // Helper function to update DOM element with topic data
    function updateValue(elementId, value) {
        document.getElementById(elementId).innerText = value;
    }
    // Subscribe to /sailbot/algo_rudder
    const algoRudderTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/algo_rudder',
        messageType: 'std_msgs/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    algoRudderTopic.subscribe(function (message) {
        updateValue('algo-rudder-value', message.data);
    });
    // Subscribe to /sailbot/algo_sail
    const algoSailTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/algo_sail',
        messageType: 'std_msgs/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    algoSailTopic.subscribe(function (message) {
        updateValue('algo-sail-value', message.data);
    });
    // Subscribe to /sailbot/control_mode
    controlModeTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/control_mode',
        messageType: 'std_msgs/String',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    controlModeTopic.subscribe(function (message) {
        updateValue('control-mode-value', message.data);
        conditionalRender();
    });
    // Subscribe to /sailbot/radio_rudder
    const radioRudderTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/radio_rudder',
        messageType: 'std_msgs/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    radioRudderTopic.subscribe(function (message) {
        updateValue('radio-rudder-value', message.data);
    });
    // Subscribe to /sailbot/radio_sail
    const radioSailTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/radio_sail',
        messageType: 'std_msgs/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    radioSailTopic.subscribe(function (message) {
        updateValue('radio-sail-value', message.data);
    });
    // Subscribe to /sailbot/rudder_angle
    const rudderAngleTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/rudder_angle',
        messageType: 'std_msgs/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    rudderAngleTopic.subscribe(function (message) {
        updateValue('rudder-angle-value', message.data);
    });
    // Subscribe to /sailbot/sail
    const sailTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/sail',
        messageType: 'std_msgs/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    sailTopic.subscribe(function (message) {
        updateValue('sail-value', message.data);
    });
    const gpsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/gps',
        messageType: 'sensor_msgs/NavSatFix',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    gpsTopic.subscribe(parseGpsData);

    const imuTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/imu',
        throttle_rate: BASE_THROTTLE_RATE,
        messageType: 'geometry_msgs/msg/Vector3'
    });
    imuTopic.subscribe(parseImuData);

    const actualRudderAngleTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/actual_rudder_angle',
        messageType: 'std_msgs/msg/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    })
    actualRudderAngleTopic.subscribe(function (message) {
        updateValue('actual-tail-angle-value', message.data);
        updateTailAngle(message.data, "actual-tail-angle-dial");
    });

    // Subscribe to /sailbot/wind
    const windAngleTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/wind',
        messageType: 'std_msgs/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    windAngleTopic.subscribe(function (message) {
        updateValue('wind-angle-value', message.data);
    });

    const actualSailAngleTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/actual_sail_angle',
        messageType: 'std_msgs/msg/Int32',
        throttle_rate: BASE_THROTTLE_RATE,
    })
    actualSailAngleTopic.subscribe(function (message) {
        updateValue('actual-sail-angle-value', message.data);
        updateSailAngle(message.data, "actual-sail-angle-dial");
    });

    const algoDebugTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/main_algo_debug',
        messageType: 'sailboat_interface/msg/AlgoDebug',
        throttle_rate: BASE_THROTTLE_RATE,
    });
    algoDebugTopic.subscribe(function (message) {
        console.log("Algo debug")
        // Extract and log the received data
        const tacking = message.tacking;
        const tackingPoint = message.tacking_point;
        const headingDir = message.heading_dir.data;
        const currDest = message.curr_dest;
        const diff = message.diff.data;
    
        document.getElementById('tacking-value').innerText = tacking;
        document.getElementById('tacking-point-value').innerText = `${tackingPoint.latitude.toFixed(6)}, ${tackingPoint.longitude.toFixed(6)}`;
        document.getElementById('heading-dir-value').innerText = headingDir;
        document.getElementById('curr-dest-value').innerText = `${currDest.latitude.toFixed(6)}, ${currDest.longitude.toFixed(6)}`;
        document.getElementById('diff-value').innerText = diff;
    });

    waypointService = new ROSLIB.Service({
        ros: ros,
        name: '/sailbot/mutate_waypoint_queue',
        serviceType: 'sailboat_interface/srv/Waypoint'
    });
    waypointService = new ROSLIB.Service({
        ros: ros,
        name: '/sailbot/mutate_waypoint_queue',
        serviceType: 'sailboat_interface/srv/Waypoint'
    });
    const droppedPacketsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sailbot/dropped_packets',
        messageType: 'std_msgs/Int32'
    });
    droppedPacketsTopic.subscribe(function (message) {
        updateValue('dropped-packets-value', message.data);
    });
}
// Connect to ROS when the page loads
window.onload = function () {
    connectToROS();
};
function rotateMarkerIcon(src, heading, callback, size) {
    const image = new Image();
    image.src = src;
    image.onload = function () {
        // Use provided size, or fallback to the image’s natural dimensions.
        const width = size || image.naturalWidth;
        const height = size || image.naturalHeight;
        const diagonal = Math.sqrt(width * width + height * height);

        // Create a canvas with the determined dimensions
        const canvas = document.createElement("canvas");
        canvas.width = diagonal;
        canvas.height = diagonal;

        const ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, width, height);

        // Translate to center and rotate (adjust by -90 degrees if needed)
        ctx.translate(diagonal / 2, diagonal / 2);
        ctx.rotate((heading - 90) * (Math.PI / 180));

        // Draw the image centered
        ctx.drawImage(image, -width / 2, -height / 2, width, height);

        // Return the rotated image as a data URL
        callback(canvas.toDataURL());
    };
    image.onerror = function (err) {
        console.error("Error loading image:", err);
    };
}
document.getElementById('submit-waypoint').addEventListener('click', function () {
    const latitude = document.getElementById('waypoint-latitude').value;
    const longitude = document.getElementById('waypoint-longitude').value;

    if (latitude && longitude) {
        // Create a waypoint string for storage
        const waypoint = `${latitude},${longitude}`;
        waypoints.push(waypoint)
        addWaypointToQueue(waypoint); // Send the waypoint to ROS
        displayWaypoints(); // Update the waypoint list in the UI

        // Parse latitude and longitude to create a LatLng object
        const latLng = {
            lat: parseFloat(latitude),
            lng: parseFloat(longitude),
        };

        // Add a marker for the new waypoint on the map
        const marker = new google.maps.Marker({
            position: latLng,
            map: map,
            title: `Waypoint (${latitude},${longitude})`,
        });

        waypointMarkers[waypoint] = marker;
        waypointPlanCoordinates.push(latLng);

        waypointPath.setPath(waypointPlanCoordinates);

        console.log(`Waypoint added: ${waypoint}`);
    } else {
        // Alert the user if inputs are missing
        alert('Please enter both latitude and longitude.');
    }
});
function addWaypointToQueue(waypoint) {
    const waypointsString = waypoints.join(';');

    const request = new ROSLIB.ServiceRequest({
        command: "set",
        argument: waypointsString
    });

    waypointService.callService(request, function (result) {
        if (result.success) {
            console.log(result.message);
        } else {
            console.error(result.message);
        }
    });

    getWaypointQueue();
}
function displayWaypoints() {
    const waypointListElement = document.getElementById('waypoint-list');
    waypointListElement.innerHTML = ''; // Clear existing list

    waypoints.forEach((waypoint, index) => {
        const waypointElement = document.createElement('div');
        waypointElement.classList.add('waypoint-item');
        waypointElement.setAttribute('draggable', true);
        waypointElement.setAttribute('data-index', index);

        // Create the text for the waypoint
        const waypointText = document.createElement('span');
        waypointText.textContent = waypoint;
        waypointElement.appendChild(waypointText);

        // Create the delete button
        const deleteButton = document.createElement('button');
        deleteButton.textContent = 'Delete';
        deleteButton.classList.add('delete-button');
        deleteButton.addEventListener('click', () => {
            deleteWaypoint(index);
        });
        waypointElement.appendChild(deleteButton);

        waypointElement.addEventListener('dragstart', handleDragStart);
        waypointElement.addEventListener('dragover', handleDragOver);
        waypointElement.addEventListener('drop', handleDrop);
        waypointElement.addEventListener('dragend', handleDragEnd);

        waypointListElement.appendChild(waypointElement);
    });
}

document.getElementById('submit-buoy').addEventListener('click', function () {
    const latitude = document.getElementById('buoy-latitude').value;
    const longitude = document.getElementById('buoy-longitude').value;

    if (latitude && longitude) {
        // Create a waypoint string for storage
        const buoy = `${latitude},${longitude}`;
        buoys.push(buoy)

        // Parse latitude and longitude to create a LatLng object
        const latLng = {
            lat: parseFloat(latitude),
            lng: parseFloat(longitude),
        };

        // Add a marker for the new waypoint on the map
        const marker = new google.maps.Marker({
            position: latLng,
            map: map,
            title: `Buoy (${latitude},${longitude})`,
            icon: {
                path: google.maps.SymbolPath.CIRCLE, // Shape of the marker (e.g., CIRCLE, FORWARD_CLOSED_ARROW, etc.)
                scale: 8, // Size of the marker
                fillColor: "#FFA500", // Marker color (e.g., red)
                fillOpacity: 1,
                strokeWeight: 1,
                strokeColor: "#FFFFFF" // Optional: border color
            }
        });

        buoyMarkers[buoy] = marker;

        console.log(`Buoy added: ${buoy}`);
    } else {
        // Alert the user if inputs are missing
        alert('Please enter both latitude and longitude.');
    }
});

let draggedIndex = null;
let draggedElement = null;

function handleDragStart(event) {
    draggedIndex = parseInt(event.target.getAttribute('data-index'));
    draggedElement = event.target;
    event.dataTransfer.effectAllowed = 'move';
    event.target.classList.add('dragging');
}
function handleDragOver(event) {
    event.preventDefault();
    event.dataTransfer.dropEffect = 'move';
}
function handleDrop(event) {
    event.preventDefault();
    const targetElement = event.target.closest('.waypoint-item');
    if (!targetElement) return;
    const targetIndex = parseInt(targetElement.getAttribute('data-index'));

    if (draggedIndex !== null && targetIndex !== null && draggedIndex !== targetIndex) {
        const movedItem = waypoints[draggedIndex];
        waypoints.splice(draggedIndex, 1);
        waypoints.splice(targetIndex, 0, movedItem);
        displayWaypoints();
    }
}
function handleDragEnd(event) {
    if (draggedElement) {
        draggedElement.classList.remove('dragging');
    }
    draggedElement = null;
    draggedIndex = null;
    const waypointsString = waypoints.join(';');

    const request = new ROSLIB.ServiceRequest({
        command: "set",
        argument: waypointsString
    });

    waypointService.callService(request, function (result) {
        if (result.success) {
            console.log(result.message);
        } else {
            console.error(result.message);
        }
    });

    getWaypointQueue()
}
function deleteWaypoint(index) {
    // Remove the waypoint from the local array
    const waypoint = waypoints[index];
    console.log(`Deleting waypoint: ${waypoint}`);

    // Check if the marker exists
    if (waypointMarkers[waypoint]) {
        console.log(`Removing marker from map: ${waypoint}`);
        waypointMarkers[waypoint].setMap(null); // Remove from map
        delete waypointMarkers[waypoint]; // Remove from object
    } else {
        console.warn(`Marker not found for: ${waypoint}`);
    }

    // Remove waypoint trail for removed waypoint
    waypointPlanCoordinates.splice(index, 1);
    waypointPath.setPath(waypointPlanCoordinates);

    // Remove the waypoint from the array
    waypoints.splice(index, 1);

    let request;

    if (waypoints.length === 0) {
        request = new ROSLIB.ServiceRequest({
            command: "pop",
            argument: ""
        })
    }
    else {
        const waypointsString = waypoints.length > 0 ? waypoints.join(';') : '';

        request = new ROSLIB.ServiceRequest({
            command: "set",
            argument: waypointsString
        });
    }

    waypointService.callService(request, function (result) {
        if (result.success) {
            console.log(result.message);
        } else {
            console.error(result.message);
        }
    });

    // Update the display
    getWaypointQueue();
    displayWaypoints();
}
function getWaypointQueue() {
    const getRequest = new ROSLIB.ServiceRequest({
        command: "get",
        argument: ""
    });

    waypointService.callService(getRequest, function (getResult) {
        if (getResult.success) {
            console.log("Current ROS waypoint queue:", getResult.message);
        } else {
            console.error("Failed to fetch waypoint queue:", getResult.message);
        }
    });
}
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

function conditionalRender() {
    let controlModeVal = document.getElementById("control-mode-value").innerText.trim();
    let algoVals = document.querySelectorAll(".algo-mode");
    let rcVals = document.querySelectorAll(".rc-mode");
    if (controlModeVal == "algorithm") {
        algoVals.forEach(el => el.style.display = "flex");
        rcVals.forEach(el => el.style.display = "none");
    }
    if (controlModeVal == "radio") {
        algoVals.forEach(el => el.style.display = "none");
        rcVals.forEach(el => el.style.display = "flex");
    }
}


// // Code for dial Configuration
const width = 90;
const height = 90;
const radius = Math.min(width, height) / 2;
const needleLength = radius * 0.7;

// Create SVG Container
const svg = d3
    .select("#dial-container-sail")
    .append("svg")
    .attr("width", width)
    .attr("height", height)
    .append("g")
    .attr("transform", `translate(${width / 2}, ${height / 2})`);

// Draw Background Circle
svg.append("circle")
    .attr("r", radius)
    .attr("fill", "lightgray")
    .attr("stroke", "black")
    .attr("stroke-width", 2);

// Add Tick Marks (for better visualization)
const numTicks = 36; // Tick marks every 10 degrees
const tickLength = 10;
for (let i = 0; i < numTicks; i++) {
    const angle = (i / numTicks) * 2 * Math.PI; // Convert to radians
    const x1 = Math.cos(angle) * (radius - tickLength);
    const y1 = Math.sin(angle) * (radius - tickLength);
    const x2 = Math.cos(angle) * radius;
    const y2 = Math.sin(angle) * radius;

    svg.append("line")
        .attr("x1", x1)
        .attr("y1", y1)
        .attr("x2", x2)
        .attr("y2", y2)
        .attr("stroke", "black")
        .attr("stroke-width", i % 3 === 0 ? 2 : 1); // Longer ticks every 30 degrees
}

// Create the Needle
const needle = svg
    .append("line")
    .attr("x1", 0)
    .attr("y1", 0)
    .attr("x2", 0)
    .attr("y2", -needleLength)
    .attr("stroke", "red")
    .attr("stroke-width", 3)
    .attr("stroke-linecap", "round");

// Add a Center Circle
svg.append("circle").attr("r", 5).attr("fill", "black");

// Update Function for the Dial
function updateSailAngle(angle, id) {
    // Update Needle
    const angleRadians = (angle - 90) * (Math.PI / 180); // Convert degrees to radians and rotate to align with dial
    needle.transition()
        .duration(500) // Smooth transition
        .attr("transform", `rotate(${angle - 90})`);

    // Update Text Display
    document.getElementById(id).innerText = "Angle: " + angle;
}


// // Code for Tail dial Configuration
const widthT = 90;
const heightT = 90;
const radiusT = Math.min(width, height) / 2;
const needleLengthT = radius * 0.7;

// Create SVG Container
const svgT = d3
    .select("#dial-container-tail")
    .append("svg")
    .attr("width", widthT)
    .attr("height", heightT)
    .append("g")
    .attr("transform", `translate(${widthT / 2}, ${heightT / 2})`);

// Draw Background Circle
svgT.append("circle")
    .attr("r", radiusT)
    .attr("fill", "lightgray")
    .attr("stroke", "black")
    .attr("stroke-width", 2);

// Add Tick Marks (for better visualization)
const numTicksT = 36; // Tick marks every 10 degrees
const tickLengthT = 10;
for (let i = 0; i < numTicksT; i++) {
    const angleT = (i / numTicksT) * 2 * Math.PI; // Convert to radians
    const x1 = Math.cos(angleT) * (radiusT - tickLengthT);
    const y1 = Math.sin(angleT) * (radiusT - tickLengthT);
    const x2 = Math.cos(angleT) * radiusT;
    const y2 = Math.sin(angleT) * radiusT;

    svgT.append("line")
        .attr("x1", x1)
        .attr("y1", y1)
        .attr("x2", x2)
        .attr("y2", y2)
        .attr("stroke", "black")
        .attr("stroke-width", i % 3 === 0 ? 2 : 1); // Longer ticks every 30 degrees
}

// Create the Needle
const needleT = svgT
    .append("line")
    .attr("x1", 0)
    .attr("y1", 0)
    .attr("x2", 0)
    .attr("y2", -needleLengthT)
    .attr("stroke", "red")
    .attr("stroke-width", 3)
    .attr("stroke-linecap", "round");

// Add a Center Circle
svgT.append("circle").attr("r", 5).attr("fill", "black");

function updateTailAngle(angle, id) {
    // Update Needle
    const angleRadians = (angle - 90) * (Math.PI / 180); // Convert degrees to radians and rotate to align with dial
    needleT.transition()
        .duration(500) // Smooth transition
        .attr("transform", `rotate(${angle - 90})`);

    // Update Text Display
    document.getElementById(id).innerText = "Angle: " + angle;
}


// // Code for Heading dial Configuration
const widthH = 90;
const heightH = 90;
const radiusH = Math.min(width, height) / 2;
const needleLengthH = radius * 0.7;

// Create SVG Container
const svgH = d3
    .select("#dial-container-head")
    .append("svg")
    .attr("width", widthT)
    .attr("height", heightT)
    .append("g")
    .attr("transform", `translate(${widthH / 2}, ${heightH / 2})`);

// Draw Background Circle
svgH.append("circle")
    .attr("r", radiusH)
    .attr("fill", "lightgray")
    .attr("stroke", "black")
    .attr("stroke-width", 2);

// Add Tick Marks (for better visualization)
const numTicksH = 36; // Tick marks every 10 degrees
const tickLengthH = 10;
for (let i = 0; i < numTicksH; i++) {
    const angleH = (i / numTicksH) * 2 * Math.PI; // Convert to radians
    const x1 = Math.cos(angleH) * (radiusH - tickLengthH);
    const y1 = Math.sin(angleH) * (radiusH - tickLengthH);
    const x2 = Math.cos(angleH) * radiusH;
    const y2 = Math.sin(angleH) * radiusH;

    svgH.append("line")
        .attr("x1", x1)
        .attr("y1", y1)
        .attr("x2", x2)
        .attr("y2", y2)
        .attr("stroke", "black")
        .attr("stroke-width", i % 3 === 0 ? 2 : 1); // Longer ticks every 30 degrees
}

// Create the Needle
const needleH = svgH
    .append("line")
    .attr("x1", 0)
    .attr("y1", 0)
    .attr("x2", 0)
    .attr("y2", -needleLengthH)
    .attr("stroke", "red")
    .attr("stroke-width", 3)
    .attr("stroke-linecap", "round");

// Add a Center Circle
svgH.append("circle").attr("r", 5).attr("fill", "black");

function updateHeadAngle(angle, id) {
    // Update Needle
    const angleRadians = (angle - 90) * (Math.PI / 180); // Convert degrees to radians and rotate to align with dial
    needleH.transition()
        .duration(500) // Smooth transition
        .attr("transform", `rotate(${angle - 90})`);

    // Update Text Display
    document.getElementById(id).innerText = "Angle: " + angle;
}