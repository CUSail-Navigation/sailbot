// main.js - Entry point with map management
import { ROSConnection } from './modules/rosConnection.js';
import { WaypointManager } from './modules/waypointManager.js';
import { BuoyManager } from './modules/buoyManager.js';
import { UIManager } from './modules/uiManager.js';
import { DialManager } from './modules/dialManager.js';

console.log("main.js loaded successfully");

// Global state
let rosConnection;
let waypointManager;
let buoyManager;
let uiManager;
let dialManager;

// Map-related globals (managed directly in main.js)
let map;
let sailboatMarker;
let currentDestMarker;
let sailPlanCoordinates = [];
let sailPath;

// Google Maps initialization - called directly by Maps API
window.initMap = function() {
    console.log("Initializing map...");
    const defaultLocation = { lat: 42.276848, lng: -71.756323 }; // Lake Quinsigamond 

    map = new google.maps.Map(document.getElementById("map"), {
        center: defaultLocation,
        zoom: 15,
    });

    // Mouse position tracking
    google.maps.event.addListener(map, "mousemove", (event) => {
        document.getElementById("mouse-pos").innerText =
            `Mouse Latitude: ${event.latLng.lat().toFixed(6)}
            Mouse Longitude: ${event.latLng.lng().toFixed(6)}`;
    });

    // Double-click for waypoint creation
    map.addListener("dblclick", (e) => {
        const latitude = e.latLng.lat();
        const longitude = e.latLng.lng();

        if (latitude && longitude && waypointManager) {
            waypointManager.addWaypoint(latitude, longitude);
        }
    });

    // Initialize sailboat trail
    sailPath = new google.maps.Polyline({
        path: sailPlanCoordinates,
        geodesic: true,
        strokeColor: "#FF0000",
        strokeOpacity: 1.0,
        strokeWeight: 2,
    });

    sailPath.setMap(map);

    // Initialize other managers now that map is ready
    initApp();
};

// Map management functions
function updateSailboatPosition(latitude, longitude, heading = null) {
    const sailboatLocation = { lat: latitude, lng: longitude };

    if (!sailboatMarker) {
        sailboatMarker = new google.maps.Marker({
            position: sailboatLocation,
            map: map,
            title: "Sailboat Location",
            icon: {
                path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                scale: 5,
                fillColor: "#007bff",
                fillOpacity: 1,
                strokeWeight: 1,
                strokeColor: "#ffffff",
                rotation: 0,
                anchor: new google.maps.Point(0, 2),
            }
        });
    } else {
        sailboatMarker.setPosition(sailboatLocation);
        updateTrail(latitude, longitude);
    }

    if (heading !== null) {
        updateSailboatHeading(heading);
    }
}

function updateSailboatHeading(heading) {
    if (sailboatMarker) {
        const googleHeading = (90 - heading + 360) % 360;
        sailboatMarker.setIcon({
            path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
            scale: 5,
            fillColor: "#007bff",
            fillOpacity: 1,
            strokeWeight: 1,
            strokeColor: "#ffffff",
            rotation: googleHeading,
            anchor: new google.maps.Point(0, 2)
        });
    }
}

function updateTrail(latitude, longitude) {
    const timestamp = Date.now();
    sailPlanCoordinates.push({ lat: latitude, lng: longitude, timestamp });

    // Filter out points older than 60 seconds
    const oneMinuteAgo = Date.now() - 60000;
    sailPlanCoordinates = sailPlanCoordinates.filter(coord => coord.timestamp >= oneMinuteAgo);

    // Update the polyline
    const currentPath = sailPlanCoordinates.map(coord => ({ lat: coord.lat, lng: coord.lng }));
    sailPath.setPath(currentPath);
}

function updateCurrentDestination(latitude, longitude) {
    const currentDestLocation = { lat: latitude, lng: longitude };
    
    if (!currentDestMarker) {
        currentDestMarker = new google.maps.Marker({
            position: currentDestLocation,
            map: map,
            title: "Current Destination Location",
            icon: {
                path: google.maps.SymbolPath.CIRCLE,
                scale: 8,
                fillColor: "#0000FF",
                fillOpacity: 1,
                strokeWeight: 1,
                strokeColor: "#FFFFFF"
            }
        });
    }
    
    currentDestMarker.setPosition(currentDestLocation);
    currentDestMarker.setMap(map);
}

// Initialize the application (called after map is ready)
function initApp() {
    console.log("Initializing app...");
    
    // Initialize managers
    rosConnection = new ROSConnection();
    waypointManager = new WaypointManager(map);
    buoyManager = new BuoyManager(map);
    dialManager = new DialManager();
    uiManager = new UIManager(rosConnection, dialManager);
    
    // Set up dependencies - pass map functions to ROS connection
    rosConnection.setMapFunctions({
        updateSailboatPosition,
        updateSailboatHeading,
        updateCurrentDestination
    });
    rosConnection.setWaypointManager(waypointManager);
    rosConnection.setDialManager(dialManager);
    rosConnection.setUIManager(uiManager);
    
    console.log("App initialization complete");
}

// Global function for collapsible sections
window.toggleSection = function(headerElement) {
    const content = headerElement.nextElementSibling;
    const icon = headerElement.querySelector(".toggle-icon");

    if (content.classList.contains("hidden")) {
        content.classList.remove("hidden");
        icon.textContent = "−";
    } else {
        content.classList.add("hidden");
        icon.textContent = "+";
    }
};

// Mode selection handling
window.toggleDropdown = function() {
    const dropdownContent = document.getElementById("dropdown-content");
    dropdownContent.style.display = dropdownContent.style.display === "block" ? "none" : "block";
};

window.selectMode = function(mode) {
    const modeButton = document.getElementById("mode-button");
    modeButton.innerText = mode;
    window.toggleDropdown();
    
    if (rosConnection) {
        rosConnection.publishControlMode(mode);
    }
};

// Close dropdown when clicking outside
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