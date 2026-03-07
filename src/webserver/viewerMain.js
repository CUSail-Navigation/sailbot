// viewerMain.js - Read-only dashboard: map, status, params (no controls)
import { ROSConnectionViewer } from './modules/rosConnectionViewer.js';
import { WaypointManager } from './modules/waypointManager.js';

let rosConnection;
let waypointManager;
let map;
let sailboatMarker;
let currentDestMarker;
let sailPlanCoordinates = [];
let sailPath;

function getRosUrlFromQuery() {
    const params = new URLSearchParams(window.location.search);
    const ros = params.get('ros');
    if (ros && (ros.startsWith('ws://') || ros.startsWith('wss://'))) {
        return ros;
    }
    return 'ws://localhost:9090';
}

function updateSailboatPosition(latitude, longitude, heading = null) {
    const sailboatLocation = { lat: latitude, lng: longitude };

    if (!sailboatMarker) {
        sailboatMarker = new google.maps.Marker({
            position: sailboatLocation,
            map: map,
            title: 'Sailboat Location',
            icon: {
                path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                scale: 5,
                fillColor: '#007bff',
                fillOpacity: 1,
                strokeWeight: 1,
                strokeColor: '#ffffff',
                rotation: 0,
                anchor: new google.maps.Point(0, 2),
            },
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
            fillColor: '#007bff',
            fillOpacity: 1,
            strokeWeight: 1,
            strokeColor: '#ffffff',
            rotation: googleHeading,
            anchor: new google.maps.Point(0, 2),
        });
    }
}

function updateTrail(latitude, longitude) {
    const timestamp = Date.now();
    sailPlanCoordinates.push({ lat: latitude, lng: longitude, timestamp });
    const oneMinuteAgo = Date.now() - 60000;
    sailPlanCoordinates = sailPlanCoordinates.filter((coord) => coord.timestamp >= oneMinuteAgo);
    const currentPath = sailPlanCoordinates.map((coord) => ({ lat: coord.lat, lng: coord.lng }));
    sailPath.setPath(currentPath);
}

function updateCurrentDestination(latitude, longitude) {
    const currentDestLocation = { lat: latitude, lng: longitude };
    if (!currentDestMarker) {
        currentDestMarker = new google.maps.Marker({
            position: currentDestLocation,
            map: map,
            title: 'Current Destination Location',
            icon: {
                path: google.maps.SymbolPath.CIRCLE,
                scale: 8,
                fillColor: '#0000FF',
                fillOpacity: 1,
                strokeWeight: 1,
                strokeColor: '#FFFFFF',
            },
        });
    }
    currentDestMarker.setPosition(currentDestLocation);
    currentDestMarker.setMap(map);
}

window.toggleSection = function (headerElement) {
    const content = headerElement.nextElementSibling;
    const icon = headerElement.querySelector('.toggle-icon');
    if (content.classList.contains('hidden')) {
        content.classList.remove('hidden');
        icon.textContent = '−';
    } else {
        content.classList.add('hidden');
        icon.textContent = '+';
    }
};

window.initMap = function () {
    const defaultLocation = { lat: 42.276848, lng: -71.756323 };
    map = new google.maps.Map(document.getElementById('map'), {
        center: defaultLocation,
        zoom: 15,
    });

    google.maps.event.addListener(map, 'mousemove', (event) => {
        document.getElementById('mouse-pos').innerText =
            `Mouse Latitude: ${event.latLng.lat().toFixed(6)}\nMouse Longitude: ${event.latLng.lng().toFixed(6)}`;
    });

    sailPath = new google.maps.Polyline({
        path: sailPlanCoordinates,
        geodesic: true,
        strokeColor: '#FF0000',
        strokeOpacity: 1.0,
        strokeWeight: 2,
    });
    sailPath.setMap(map);

    waypointManager = new WaypointManager(map, { readOnly: true });
    rosConnection = new ROSConnectionViewer();
    rosConnection.setMapFunctions({
        updateSailboatPosition,
        updateSailboatHeading,
        updateCurrentDestination,
    });
    rosConnection.setWaypointManager(waypointManager);

    const rosUrlInput = document.getElementById('ros-url');
    rosUrlInput.value = getRosUrlFromQuery();

    document.getElementById('connect-to-ros').addEventListener('click', () => {
        const url = rosUrlInput.value.trim();
        if (url) {
            rosConnection.connectToROS(url);
        } else {
            alert('Please enter a ROS URL (e.g. ws://localhost:9090).');
        }
    });
    rosUrlInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') document.getElementById('connect-to-ros').click();
    });
};
