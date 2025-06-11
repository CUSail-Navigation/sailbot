// modules/mapManager.js - Google Maps management
export class MapManager {
    constructor() {
        console.log("constructing map manager...");
        this.map = null;
        this.sailboatMarker = null;
        this.currentDestMarker = null;
        this.sailPlanCoordinates = [];
        this.sailPath = null;
        this.onWaypointDoubleClick = null; // Callback for waypoint creation
    }

    initMap() {
        console.log("Initializing map...")
        const defaultLocation = { lat: 42.45, lng: -76.474 }; // Ithaca, NY

        this.map = new google.maps.Map(document.getElementById("map"), {
            center: defaultLocation,
            zoom: 12,
        });

        // Mouse position tracking
        google.maps.event.addListener(this.map, "mousemove", (event) => {
            document.getElementById("mouse-pos").innerText =
                `Mouse Latitude: ${event.latLng.lat().toFixed(6)}
                Mouse Longitude: ${event.latLng.lng().toFixed(6)}`;
        });

        // Double-click for waypoint creation
        this.map.addListener("dblclick", (e) => {
            const latitude = e.latLng.lat();
            const longitude = e.latLng.lng();

            if (latitude && longitude && this.onWaypointDoubleClick) {
                this.onWaypointDoubleClick(latitude, longitude);
            }
        });

        // Initialize sailboat trail
        this.sailPath = new google.maps.Polyline({
            path: this.sailPlanCoordinates,
            geodesic: true,
            strokeColor: "#FF0000",
            strokeOpacity: 1.0,
            strokeWeight: 2,
        });

        this.sailPath.setMap(this.map);
    }

    setWaypointDoubleClickHandler(callback) {
        this.onWaypointDoubleClick = callback;
    }

    updateSailboatPosition(latitude, longitude, heading = null) {
        const sailboatLocation = { lat: latitude, lng: longitude };

        if (!this.sailboatMarker) {
            this.sailboatMarker = new google.maps.Marker({
                position: sailboatLocation,
                map: this.map,
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
            this.sailboatMarker.setPosition(sailboatLocation);
            this.updateTrail(latitude, longitude);
        }

        if (heading !== null) {
            this.updateSailboatHeading(heading);
        }
    }

    updateSailboatHeading(heading) {
        if (this.sailboatMarker) {
            const googleHeading = (90 - heading + 360) % 360;
            this.sailboatMarker.setIcon({
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

    updateTrail(latitude, longitude) {
        const timestamp = Date.now();
        this.sailPlanCoordinates.push({ lat: latitude, lng: longitude, timestamp });

        // Filter out points older than 600 seconds
        const oneMinuteAgo = Date.now() - 600000;
        this.sailPlanCoordinates = this.sailPlanCoordinates.filter(coord => coord.timestamp >= oneMinuteAgo);

        // Update the polyline
        const currentPath = this.sailPlanCoordinates.map(coord => ({ lat: coord.lat, lng: coord.lng }));
        this.sailPath.setPath(currentPath);
    }

    updateCurrentDestination(latitude, longitude) {
        const currentDestLocation = { lat: latitude, lng: longitude };
        
        if (!this.currentDestMarker) {
            this.currentDestMarker = new google.maps.Marker({
                position: currentDestLocation,
                map: this.map,
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
        
        this.currentDestMarker.setPosition(currentDestLocation);
        this.currentDestMarker.setMap(this.map);
    }

    getMap() {
        return this.map;
    }
}