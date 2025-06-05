// modules/waypointManager.js - Waypoint management
export class WaypointManager {
    constructor(map) {
        this.map = map;
        this.waypoints = [];
        this.waypointMarkers = {};
        this.waypointPlanCoordinates = [];
        this.waypointPath = null;
        this.draggedIndex = null;
        this.draggedElement = null;
        this.waypointService = null; // Will be set by ROSConnection
        
        this.setupEventListeners();
        this.initializeWaypointPath();
    }

    setWaypointService(service) {
        this.waypointService = service;
    }

    setupEventListeners() {
        // Manual waypoint submission
        document.getElementById('submit-waypoint').addEventListener('click', () => {
            const latitude = document.getElementById('waypoint-latitude').value;
            const longitude = document.getElementById('waypoint-longitude').value;

            if (latitude && longitude) {
                this.addWaypoint(parseFloat(latitude), parseFloat(longitude));
            } else {
                alert('Please enter both latitude and longitude.');
            }
        });
    }

    initializeWaypointPath() {
        this.waypointPath = new google.maps.Polyline({
            path: this.waypointPlanCoordinates,
            geodesic: true,
            strokeColor: "#911084",
            strokeOpacity: 1.0,
            strokeWeight: 2,
        });
        this.waypointPath.setMap(this.map);
    }

    addWaypoint(latitude, longitude) {
        const waypoint = `${latitude},${longitude}`;
        this.waypoints.push(waypoint);
        
        this.addWaypointToQueue(waypoint);
        this.displayWaypoints();

        const latLng = { lat: latitude, lng: longitude };
        
        // Add marker to map
        const marker = new google.maps.Marker({
            position: latLng,
            map: this.map,
            title: `Waypoint (${latitude},${longitude})`,
        });

        // Add click listener for deletion
        marker.addListener("click", () => {
            const index = this.waypoints.indexOf(waypoint);
            if (index !== -1) {
                this.deleteWaypoint(index);
            }
        });

        this.waypointMarkers[waypoint] = marker;
        this.waypointPlanCoordinates.push(latLng);
        
        if (this.waypointPath) {
            this.waypointPath.setPath(this.waypointPlanCoordinates);
        }

        console.log(`Waypoint added: ${waypoint}`);
    }

    addWaypointToQueue(waypoint) {
        const waypointsString = this.waypoints.join(';');

        if (!this.waypointService) {
            console.warn("Waypoint service not available");
            return;
        }

        const request = new ROSLIB.ServiceRequest({
            command: "set",
            argument: waypointsString
        });

        this.waypointService.callService(request, (result) => {
            if (result.success) {
                console.log(result.message);
            } else {
                console.error(result.message);
            }
        });

        this.syncWaypointQueueFromBackend();
    }

    syncWaypointQueueFromBackend() {
        if (!this.waypointService) {
            console.warn("Waypoint service not available for sync");
            return;
        }

        const getRequest = new ROSLIB.ServiceRequest({
            command: "get",
            argument: ""
        });

        this.waypointService.callService(getRequest, (getResult) => {
            if (getResult.success) {
                console.log("Synced waypoint queue from backend:", getResult.message);

                const formatted = getResult.message
                    .replace(/\(/g, '[')
                    .replace(/\)/g, ']')
                    .replace(/'/g, '"');

                const parsed = JSON.parse(formatted);
                this.waypoints = parsed.map(pair => `${pair[0]},${pair[1]}`);
                this.waypointPlanCoordinates = parsed.map(pair => ({ lat: pair[0], lng: pair[1] }));

                // Clear existing markers
                for (const key in this.waypointMarkers) {
                    this.waypointMarkers[key].setMap(null);
                }
                Object.keys(this.waypointMarkers).forEach(key => delete this.waypointMarkers[key]);

                // Add new markers
                parsed.forEach(([lat, lng]) => {
                    const key = `${lat},${lng}`;
                    const marker = new google.maps.Marker({
                        position: { lat, lng },
                        map: this.map,
                        title: `Waypoint (${lat}, ${lng})`,
                    });

                    marker.addListener("click", () => {
                        const index = this.waypoints.indexOf(key);
                        if (index !== -1) {
                            this.deleteWaypoint(index);
                        }
                    });

                    this.waypointMarkers[key] = marker;
                });

                this.displayWaypoints();
                if (this.waypointPath) {
                    this.waypointPath.setPath(this.waypointPlanCoordinates);
                }
            } else {
                console.error("Failed to sync waypoint queue from backend:", getResult.message);
            }
        });
    }

    displayWaypoints() {
        const waypointListElement = document.getElementById('waypoint-list');
        waypointListElement.innerHTML = '';

        this.waypoints.forEach((waypoint, index) => {
            const waypointElement = document.createElement('div');
            waypointElement.classList.add('waypoint-item');
            waypointElement.setAttribute('draggable', true);
            waypointElement.setAttribute('data-index', index);

            const waypointText = document.createElement('div');
            waypointText.classList.add('waypoint-coord');

            const [lat, lng] = waypoint.split(',');

            const latSpan = document.createElement('span');
            latSpan.classList.add('lat');
            latSpan.textContent = lat;

            const lngSpan = document.createElement('span');
            lngSpan.classList.add('lng');
            lngSpan.textContent = lng;

            waypointText.appendChild(latSpan);
            waypointText.appendChild(document.createTextNode(', '));
            waypointText.appendChild(lngSpan);
            waypointElement.appendChild(waypointText);

            const deleteButton = document.createElement('button');
            deleteButton.textContent = 'Delete';
            deleteButton.classList.add('delete-button');
            deleteButton.addEventListener('click', () => {
                this.deleteWaypoint(index);
            });
            waypointElement.appendChild(deleteButton);

            // Drag and drop handlers
            waypointElement.addEventListener('dragstart', (e) => this.handleDragStart(e));
            waypointElement.addEventListener('dragover', (e) => this.handleDragOver(e));
            waypointElement.addEventListener('drop', (e) => this.handleDrop(e));
            waypointElement.addEventListener('dragend', (e) => this.handleDragEnd(e));

            waypointListElement.appendChild(waypointElement);
        });
    }

    deleteWaypoint(index) {
        const waypoint = this.waypoints[index];
        console.log(`Deleting waypoint: ${waypoint}`);

        // Remove marker from map
        if (this.waypointMarkers[waypoint]) {
            this.waypointMarkers[waypoint].setMap(null);
            delete this.waypointMarkers[waypoint];
        }

        // Remove from coordinates array
        this.waypointPlanCoordinates.splice(index, 1);
        if (this.waypointPath) {
            this.waypointPath.setPath(this.waypointPlanCoordinates);
        }

        // Remove from waypoints array
        this.waypoints.splice(index, 1);

        // Update backend
        let request;
        if (this.waypoints.length === 0) {
            request = new ROSLIB.ServiceRequest({
                command: "pop",
                argument: ""
            });
        } else {
            const waypointsString = this.waypoints.join(';');
            request = new ROSLIB.ServiceRequest({
                command: "set",
                argument: waypointsString
            });
        }

        if (this.waypointService) {
            this.waypointService.callService(request, (result) => {
                if (result.success) {
                    console.log(result.message);
                } else {
                    console.error(result.message);
                }
            });
        }

        this.syncWaypointQueueFromBackend();
        this.displayWaypoints();
    }

    handleDragStart(event) {
        this.draggedIndex = parseInt(event.target.getAttribute('data-index'));
        this.draggedElement = event.target;
        event.dataTransfer.effectAllowed = 'move';
        event.target.classList.add('dragging');
    }

    handleDragOver(event) {
        event.preventDefault();
        event.dataTransfer.dropEffect = 'move';
    }

    handleDrop(event) {
        event.preventDefault();
        const targetElement = event.target.closest('.waypoint-item');
        if (!targetElement) return;
        
        const targetIndex = parseInt(targetElement.getAttribute('data-index'));

        if (this.draggedIndex !== null && targetIndex !== null && this.draggedIndex !== targetIndex) {
            const movedItem = this.waypoints[this.draggedIndex];
            this.waypoints.splice(this.draggedIndex, 1);
            this.waypoints.splice(targetIndex, 0, movedItem);
            this.displayWaypoints();
        }
    }

    handleDragEnd(event) {
        if (this.draggedElement) {
            this.draggedElement.classList.remove('dragging');
        }
        this.draggedElement = null;
        this.draggedIndex = null;
        
        // Update backend with new order
        const waypointsString = this.waypoints.join(';');
        const request = new ROSLIB.ServiceRequest({
            command: "set",
            argument: waypointsString
        });

        if (this.waypointService) {
            this.waypointService.callService(request, (result) => {
                if (result.success) {
                    console.log(result.message);
                } else {
                    console.error(result.message);
                }
            });
        }
        
        this.syncWaypointQueueFromBackend();
    }
}