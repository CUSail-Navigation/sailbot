// modules/buoyManager.js - Buoy management
export class BuoyManager {
    constructor(map) {
        this.map = map;
        this.buoys = [];
        this.buoyMarkers = {};
        
        this.setupEventListeners();
    }

    setupEventListeners() {
        document.getElementById('submit-buoy').addEventListener('click', () => {
            const latitude = document.getElementById('buoy-latitude').value;
            const longitude = document.getElementById('buoy-longitude').value;

            if (latitude && longitude) {
                this.addBuoy(parseFloat(latitude), parseFloat(longitude));
            } else {
                alert('Please enter both latitude and longitude.');
            }
        });
    }

    addBuoy(latitude, longitude) {
        const buoy = `${latitude},${longitude}`;
        this.buoys.push(buoy);

        const latLng = { lat: latitude, lng: longitude };

        // Add marker for the new buoy on the map
        const marker = new google.maps.Marker({
            position: latLng,
            map: this.map,
            title: `Buoy (${latitude},${longitude})`,
            icon: {
                path: google.maps.SymbolPath.CIRCLE,
                scale: 8,
                fillColor: "#FFA500",
                fillOpacity: 1,
                strokeWeight: 1,
                strokeColor: "#FFFFFF"
            }
        });

        // Add click listener to remove buoy
        marker.addListener("click", () => {
            console.log(`Buoy clicked and removed: ${buoy}`);
            marker.setMap(null); // Remove from map
            delete this.buoyMarkers[buoy]; // Remove from marker tracking
            const index = this.buoys.indexOf(buoy);
            if (index !== -1) {
                this.buoys.splice(index, 1);
            }
        });

        this.buoyMarkers[buoy] = marker;
        console.log(`Buoy added: ${buoy}`);
    }

    getBuoys() {
        return this.buoys;
    }

    getBuoyMarkers() {
        return this.buoyMarkers;
    }
}