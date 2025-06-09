// modules/uiManager.js - UI controls and algorithm parameters
export class UIManager {
    constructor(rosConnection, dialManager) {
        this.rosConnection = rosConnection;
        this.dialManager = dialManager;

        this.setupEventListeners();
        this.initializeControlMode();
    }

    setupEventListeners() {
        // ROS connection button
        this.setupROSConnection();

        // Algorithm runtime parameters
        this.setupAlgoParams();
    }

    setupROSConnection() {
        // Connect to ROS button
        document.getElementById('connect-to-ros').addEventListener('click', () => {
            const rosbridgeAddress = document.getElementById('ros-url').value;
            if (rosbridgeAddress) {
                console.log('Connecting to ROS at:', rosbridgeAddress);
                this.rosConnection.connectToROS(rosbridgeAddress);
            } else {
                alert('Please enter a valid ROS URL.');
            }
        });

        // Set default ROS URL if empty
        const rosUrlInput = document.getElementById('ros-url');
        if (!rosUrlInput.value) {
            rosUrlInput.value = 'ws://localhost:9090';
        }
    }

    setupAlgoParams() {
        // No-go zone parameter
        document.getElementById('no-go-zone-submit').addEventListener('click', () => {
            const noGoZoneInput = document.getElementById('no-go-zone-input').value;

            if (noGoZoneInput && !isNaN(noGoZoneInput)) {
                this.rosConnection.publishNoGoZone(parseInt(noGoZoneInput));
                console.log(`Setting no-go zone to: ${noGoZoneInput}°`);
            } else {
                alert('Please enter a valid numeric value for the no-go zone.');
            }
        });

        // Neutral zone parameter
        document.getElementById('neutral-zone-submit').addEventListener('click', () => {
            const neutralZoneInput = document.getElementById('neutral-zone-input').value;

            if (neutralZoneInput && !isNaN(neutralZoneInput)) {
                this.rosConnection.publishNeutralZone(parseInt(neutralZoneInput));
                console.log(`Setting neutral zone to: ${neutralZoneInput}°`);
            } else {
                alert('Please enter a valid numeric value for the neutral zone.');
            }
        });

        // Neutral zone parameter
        document.getElementById('tacking-buffer-submit').addEventListener('click', () => {
            const tackingBufferInput = document.getElementById('tacking-buffer-input').value;

            if (tackingBufferInput && !isNaN(tackingBufferInput)) {
                this.rosConnection.publishTackingBuffer(parseInt(tackingBufferInput));
                console.log(`Setting tacking buffer to: ${tackingBufferInput}°`);
            } else {
                alert('Please enter a valid numeric value for the tacking buffer.');
            }
        });

        // HSV Lower parameter
        document.getElementById('hsv-lower-submit').addEventListener('click', () => {
            const input = document.getElementById('hsv-lower-input').value;

            try {
                const values = input.split(',').map(v => parseInt(v.trim(), 10));

                if (values.length === 3 && values.every(v => !isNaN(v))) {
                    this.rosConnection.publishHsvLower(values);
                    console.log(`Setting HSV lower to: [${values.join(', ')}]`);
                    document.getElementById('hsv-lower-input').value = '';
                } else {
                    alert('Please enter 3 comma-separated integers for HSV lower (e.g. "0, 120, 180").');
                }
            } catch (e) {
                alert('Invalid input format for HSV lower.');
            }
        });

        // HSV Upper parameter
        document.getElementById('hsv-upper-submit').addEventListener('click', () => {
            const input = document.getElementById('hsv-upper-input').value;

            try {
                const values = input.split(',').map(v => parseInt(v.trim(), 10));

                if (values.length === 3 && values.every(v => !isNaN(v))) {
                    this.rosConnection.publishHsvUpper(values);
                    console.log(`Setting HSV upper to: [${values.join(', ')}]`);
                    document.getElementById('hsv-upper-input').value = '';
                } else {
                    alert('Please enter 3 comma-separated integers for HSV upper (e.g. "10, 160, 255").');
                }
            } catch (e) {
                alert('Invalid input format for HSV upper.');
            }
        });

        // Detection Threshold parameter
        document.getElementById('detection-threshold-submit').addEventListener('click', () => {
            const input = document.getElementById('detection-threshold-input').value;

            if (input && !isNaN(input)) {
                this.rosConnection.publishDetectionThreshold(parseInt(input, 10));
                console.log(`Setting detection threshold to: ${input}`);
                document.getElementById('detection-threshold-input').value = '';
            } else {
                alert('Please enter a valid numeric value for the detection threshold.');
            }
        });

        // Allow Enter key to submit parameters
        document.getElementById('no-go-zone-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                document.getElementById('no-go-zone-submit').click();
            }
        });

        document.getElementById('neutral-zone-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                document.getElementById('neutral-zone-submit').click();
            }
        });

        // Allow Enter key for ROS URL
        document.getElementById('ros-url').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                document.getElementById('connect-to-ros').click();
            }
        });
    }

    initializeControlMode() {
        // Initialize current control mode from button text if it exists
        document.addEventListener('DOMContentLoaded', () => {
            const modeButton = document.getElementById("mode-button");
            if (modeButton && modeButton.innerText && modeButton.innerText !== "Select Mode") {
                const currentMode = modeButton.innerText;
                if (this.rosConnection) {
                    this.rosConnection.currentControlMode = currentMode;
                    console.log(`Initialized current control mode from button: ${currentMode}`);
                }
            }
        });
    }

    conditionalRender() {
        // Show/hide UI elements based on current control mode
        const controlModeVal = document.getElementById("control-mode-value").innerText.trim().toLowerCase();
        const algoVals = document.querySelectorAll(".algo-mode");
        const rcVals = document.querySelectorAll(".rc-mode");

        if (controlModeVal === "algorithm" || controlModeVal === "algo") {
            algoVals.forEach(el => {
                el.style.display = "flex";
                el.style.opacity = "1";
            });
            rcVals.forEach(el => {
                el.style.display = "none";
                el.style.opacity = "0.5";
            });
            console.log("UI switched to algorithm mode");
        } else if (controlModeVal === "radio" || controlModeVal === "rc") {
            algoVals.forEach(el => {
                el.style.display = "none";
                el.style.opacity = "0.5";
            });
            rcVals.forEach(el => {
                el.style.display = "flex";
                el.style.opacity = "1";
            });
            console.log("UI switched to radio control mode");
        } else {
            // Default: show both with reduced opacity
            algoVals.forEach(el => {
                el.style.display = "flex";
                el.style.opacity = "0.7";
            });
            rcVals.forEach(el => {
                el.style.display = "flex";
                el.style.opacity = "0.7";
            });
        }
    }

    // Alert when buoy distance value is <= 2
    /*
    alertBuoyDistance() {
        const buoyDistanceEl = document.getElementById('buoy-dist-value');
        const redFlagImg = document.getElementById('red-flag-image');

        if (!buoyDistanceEl || !redFlagImg) return;

        const distance = parseFloat(buoyDistanceEl.innerText);

        redFlagImg.style.display = 'block';
    } */

    updateBuoyBool() {
        const distanceText = document.getElementById("buoy-dist-value").textContent;
        const distance = parseFloat(distanceText);

        const boolElement = document.getElementById("buoy-dist-bool");

        if (!isNaN(distance)) {
            boolElement.textContent = distance <= 2 ? "True" : "False";
        } else {
            boolElement.textContent = "N/A";
        }
    }

    // Utility method to show connection status
    updateConnectionStatus(connected, url = '') {
        const statusElement = document.getElementById('connection-status');
        if (statusElement) {
            if (connected) {
                statusElement.textContent = `Connected to ${url}`;
                statusElement.style.color = 'green';
            } else {
                statusElement.textContent = 'Disconnected';
                statusElement.style.color = 'red';
            }
        }
    }

    // Method to provide user feedback
    showNotification(message, type = 'info') {
        // Create a simple notification system
        const notification = document.createElement('div');
        notification.className = `notification ${type}`;
        notification.textContent = message;
        notification.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                padding: 10px 20px;
                border-radius: 5px;
                z-index: 1000;
                font-weight: bold;
                ${type === 'error' ? 'background: #ff6b6b; color: white;' :
                type === 'success' ? 'background: #51cf66; color: white;' :
                    'background: #339af0; color: white;'}
            `;

        document.body.appendChild(notification);

        // Auto-remove after 3 seconds
        setTimeout(() => {
            if (notification.parentNode) {
                notification.parentNode.removeChild(notification);
            }
        }, 3000);
    }

    // Clear all input fields
    clearAllInputs() {
        const inputs = [
            'waypoint-latitude',
            'waypoint-longitude',
            'buoy-latitude',
            'buoy-longitude',
            'no-go-zone-input',
            'neutral-zone-input'
        ];

        inputs.forEach(id => {
            const element = document.getElementById(id);
            if (element) {
                element.value = '';
            }
        });
    }
}