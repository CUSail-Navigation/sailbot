// modules/rosConnection.js - ROS bridge communication
export class ROSConnection {
    constructor() {
        this.ros = null;
        this.controlModeTopic = null;
        this.webserverRudderTopic = null;
        this.webserverSailTopic = null;
        this.noGoZoneTopic = null;
        this.neutralZoneTopic = null;
        this.waypointService = null;
        this.currentControlMode = null;
        this.BASE_THROTTLE_RATE = 1000;
        
        // Dependencies - set by main.js
        this.waypointManager = null;
        this.mapFunctions = null;
        this.dialManager = null;
        this.uiManager = null;
        
        this.setupEventListeners();
    }

    setWaypointManager(manager) { this.waypointManager = manager; }
    setMapFunctions(functions) { this.mapFunctions = functions; }
    setDialManager(manager) { this.dialManager = manager; }
    setUIManager(manager) { this.uiManager = manager; }

    setupEventListeners() {
        // Connect button
        document.getElementById('connect-to-ros').addEventListener('click', () => {
            const rosbridgeAddress = document.getElementById('ros-url').value;
            if (rosbridgeAddress) {
                this.connectToROS(rosbridgeAddress);
            } else {
                console.error('Please enter a valid ROS URL.');
            }
        });

        // Initialize current control mode from button
        document.addEventListener('DOMContentLoaded', () => {
            const modeButton = document.getElementById("mode-button");
            if (modeButton && modeButton.innerText && modeButton.innerText !== "Select Mode") {
                this.currentControlMode = modeButton.innerText;
                console.log(`Initialized current control mode from button: ${this.currentControlMode}`);
            }
        });
    }

    connectToROS(url = 'ws://localhost:9090') {
        this.ros = new ROSLIB.Ros({ url });

        this.ros.on('connection', () => {
            console.log('Connected to rosbridge server at:', url);
            this.subscribeToTopics();
            this.initializePublishers();

            // Republish current control mode on reconnect
            if (this.controlModeTopic && this.currentControlMode) {
                const message = new ROSLIB.Message({ data: this.currentControlMode.toLowerCase() });
                this.controlModeTopic.publish(message);
                console.log(`Published control mode on connect: ${this.currentControlMode.toLowerCase()}`);
            }
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to rosbridge server:', error);
        });

        this.ros.on('close', () => {
            console.log('Connection to rosbridge server closed.');
        });
    }

    subscribeToTopics() {
        // Helper function to update DOM elements
        const updateValue = (elementId, value) => {
            document.getElementById(elementId).innerText = value;
        };

        // Subscribe to algorithm rudder
        const algoRudderTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/algo_rudder',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        algoRudderTopic.subscribe((message) => {
            updateValue('algo-rudder-value', message.data);
        });

        // Subscribe to algorithm sail
        const algoSailTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/algo_sail',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        algoSailTopic.subscribe((message) => {
            updateValue('algo-sail-value', message.data);
        });

        // Subscribe to control mode
        this.controlModeTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/control_mode',
            messageType: 'std_msgs/String',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        this.controlModeTopic.subscribe((message) => {
            updateValue('control-mode-value', message.data);
            if (this.uiManager) {
                this.uiManager.conditionalRender();
            }
        });

        // Subscribe to radio rudder
        const radioRudderTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/radio_rudder',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        radioRudderTopic.subscribe((message) => {
            updateValue('radio-rudder-value', message.data);
        });

        // Subscribe to radio sail
        const radioSailTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/radio_sail',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        radioSailTopic.subscribe((message) => {
            updateValue('radio-sail-value', message.data);
        });

        // Subscribe to rudder angle
        const rudderAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/rudder_angle',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        rudderAngleTopic.subscribe((message) => {
            updateValue('rudder-angle-value', message.data);
        });

        // Subscribe to sail
        const sailTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/sail',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        sailTopic.subscribe((message) => {
            updateValue('sail-value', message.data);
        });

        // Subscribe to GPS
        const gpsTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/gps',
            messageType: 'sensor_msgs/NavSatFix',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        gpsTopic.subscribe((message) => this.parseGpsData(message));

        // Subscribe to IMU
        const imuTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/imu',
            throttle_rate: this.BASE_THROTTLE_RATE,
            messageType: 'geometry_msgs/msg/Vector3'
        });
        imuTopic.subscribe((message) => this.parseImuData(message));

        // Subscribe to actual rudder angle
        const actualRudderAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/actual_rudder_angle',
            messageType: 'std_msgs/msg/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        actualRudderAngleTopic.subscribe((message) => {
            updateValue('actual-tail-angle-value', message.data);
            if (this.dialManager) {
                this.dialManager.updateTailAngle(message.data, "actual-tail-angle-dial");
            }
        });

        // Subscribe to wind angle
        const windAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/wind',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        windAngleTopic.subscribe((message) => {
            updateValue('wind-angle-value', message.data);
        });

        // Subscribe to actual sail angle
        const actualSailAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/actual_sail_angle',
            messageType: 'std_msgs/msg/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        actualSailAngleTopic.subscribe((message) => {
            updateValue('actual-sail-angle-value', message.data);
            if (this.dialManager) {
                this.dialManager.updateSailAngle(message.data, "actual-sail-angle-dial");
            }
        });

        // Subscribe to algorithm debug
        const algoDebugTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/main_algo_debug',
            messageType: 'sailboat_interface/msg/AlgoDebug',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        algoDebugTopic.subscribe((message) => this.parseAlgoDebug(message));

        // Subscribe to dropped packets
        const droppedPacketsTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/dropped_packets',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE
        });
        droppedPacketsTopic.subscribe((message) => {
            updateValue('dropped-packets-value', message.data);
        });

        // Subscribe to current waypoint
        const currentWaypoint = new ROSLIB.Topic({
            ros: this.ros,
            name: 'sailbot/current_waypoint',
            messageType: 'sensor_msgs/NavSatFix',
            throttle_rate: this.BASE_THROTTLE_RATE
        });
        currentWaypoint.subscribe(() => {
            if (this.waypointManager) {
                this.waypointManager.syncWaypointQueueFromBackend();
            }
        });

        // Initialize waypoint service
        this.waypointService = new ROSLIB.Service({
            ros: this.ros,
            name: '/sailbot/mutate_waypoint_queue',
            serviceType: 'sailboat_interface/srv/Waypoint'
        });

        // Pass service to waypoint manager
        if (this.waypointManager) {
            this.waypointManager.setWaypointService(this.waypointService);
        }

        // Sync waypoints on startup
        if (this.waypointManager) {
            this.waypointManager.syncWaypointQueueFromBackend();
        }
    }

    initializePublishers() {
        this.webserverRudderTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/webserver_rudder',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE
        });

        this.webserverSailTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/webserver_sail',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE
        });

        this.noGoZoneTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/no_go_zone',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE
        });

        this.neutralZoneTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/neutral_zone',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE
        });
    }

    parseGpsData(message) {
        const latitude = message.latitude;
        const longitude = message.longitude;

        document.getElementById('latitude-value').innerText = latitude.toFixed(6);
        document.getElementById('longitude-value').innerText = longitude.toFixed(6);

        if (this.mapFunctions) {
            this.mapFunctions.updateSailboatPosition(latitude, longitude);
        }
    }

    parseImuData(message) {
        const heading = message.z;
        const formattedHeading = heading.toFixed(6);

        document.getElementById('heading-value').innerText = formattedHeading;
        
        if (this.dialManager) {
            this.dialManager.updateHeadAngle(formattedHeading, 'heading-value-dial');
        }
        
        if (this.mapFunctions) {
            this.mapFunctions.updateSailboatHeading(heading);
        }
    }

    parseAlgoDebug(message) {
        const tacking = message.tacking;
        const headingDir = message.heading_dir.data;
        const currDest = message.curr_dest;
        const diff = message.diff.data;
        const dist = message.dist_to_dest.data;
        const noGoZone = message.no_go_zone.data;
        const neutralZone = message.neutral_zone.data;

        // Update map with current destination
        if (this.mapFunctions) {
            this.mapFunctions.updateCurrentDestination(currDest.latitude, currDest.longitude);
        }

        document.getElementById('tacking-value').innerText = tacking;
        document.getElementById('heading-dir-value').innerText = headingDir;
        document.getElementById('curr-dest-value').innerText = `${currDest.latitude.toFixed(6)}, ${currDest.longitude.toFixed(6)}`;
        document.getElementById('diff-value').innerText = diff;
        document.getElementById('dist-value').innerText = dist;
        document.getElementById('no-go-zone-value').innerText = noGoZone;
        document.getElementById('neutral-zone-value').innerText = neutralZone;
    }

    publishControlMode(mode) {
        this.currentControlMode = mode;
        
        if (this.controlModeTopic) {
            const message = new ROSLIB.Message({ data: mode.toLowerCase() });
            this.controlModeTopic.publish(message);
            console.log(`Published control mode: ${mode.toLowerCase()}`);
        } else {
            console.warn("Control mode topic not initialized yet");
        }
    }

    publishNoGoZone(value) {
        if (this.noGoZoneTopic) {
            const message = new ROSLIB.Message({ data: parseInt(value, 10) });
            this.noGoZoneTopic.publish(message);
            console.log(`Published to noGoZoneTopic: ${value}`);
        }
    }

    publishNeutralZone(value) {
        if (this.neutralZoneTopic) {
            const message = new ROSLIB.Message({ data: parseInt(value, 10) });
            this.neutralZoneTopic.publish(message);
            console.log(`Published to neutralZoneTopic: ${value}`);
        }
    }

    getWaypointService() {
        return this.waypointService;
    }
}