// modules/rosConnectionViewer.js - Read-only ROS connection (subscribe + waypoint get only)
export class ROSConnectionViewer {
    constructor() {
        this.ros = null;
        this.BASE_THROTTLE_RATE = 1000;

        this.waypointManager = null;
        this.mapFunctions = null;
    }

    setWaypointManager(manager) {
        this.waypointManager = manager;
    }
    setMapFunctions(functions) {
        this.mapFunctions = functions;
    }

    connectToROS(url = 'ws://localhost:9090') {
        this.ros = new ROSLIB.Ros({ url });

        this.ros.on('connection', () => {
            console.log('Viewer connected to rosbridge at:', url);
            this.subscribeToTopics();
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to rosbridge:', error);
        });

        this.ros.on('close', () => {
            console.log('Connection to rosbridge closed.');
        });
    }

    subscribeToTopics() {
        const updateValue = (elementId, value) => {
            const el = document.getElementById(elementId);
            if (el) el.innerText = value;
        };

        const algoRudderTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/algo_rudder',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        algoRudderTopic.subscribe((message) => updateValue('algo-rudder-value', message.data));

        const algoSailTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/algo_sail',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        algoSailTopic.subscribe((message) => updateValue('algo-sail-value', message.data));

        const controlModeTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/control_mode',
            messageType: 'std_msgs/String',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        controlModeTopic.subscribe((message) => {
            updateValue('control-mode-value', message.data);
            this.conditionalRender();
        });

        const eventModeTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/current_mode',
            messageType: 'std_msgs/String',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        eventModeTopic.subscribe((message) => {
            updateValue('event-mode-value', message.data);
            this.conditionalRender();
        });

        const buoyDistTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/buoy_distance',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        buoyDistTopic.subscribe((message) => {
            updateValue('buoy-dist-value', message.data);
            this.updateBuoyBool();
        });

        const radioRudderTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/radio_rudder',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        radioRudderTopic.subscribe((message) => updateValue('radio-rudder-value', message.data));

        const radioSailTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/radio_sail',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        radioSailTopic.subscribe((message) => updateValue('radio-sail-value', message.data));

        const rudderAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/rudder_angle',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        rudderAngleTopic.subscribe((message) => updateValue('rudder-angle-value', message.data));

        const sailTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/sail',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        sailTopic.subscribe((message) => updateValue('sail-value', message.data));

        const gpsTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/gps',
            messageType: 'sensor_msgs/NavSatFix',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        gpsTopic.subscribe((message) => this.parseGpsData(message));

        const imuTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/imu',
            throttle_rate: this.BASE_THROTTLE_RATE,
            messageType: 'geometry_msgs/msg/Vector3',
        });
        imuTopic.subscribe((message) => this.parseImuData(message));

        const actualRudderAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/actual_rudder_angle',
            messageType: 'std_msgs/msg/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        actualRudderAngleTopic.subscribe((message) => {
            updateValue('actual-rudder-angle-value', message.data - 25);
        });

        const windAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/wind',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        windAngleTopic.subscribe((message) => updateValue('wind-angle-value', message.data));

        const actualSailAngleTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/actual_sail_angle',
            messageType: 'std_msgs/msg/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        actualSailAngleTopic.subscribe((message) => updateValue('actual-sail-angle-value', message.data));

        const algoDebugTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/main_algo_debug',
            messageType: 'sailboat_interface/msg/AlgoDebug',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        algoDebugTopic.subscribe((message) => this.parseAlgoDebug(message));

        const droppedPacketsTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sailbot/dropped_packets',
            messageType: 'std_msgs/Int32',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        droppedPacketsTopic.subscribe((message) => updateValue('dropped-packets-value', message.data));

        const currentWaypoint = new ROSLIB.Topic({
            ros: this.ros,
            name: 'sailbot/current_waypoint',
            messageType: 'sensor_msgs/NavSatFix',
            throttle_rate: this.BASE_THROTTLE_RATE,
        });
        currentWaypoint.subscribe(() => {
            if (this.waypointManager) {
                this.waypointManager.syncWaypointQueueFromBackend();
            }
        });

        // Read-only: waypoint service for "get" only (display list)
        const waypointService = new ROSLIB.Service({
            ros: this.ros,
            name: '/sailbot/mutate_waypoint_queue',
            serviceType: 'sailboat_interface/srv/Waypoint',
        });
        if (this.waypointManager) {
            this.waypointManager.setWaypointService(waypointService);
            this.waypointManager.syncWaypointQueueFromBackend();
        }
    }

    conditionalRender() {
        const controlModeValEl = document.getElementById('control-mode-value');
        if (!controlModeValEl) return;
        const controlModeVal = controlModeValEl.innerText.trim().toLowerCase();
        const algoVals = document.querySelectorAll('.algo-mode');
        const rcVals = document.querySelectorAll('.rc-mode');

        if (controlModeVal === 'algorithm' || controlModeVal === 'algo') {
            algoVals.forEach((el) => {
                el.style.display = 'flex';
                el.style.opacity = '1';
            });
            rcVals.forEach((el) => {
                el.style.display = 'none';
                el.style.opacity = '0.5';
            });
        } else if (controlModeVal === 'radio' || controlModeVal === 'rc') {
            algoVals.forEach((el) => {
                el.style.display = 'none';
                el.style.opacity = '0.5';
            });
            rcVals.forEach((el) => {
                el.style.display = 'flex';
                el.style.opacity = '1';
            });
        } else {
            algoVals.forEach((el) => {
                el.style.display = 'flex';
                el.style.opacity = '0.7';
            });
            rcVals.forEach((el) => {
                el.style.display = 'flex';
                el.style.opacity = '0.7';
            });
        }
    }

    updateBuoyBool() {
        const buoyDistEl = document.getElementById('buoy-dist-value');
        const boolEl = document.getElementById('buoy-dist-bool');
        if (!buoyDistEl || !boolEl) return;
        const distance = parseFloat(buoyDistEl.textContent);
        boolEl.textContent = !isNaN(distance) ? (distance <= 2 ? 'True' : 'False') : 'N/A';
    }

    parseGpsData(message) {
        const latitude = message.latitude;
        const longitude = message.longitude;
        const latEl = document.getElementById('latitude-value');
        const lonEl = document.getElementById('longitude-value');
        if (latEl) latEl.innerText = latitude.toFixed(6);
        if (lonEl) lonEl.innerText = longitude.toFixed(6);
        if (this.mapFunctions) {
            this.mapFunctions.updateSailboatPosition(latitude, longitude);
        }
    }

    parseImuData(message) {
        const heading = message.z;
        const formattedHeading = heading.toFixed(6);
        const headingEl = document.getElementById('heading-value');
        if (headingEl) headingEl.innerText = formattedHeading;
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

        if (this.mapFunctions) {
            this.mapFunctions.updateCurrentDestination(currDest.latitude, currDest.longitude);
        }

        const set = (id, val) => {
            const el = document.getElementById(id);
            if (el) el.innerText = val;
        };
        set('tacking-value', tacking);
        set('heading-dir-value', headingDir);
        set('curr-dest-value', `${currDest.latitude.toFixed(6)}, ${currDest.longitude.toFixed(6)}`);
        set('diff-value', diff);
        set('dist-value', dist);
        set('no-go-zone-value', noGoZone);
        set('neutral-zone-value', neutralZone);
    }
}
