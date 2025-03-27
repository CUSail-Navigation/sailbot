// logging.js

console.log("logging.js loaded successfully");

const LatestData = {
    latitude: null,
    longitude: null,
    heading: null,
    sailAngle: null,
    rudderAngle: null,
    windAngle: null,
    tacking: null,
    tackingPointLat: null,
    tackingPointLng: null,
    headingDir: null,
    currDestLat: null,
    currDestLng: null,
    diff: null
};


// encapsulate logging functionality
const Logger = (function () {
    // stores logged entries into memory
    let logData = [];
    let loggingInterval = null;

    // API functions: 
    /**
     * Logs a new entry with a timestamp and the provided data
     * @param {*} entry - a key, value pair representing the field to be logged
     */
    function log(entry) {
        console.log("Logging entry: ", entry);
        logData.push({ timestamp: new Date().toISOString(), ...entry });
    }

    /**
     * Clears all logged data
     */
    function clear() {
        alert("Clearing log data.");
        logData = [];
    }

    /**
     * Converts the logged data in memory into a CSV file and triggers a download.
     * @param {*} filename - file name to be used for the downloaded csv
     */
    function download(filename = "log") {
        if (logData.length === 0) {
            alert("No data to download.");
            return;
        }
        alert("Downloading log data as CSV file.");

        // GPT generated:
        const header = Object.keys(logData[0]).join(',') + '\n';
        const rows = logData.map(entry =>
            Object.values(entry).join(',')
        ).join('\n');

        const blob = new Blob([header + rows], { type: 'text/csv;charset=utf-8;' });
        const url = URL.createObjectURL(blob);
        const link = document.createElement("a");

        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        link.setAttribute("href", url);
        link.setAttribute("download", `${filename}-${timestamp}.csv`);
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
    }

    function startLogging(intervalMs = 1000) {
        if (loggingInterval !== null) return; // prevent multiple intervals

        alert("Logging started. Data will be logged every " + intervalMs + "ms.");
        loggingInterval = setInterval(() => {
            log({
                latitude: LatestData.latitude,
                longitude: LatestData.longitude,
                heading: LatestData.heading,
                sailAngle: LatestData.sailAngle,
                rudderAngle: LatestData.rudderAngle,
                windAngle: LatestData.windAngle,
                tacking: LatestData.tacking,
                tackingPointLat: LatestData.tackingPointLat,
                tackingPointLng: LatestData.tackingPointLng,
                headingDir: LatestData.headingDir,
                currDestLat: LatestData.currDestLat,
                currDestLng: LatestData.currDestLng,
                diff: LatestData.diff
            });
        }, intervalMs);
    }

    function stopLogging() {
        alert("Logging stopped.");
        clearInterval(loggingInterval);
        loggingInterval = null;
    }

    return { log, clear, download, startLogging, stopLogging };
})();
