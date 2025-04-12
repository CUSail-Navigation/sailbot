// Store historical data with timestamps
let dataHistory = {
    timestamps: [], // Array of timestamps
    positions: [], // Array of {lat, lng} objects
    controlModes: [], // Array of control mode strings
    sensorData: [], // Array of objects containing all sensor readings
  };
  
  // Track current view index
  let currentViewIndex = 0;
  
  export function initializeScrollBar(rootElementId) {
    const rootElement = document.getElementById(rootElementId);
    if (!rootElement) {
      console.error(`Element with ID ${rootElementId} not found!`);
      return;
    }
  
    // Create scrollbar container
    rootElement.innerHTML = `
      <div class="time-scrollbar-container">
        <div class="scrollbar-label">Time Navigation</div>
        <input type="range" id="time-scrollbar" min="0" max="0" value="0">
        <div class="time-labels">
          <span>Now</span>
          <span>Halfway</span>
          <span>Start of History</span>
        </div>
        <div id="history-status">Live Data</div>
      </div>
    `;
  
    // Set up event listener for the scrollbar
    const scrollbar = document.getElementById("time-scrollbar");
    if (scrollbar) {
      scrollbar.addEventListener("input", handleScrollbarChange);
      console.log("Scrollbar event listener attached successfully");
    } else {
      console.error("Could not find scrollbar element.");
    }
  
    // Initial update of the display
    updateScrollbarDisplay();
  
    // Set up periodic updates for the scrollbar range
    setInterval(updateScrollbarRange, 1000);
  }
  
  /**
   * Updates the scrollbar range to match the actual data history length
   */
  function updateScrollbarRange() {
    const historyLength = dataHistory.timestamps.length;
    if (historyLength === 0) return;
    
    const scrollbar = document.getElementById("time-scrollbar");
    if (scrollbar) {
      // Set max to the current history length
      scrollbar.max = historyLength - 1;
      
      // If we're viewing live data, keep the scrollbar at 0
      if (currentViewIndex === 0) {
        scrollbar.value = 0;
      } else {
        // Make sure the current view index doesn't exceed the history length
        currentViewIndex = Math.min(currentViewIndex, historyLength - 1);
        scrollbar.value = currentViewIndex;
      }
    }
  }
  
  /**
   * Handles when the user changes the scrollbar position
   */
  function handleScrollbarChange(event) {
    const scrollValue = parseInt(event.target.value);
    currentViewIndex = scrollValue;
    console.log("Scrollbar changed to:", currentViewIndex);
    updateHistoryView();
    updateScrollbarDisplay();
  }
  
  /**
   * Updates the scrollbar display based on current state
   */
  function updateScrollbarDisplay() {
    const historyStatus = document.getElementById("history-status");
    if (!historyStatus) {
      console.error("Could not find history status element");
      return;
    }
  
    if (currentViewIndex === 0) {
      historyStatus.textContent = "Live Data";
      historyStatus.style.color = "#4CAF50"; // Green
    } else {
      const secondsAgo = currentViewIndex;
      historyStatus.textContent = `Data from ${secondsAgo} second${
        secondsAgo !== 1 ? "s" : ""
      } ago`;
      historyStatus.style.color = "#f39c12"; // Orange
    }
  }
  
  /**
   * Updates the dashboard display with historical data
   */
  function updateHistoryView() {
    // If showing current data or no history available
    if (currentViewIndex === 0 || dataHistory.timestamps.length === 0) {
      return; // No need to update, current data is already displayed
    }
  
    // Get the index in our history array
    const historyIndex = Math.min(
      currentViewIndex,
      dataHistory.timestamps.length - 1
    );
  
    // Update sailboat position on map
    if (dataHistory.positions[historyIndex] && window.sailboatMarker) {
      window.sailboatMarker.setPosition(dataHistory.positions[historyIndex]);
      window.map.setCenter(dataHistory.positions[historyIndex]);
    }
  
    // Update all the sensor readings
    const sensorData = dataHistory.sensorData[historyIndex];
    if (sensorData) {
      // Update all display values from this historical data point
      for (const [key, value] of Object.entries(sensorData)) {
        const element = document.getElementById(key);
        if (element) {
          element.innerText = value;
        }
      }
  
      // Update dials
      if (sensorData.hasOwnProperty("actualSailAngle")) {
        window.updateSailAngle(
          sensorData.actualSailAngle,
          "actual-sail-angle-dial"
        );
      }
  
      if (sensorData.hasOwnProperty("actualTailAngle")) {
        window.updateTailAngle(
          sensorData.actualTailAngle,
          "actual-tail-angle-dial"
        );
      }
  
      if (sensorData.hasOwnProperty("headingAngle")) {
        window.updateHeadAngle(sensorData.headingAngle, "heading-value-dial");
      }
    }
  }
  
  // Adds current state
  export function recordHistoricalData(data) {
    // Add timestamp
    const timestamp = Date.now();
    dataHistory.timestamps.unshift(timestamp);
  
    // Add position
    if (data.position) {
      dataHistory.positions.unshift(data.position);
    } else {
      // If no position, use the last known position
      dataHistory.positions.unshift(
        dataHistory.positions.length > 0 ? dataHistory.positions[0] : null
      );
    }
  
    // Add sensor data
    dataHistory.sensorData.unshift(data.sensorData);
  
    // Add control mode
    dataHistory.controlModes.unshift(data.controlMode);
  }
  
  export function isViewingLiveData() {
    return currentViewIndex === 0;
  }