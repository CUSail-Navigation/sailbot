// modules/dialManager.js - D3.js dial management
export class DialManager {
    constructor() {
        this.initializeDials();
    }

    initializeDials() {
        this.createSailDial();
        this.createTailDial();
        this.createHeadingDial();
    }

    createSailDial() {
        const width = 90;
        const height = 90;
        const radius = Math.min(width, height) / 2;
        const needleLength = radius * 0.7;

        // Create SVG Container
        const svg = d3
            .select("#dial-container-sail")
            .append("svg")
            .attr("width", width)
            .attr("height", height)
            .append("g")
            .attr("transform", `translate(${width / 2}, ${height / 2})`);

        // Draw Background Circle
        svg.append("circle")
            .attr("r", radius)
            .attr("fill", "lightgray")
            .attr("stroke", "black")
            .attr("stroke-width", 2);

        // Add Tick Marks
        this.addTickMarks(svg, radius, 36);

        // Create the Needle
        this.sailNeedle = svg
            .append("line")
            .attr("x1", 0)
            .attr("y1", 0)
            .attr("x2", 0)
            .attr("y2", -needleLength)
            .attr("stroke", "red")
            .attr("stroke-width", 3)
            .attr("stroke-linecap", "round");

        // Add a Center Circle
        svg.append("circle").attr("r", 5).attr("fill", "black");
    }

    createTailDial() {
        const width = 90;
        const height = 90;
        const radius = Math.min(width, height) / 2;
        const needleLength = radius * 0.7;

        // Create SVG Container
        const svg = d3
            .select("#dial-container-tail")
            .append("svg")
            .attr("width", width)
            .attr("height", height)
            .append("g")
            .attr("transform", `translate(${width / 2}, ${height / 2})`);

        // Draw Background Circle
        svg.append("circle")
            .attr("r", radius)
            .attr("fill", "lightgray")
            .attr("stroke", "black")
            .attr("stroke-width", 2);

        // Add Tick Marks
        this.addTickMarks(svg, radius, 36);

        // Create the Needle
        this.tailNeedle = svg
            .append("line")
            .attr("x1", 0)
            .attr("y1", 0)
            .attr("x2", 0)
            .attr("y2", -needleLength)
            .attr("stroke", "red")
            .attr("stroke-width", 3)
            .attr("stroke-linecap", "round");

        // Add a Center Circle
        svg.append("circle").attr("r", 5).attr("fill", "black");
    }

    createHeadingDial() {
        const width = 90;
        const height = 90;
        const radius = Math.min(width, height) / 2;
        const needleLength = radius * 0.7;

        // Create SVG Container
        const svg = d3
            .select("#dial-container-head")
            .append("svg")
            .attr("width", width)
            .attr("height", height)
            .append("g")
            .attr("transform", `translate(${width / 2}, ${height / 2})`);

        // Draw Background Circle
        svg.append("circle")
            .attr("r", radius)
            .attr("fill", "lightgray")
            .attr("stroke", "black")
            .attr("stroke-width", 2);

        // Add Tick Marks
        this.addTickMarks(svg, radius, 36);

        // Create the Needle
        this.headingNeedle = svg
            .append("line")
            .attr("x1", 0)
            .attr("y1", 0)
            .attr("x2", 0)
            .attr("y2", -needleLength)
            .attr("stroke", "red")
            .attr("stroke-width", 3)
            .attr("stroke-linecap", "round");

        // Add a Center Circle
        svg.append("circle").attr("r", 5).attr("fill", "black");
    }

    addTickMarks(svg, radius, numTicks) {
        const tickLength = 10;
        for (let i = 0; i < numTicks; i++) {
            const angle = (i / numTicks) * 2 * Math.PI;
            const x1 = Math.cos(angle) * (radius - tickLength);
            const y1 = Math.sin(angle) * (radius - tickLength);
            const x2 = Math.cos(angle) * radius;
            const y2 = Math.sin(angle) * radius;

            svg.append("line")
                .attr("x1", x1)
                .attr("y1", y1)
                .attr("x2", x2)
                .attr("y2", y2)
                .attr("stroke", "black")
                .attr("stroke-width", i % 3 === 0 ? 2 : 1);
        }
    }

    updateSailAngle(angle, id) {
        if (this.sailNeedle) {
            this.sailNeedle.transition()
                .duration(500)
                .attr("transform", `rotate(${angle - 90})`);
        }
        document.getElementById(id).innerText = "Angle: " + angle;
    }

    updateTailAngle(angle, id) {
        if (this.tailNeedle) {
            this.tailNeedle.transition()
                .duration(500)
                .attr("transform", `rotate(${angle - 90})`);
        }
        document.getElementById(id).innerText = "Angle: " + angle;
    }

    updateHeadAngle(angle, id) {
        if (this.headingNeedle) {
            this.headingNeedle.transition()
                .duration(500)
                .attr("transform", `rotate(${angle - 90})`);
        }
        document.getElementById(id).innerText = "Angle: " + angle;
    }
}