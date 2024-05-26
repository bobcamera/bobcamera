function parseSVG(svgString) {
    var parser = new DOMParser();
    var doc = parser.parseFromString(svgString, "image/svg+xml");
    var svg = doc.querySelector('svg');

    var shapes = [];
    var svgDetails = {
        width: parseFloat(svg.getAttribute('width')),
        height: parseFloat(svg.getAttribute('height')),
        shapes: shapes
    };

    // Handle polygons and polylines
    doc.querySelectorAll('polygon, polyline').forEach(element => {
        shapes.push({
            type: element.tagName,
            points: extractPoints(element)
        });
    });

    // Handle rectangles
    doc.querySelectorAll('rect').forEach(rect => {
        var x = parseFloat(rect.getAttribute('x')) || 0;
        var y = parseFloat(rect.getAttribute('y')) || 0;
        var width = parseFloat(rect.getAttribute('width'));
        var height = parseFloat(rect.getAttribute('height'));

        shapes.push({
            type: 'rectangle',
            points: [
                { x: x, y: y },
                { x: x + width, y: y },
                { x: x + width, y: y + height },
                { x: x, y: y + height }
            ]
        });
    });

    // Handle paths with simple commands
    doc.querySelectorAll('path').forEach(path => {
        var d = path.getAttribute('d');
        shapes.push({
            type: 'path',
            points: parsePath(d)
        });
    });

    return svgDetails;
}

// Extract points from polygon and polyline
function extractPoints(element) {
    var points = [];
    var rawPoints = element.getAttribute("points").trim().split(/\s+|,/);
    for (var i = 0; i < rawPoints.length; i += 2) {
        points.push({ x: parseFloat(rawPoints[i]), y: parseFloat(rawPoints[i + 1]) });
    }
    return points;
}

// Basic parsing of path data
function parsePath(d) {
    var commands = d.split(/(?=[LMCZAQ])/);
    var points = [];
    commands.forEach(cmd => {
        var type = cmd[0];
        var args = cmd.slice(1).trim().split(/[ ,]+/);
        switch (type) {
            case 'M': // Moveto
            case 'L': // Lineto
                points.push({ x: parseFloat(args[0]), y: parseFloat(args[1]) });
                break;
            // Add additional cases here for 'C', 'Q', etc., if needed
        }
    });
    return points;
}

function scaleSVG(svgDetails, newWidth, newHeight) {
    var scaleX = newWidth / svgDetails.width;
    var scaleY = newHeight / svgDetails.height;

    // Scale each shape in the SVG
    svgDetails.shapes.forEach(shape => {
        shape.points = shape.points.map(point => ({
            x: point.x * scaleX,
            y: point.y * scaleY
        }));
    });

    // Update the SVG dimensions in the details
    svgDetails.width = newWidth;
    svgDetails.height = newHeight;

    return svgDetails;
}

function getPathPointsOnly(svgDetails) {
    // Filter the shapes array to get only those shapes that are paths
    var paths = svgDetails.shapes.filter(shape => shape.type === 'path');
    
    // Map over the filtered paths to extract only their points
    var pathPoints = paths.map(path => path.points);
    
    return pathPoints;
}

function invertPath(svgDetails) {
    var width = svgDetails.width;
    var height = svgDetails.height;

    // Inverted paths array with details structure
    var invertedSVGDetails = {
        width: svgDetails.width,
        height: svgDetails.height,
        shapes: svgDetails.shapes.map(shape => {
            if (shape.type !== 'path') {
                return shape; // Return non-path shapes unchanged
            }

            // Create a new path that includes the original path inverted within the SVG bounds
            var boundingRectangle = [
                { x: 0, y: 0 },
                { x: width, y: 0 },
                { x: width, y: height },
                { x: 0, y: height }
            ];

            var invertedPoints = boundingRectangle.concat(shape.points);
            invertedPoints.push({ x: 0, y: 0 }); // Close the path back to the start

            return {
                type: 'path',
                points: invertedPoints
            };
        })
    };

    return invertedSVGDetails;
}

function drawSVGOnCanvas2(svgDetails, width, height) {
    scaleSVG(svgDetails, width, height)
    var canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    var ctx = canvas.getContext('2d');

    // Fill the canvas with black
    ctx.fillStyle = 'black';
    ctx.fillRect(0, 0, width, height);

    // Draw the shapes
    ctx.fillStyle = 'white';//;
    svgDetails.shapes.forEach(shape => {
        if (shape.type === 'path') {// || shape.type === 'polygon' || shape.type === 'rectangle') {
            ctx.beginPath();
            shape.points.forEach((point, index) => {
                if (index === 0) {
                    ctx.moveTo(point.x, point.y);
                } else {
                    ctx.lineTo(point.x, point.y);
                }
            });
            ctx.closePath();
            ctx.fill();
        }
    });

    return canvas;
}

function drawSVGOnCanvas(svgDetails, canvas, alpha) {
    var canvasCtx = canvas.getContext('2d');

    if (Object.keys(svgDetails).length === 0 && svgDetails.constructor === Object) {
        canvasCtx.clearRect(0, 0, canvas.width, canvas.height);
        return;
    }

    scaleSVG(svgDetails, canvas.width, canvas.height);

    canvasCtx.fillStyle = 'rgba(0, 0, 0, ' + alpha + ')';
    canvasCtx.fillRect(0, 0, canvas.width, canvas.height);

    let oldGlobalCompositeOperation = canvasCtx.globalCompositeOperation
    canvasCtx.globalCompositeOperation = 'destination-out';
    canvasCtx.fillStyle = 'white';
    svgDetails.shapes.forEach(shape => {
        if (shape.type === 'path') {// || shape.type === 'polygon' || shape.type === 'rectangle') {
            canvasCtx.beginPath();
            shape.points.forEach((point, index) => {
                if (index === 0) {
                    canvasCtx.moveTo(point.x, point.y);
                } else {
                    canvasCtx.lineTo(point.x, point.y);
                }
            });
            canvasCtx.closePath();
            canvasCtx.fill();
        }
    });
    canvasCtx.globalCompositeOperation = oldGlobalCompositeOperation;
}

function drawOnFinalCanvas(canvasCtx, canvas2, alpha, blendMode) {
    let previousAlpha = canvasCtx.globalAlpha;
    let previousCompositeOperation = canvasCtx.globalCompositeOperation;
    canvasCtx.globalAlpha = alpha;
    canvasCtx.globalCompositeOperation = blendMode;
    canvasCtx.drawImage(canvas2, 0, 0);
    canvasCtx.globalAlpha = previousAlpha;
    canvasCtx.globalCompositeOperation = previousCompositeOperation;
}