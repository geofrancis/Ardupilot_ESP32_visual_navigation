#ifndef INDEX_COLOR_TRACKING_H
#define INDEX_COLOR_TRACKING_H

#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
   <title>MAVLink Rover - Constrained Intelligence</title>
   <meta charset="utf-8">
   <meta name="viewport" content="width=device-width,initial-scale=1">
   <script async src="https://docs.opencv.org/master/opencv.js" type="text/javascript" onload="initOpenCV()"></script>
   <style>
      body { font-family: sans-serif; background: #121212; color: #e0e0e0; margin: 0; padding: 0; text-align: center; }
      .main-container { display: flex; flex-direction: column; align-items: center; width: 100%; max-width: 800px; margin: 0 auto; padding-bottom: 40px; }
      
      canvas { width: 100%; height: auto; border-bottom: 2px solid #333; background: #000; display: block; }
      #imageMask { width: 60%; border: 1px solid #444; margin-top: 10px; border-radius: 4px; }
      
      .controls { width: 95%; background: #1e1e1e; padding: 20px; border-radius: 15px; margin-top: 20px; border: 1px solid #333; box-sizing: border-box; }
      button { padding: 12px; margin: 5px; border-radius: 8px; border: none; cursor: pointer; font-weight: bold; width: 45%; font-size: 0.9em; }
      
      #filterToggle { background: #0277bd; color: white; }
      #lockToggle { background: #4a148c; color: white; }
      #colorDetect { background: #2e7d32; color: white; width: 92%; margin-bottom: 15px; }
      
      input[type=range] { width: 100%; margin: 15px 0; height: 10px; }
      .data-box { color: #00e676; font-weight: bold; font-family: monospace; font-size: 1.1em; }
      .label-row { display: flex; justify-content: space-between; font-size: 0.9em; color: #aaa; margin-top: 5px; }
      .target-info { background: #000; padding: 10px; border-radius: 8px; margin: 10px 0; border: 1px solid #00e676; display: flex; justify-content: space-around; }
   </style>
</head>
<body>
    <div class="main-container">
        <canvas id="imageCanvas"></canvas>
        <img id="ShowImage" src="" style="display:none">

        <div style="width: 100%; margin-top: 10px;">
            <button id="colorDetect">BOOTING...</button>
            <div style="font-size: 0.8em; color: #777;">System Status: <span id="streamStatus">Active</span></div>
        </div>

        <canvas id="imageMask"></canvas>

        <div class="controls">
            <div class="target-info">
                <span>Target: <span class="data-box" id="XCMdemo">0</span>, <span class="data-box" id="YCMdemo">0</span></span>
                <span>Lock: <span id="lockStatus" style="color:#ff5252">OFF</span></span>
            </div>

            <div style="display: flex; justify-content: space-between;">
                <button id="filterToggle">AUTO-FILTER: ON</button>
                <button id="lockToggle">LOCK TO ZONE: OFF</button>
            </div>
            
            <div class="label-row"><span>Growth Delay:</span><span id="gcVal" class="data-box">500</span>ms</div>
            <input type="range" id="growth_limit_ms" min="100" max="5000" step="100" value="500">

            <div class="label-row"><span>Exit Tolerance (%):</span><span id="exitVal" class="data-box">20</span></div>
            <input type="range" id="exit_pct" min="0" max="100" value="20">

            <div class="label-row"><span>Box Width (%):</span><span id="bwVal" class="data-box">70</span></div>
            <input type="range" id="box_width" min="10" max="100" value="70">

            <div class="label-row"><span>Box Top (Y):</span><span id="btVal" class="data-box">150</span></div>
            <input type="range" id="box_top" min="0" max="296" value="150">

            <div class="label-row"><span>RGB Range:</span><span id="rangeVal" class="data-box">30</span></div>
            <input type="range" id="rgbr" min="5" max="100" value="30">

            <div class="label-row"><span>Probe Position:</span><span id="pDisplay" class="data-box">200, 250</span></div>
            <input type="range" id="x_p" min="0" max="400" value="200">
            <input type="range" id="y_p" min="0" max="296" value="250">
        </div>
    </div>

<script>
var colorDetect = document.getElementById('colorDetect');
var filterToggle = document.getElementById('filterToggle');
var lockToggle = document.getElementById('lockToggle');
var ShowImage = document.getElementById('ShowImage');
var statusText = document.getElementById('streamStatus');

var sliderX = document.getElementById("x_p");
var sliderY = document.getElementById("y_p");
var sliderRGB = document.getElementById("rgbr");
var sliderGC = document.getElementById("growth_limit_ms");
var sliderBT = document.getElementById("box_top");
var sliderBW = document.getElementById("box_width");
var sliderExit = document.getElementById("exit_pct");

let b_autoFilter = true;
let b_pathLock = false;
let currentSmoothedRGBR = 30.0; 
let growthStartTime = 0;
let lastSendTime = 0;
let isBusy = false;
let watchdog;

function initOpenCV() {
    statusText.innerHTML = "CV Initialized";
    colorDetect.innerHTML = "RESET STREAM";
    setTimeout(triggerCapture, 500);
}

function resetWatchdog() {
    clearTimeout(watchdog);
    watchdog = setTimeout(() => { isBusy = false; triggerCapture(); }, 3000);
}

function triggerCapture() {
    ShowImage.src = location.origin+'/?colorDetect='+Math.random();
}

filterToggle.onclick = function() {
    b_autoFilter = !b_autoFilter;
    this.innerHTML = "AUTO-FILTER: " + (b_autoFilter ? "ON" : "OFF");
    this.style.background = b_autoFilter ? "#0277bd" : "#455a64";
};

lockToggle.onclick = function() {
    b_pathLock = !b_pathLock;
    this.innerHTML = "LOCK TO ZONE: " + (b_pathLock ? "ON" : "OFF");
    this.style.background = b_pathLock ? "#4a148c" : "#6a1b9a";
    document.getElementById("lockStatus").innerHTML = b_pathLock ? "ACTIVE" : "OFF";
    document.getElementById("lockStatus").style.color = b_pathLock ? "#00e676" : "#ff5252";
};

colorDetect.onclick = () => { isBusy = false; triggerCapture(); };

ShowImage.onload = function () {
  resetWatchdog();
  document.getElementById("imageCanvas").width = document.getElementById("imageMask").width = ShowImage.width;
  document.getElementById("imageCanvas").height = document.getElementById("imageMask").height = ShowImage.height;
  if(!isBusy) DetectImage();        
}

async function DetectImage() {
  isBusy = true;
  let src = cv.imread(ShowImage);
  if (src.empty()) { src.delete(); isBusy = false; return; }

  let targetVal = parseInt(sliderRGB.value);
  let gcLimitMs = parseInt(sliderGC.value);
  let boxTopY = parseInt(sliderBT.value);
  let boxWidthPct = parseInt(sliderBW.value) / 100;
  let exitTolerance = parseInt(sliderExit.value) / 100;
  
  document.getElementById("rangeVal").innerHTML = targetVal;
  document.getElementById("gcVal").innerHTML = gcLimitMs;
  document.getElementById("btVal").innerHTML = boxTopY;
  document.getElementById("bwVal").innerHTML = Math.round(boxWidthPct * 100);
  document.getElementById("exitVal").innerHTML = Math.round(exitTolerance * 100);
  document.getElementById("pDisplay").innerHTML = sliderX.value + ", " + sliderY.value;

  currentSmoothedRGBR = (targetVal * 0.15) + (currentSmoothedRGBR * 0.85);

  // Probe Logic - Clamped to image dimensions
  let pX = Math.max(0, Math.min(parseInt(sliderX.value), src.cols - 1));
  let pY = Math.max(0, Math.min(parseInt(sliderY.value), src.rows - 1));
  let pixel = src.ucharPtr(pY, pX);
  
  let mask = new cv.Mat();
  let low = new cv.Mat(src.rows, src.cols, src.type(), [pixel[0]-currentSmoothedRGBR, pixel[1]-currentSmoothedRGBR, pixel[2]-currentSmoothedRGBR, 0]);
  let high = new cv.Mat(src.rows, src.cols, src.type(), [pixel[0]+currentSmoothedRGBR, pixel[1]+currentSmoothedRGBR, pixel[2]+currentSmoothedRGBR, 255]);
  cv.inRange(src, low, high, mask);

  let deadzoneY = src.rows * 0.955; 
  let marginW = (src.cols * (1 - boxWidthPct)) / 2;

  // Draw Visual Guides
  cv.line(src, new cv.Point(0, boxTopY), new cv.Point(src.cols, boxTopY), [150, 150, 150, 255], 1);
  cv.rectangle(src, new cv.Point(marginW, boxTopY), new cv.Point(src.cols - marginW, deadzoneY), [100, 100, 100, 100], 1);
  cv.circle(src, new cv.Point(pX, pY), 4, [0, 255, 0, 255], -1);

  let contours = new cv.MatVector();
  let hierarchy = new cv.Mat();
  cv.findContours(mask, contours, hierarchy, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE);

  if(contours.size() > 0){
      let maxArea = 0, maxIdx = -1;
      for(let i=0; i<contours.size(); i++){
          let area = cv.contourArea(contours.get(i));
          if(area > maxArea && area < (src.rows * src.cols * 0.95)){
              maxArea = area; maxIdx = i;
          }
      }
      if(maxIdx >= 0){
          let cnt = contours.get(maxIdx);
          let M = cv.moments(cnt);
          let tX = Math.round(M.m10/M.m00);
          let tY = Math.round(M.m01/M.m00);
          
          if(tY < deadzoneY) {
              // --- CLAMPED PROBE MEMORY ---
              if(b_pathLock) {
                  // Constrain probe to the grey box limits
                  let clampedX = Math.max(marginW, Math.min(tX, src.cols - marginW));
                  let clampedY = Math.max(boxTopY, Math.min(tY, deadzoneY));
                  
                  if(!sliderX.matches(':active')) sliderX.value = Math.round(clampedX);
                  if(!sliderY.matches(':active')) sliderY.value = Math.round(clampedY);
              }

              let rect = cv.boundingRect(cnt);
              if(b_autoFilter) {
                  let outLeft = Math.max(0, marginW - rect.x);
                  let outRight = Math.max(0, (rect.x + rect.width) - (src.cols - marginW));
                  let outTop = Math.max(0, boxTopY - rect.y);
                  let pctOutX = (outLeft + outRight) / rect.width;
                  let pctOutY = outTop / rect.height;

                  if(pctOutX <= exitTolerance && pctOutY <= exitTolerance && tY > boxTopY){
                      if(growthStartTime === 0) growthStartTime = Date.now();
                      if(Date.now() - growthStartTime > gcLimitMs && targetVal < 85) { 
                          sliderRGB.value = targetVal + 1;
                          growthStartTime = Date.now();
                      }
                  } else {
                      if(targetVal > 8) sliderRGB.value = targetVal - 1;
                      growthStartTime = 0;
                  }
              }

              document.getElementById("XCMdemo").innerHTML = tX;
              document.getElementById("YCMdemo").innerHTML = tY;

              if(Date.now() - lastSendTime > 120){
                  fetch(location.origin+'/?cm='+tX+';'+tY+';stop');
                  lastSendTime = Date.now();
              }
          }
      }
  }

  cv.imshow('imageMask', mask);
  cv.imshow('imageCanvas', src);
  src.delete(); mask.delete(); low.delete(); high.delete(); contours.delete(); hierarchy.delete();
  isBusy = false;
  setTimeout(triggerCapture, 15); 
}
</script>
</body>
</html>
)rawliteral";

#endif
