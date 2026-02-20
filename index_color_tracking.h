#ifndef INDEX_COLOR_TRACKING_H
#define INDEX_COLOR_TRACKING_H

#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
   <title>MAVLink Rover - Pro Suite with Memory</title>
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
      .btn-save { background: #d32f2f !important; width: 92% !important; margin-top: 10px; }
      #filterToggle { background: #0277bd; color: white; }
      #lockToggle { background: #4a148c; color: white; }
      #morphToggle { background: #e65100; color: white; width: 92%; }
      #colorDetect { background: #2e7d32; color: white; width: 92%; margin-bottom: 15px; }
      input[type=range] { width: 100%; margin: 8px 0; height: 10px; }
      .data-box { color: #00e676; font-weight: bold; font-family: monospace; font-size: 1.1em; }
      .ref-data { color: #ffeb3b; font-weight: bold; font-family: monospace; font-size: 1.1em; }
      .label-row { display: flex; justify-content: space-between; font-size: 0.85em; color: #aaa; margin-top: 4px; }
      .target-info { background: #000; padding: 10px; border-radius: 8px; margin: 10px 0; border: 1px solid #444; display: flex; justify-content: space-around; }
      hr { border: 0; border-top: 1px solid #333; margin: 15px 0; }
   </style>
</head>
<body>
    <div class="main-container">
        <canvas id="imageCanvas"></canvas>
        <img id="ShowImage" src="" style="display:none">
        <div style="width: 100%; margin-top: 10px;"><button id="colorDetect">RESTART VIDEO</button></div>
        <canvas id="imageMask"></canvas>

        <div class="controls">
            <button id="saveProfile" class="btn-save">SAVE SLIDER POSITIONS</button>
            <div class="target-info">
                <span>X: <span class="data-box" id="XCMdemo">0</span></span>
                <span>Tether Dist: <span id="distDisplay" style="color:#ffeb3b">0</span></span>
            </div>

            <button id="morphToggle">MORPH CLEAN: OFF</button>
            <div style="display: flex; justify-content: space-between; margin-top:10px;">
                <button id="filterToggle">AUTO-FILTER: ON</button>
                <button id="lockToggle">LOCK TO ZONE: OFF</button>
            </div>
            
            <hr>
            <div class="label-row"><span>X-Steer Smooth (Lerp):</span><span id="xLerpVal" class="data-box">0.20</span></div>
            <input type="range" id="x_lerp" min="0.01" max="1.0" step="0.01" value="0.20">

            <div class="label-row"><span>Tether Tolerance (150=Off):</span><span id="tetherVal" class="data-box">40</span></div>
            <input type="range" id="color_tether" min="5" max="150" value="40">

            <div class="label-row"><span>Growth Delay (Frames):</span><span id="gcVal" class="data-box">30</span></div>
            <input type="range" id="growth_limit_frames" min="1" max="200" value="30">

            <hr>
            <div class="label-row"><span>Box Top (Y):</span><span id="btVal" class="data-box">150</span></div>
            <input type="range" id="box_top" min="0" max="296" value="150">

            <div class="label-row"><span>Box Width (%):</span><span id="bwVal" class="data-box">70</span></div>
            <input type="range" id="box_width" min="10" max="100" value="70">

            <div class="label-row"><span>Exit Tolerance (%):</span><span id="exitVal" class="data-box">20</span></div>
            <input type="range" id="exit_pct" min="0" max="100" value="20">

            <hr>
            <div class="label-row" style="color:#ffeb3b"><span>Ref (Yellow) X/Y:</span><span id="refDisplay" class="ref-data">200, 260</span></div>
            <input type="range" id="ref_x" min="0" max="400" value="200">
            <input type="range" id="ref_y" min="0" max="296" value="260">

            <div class="label-row" style="color:#00e676"><span>Track (Green) X/Y:</span><span id="pDisplay" class="data-box">200, 250</span></div>
            <input type="range" id="x_p" min="0" max="400" value="200">
            <input type="range" id="y_p" min="0" max="296" value="250">

            <div class="label-row"><span>RGB Range:</span><span id="rangeVal" class="data-box">30</span></div>
            <input type="range" id="rgbr" min="5" max="100" value="30">
        </div>
    </div>

<script>
// Sliders and Buttons
const controls = ["x_lerp", "color_tether", "growth_limit_frames", "box_top", "box_width", "exit_pct", "ref_x", "ref_y", "x_p", "y_p", "rgbr"];
const elements = {};
controls.forEach(id => elements[id] = document.getElementById(id));

const saveBtn = document.getElementById('saveProfile');
const colorDetect = document.getElementById('colorDetect'), filterToggle = document.getElementById('filterToggle');
const lockToggle = document.getElementById('lockToggle'), morphToggle = document.getElementById('morphToggle'), ShowImage = document.getElementById('ShowImage');

let b_autoFilter = true, b_pathLock = false, b_morphClean = false;
let currentSmoothedRGBR = 30.0, growthFrameCounter = 0, lastSendTime = 0, isBusy = false, watchdog;

// Persistence Logic
saveBtn.onclick = () => {
    const config = {};
    controls.forEach(id => config[id] = elements[id].value);
    localStorage.setItem('roverConfig', JSON.stringify(config));
    saveBtn.innerHTML = "SAVED!";
    setTimeout(() => { saveBtn.innerHTML = "SAVE SLIDER POSITIONS"; }, 2000);
};

function loadConfig() {
    const saved = localStorage.getItem('roverConfig');
    if (saved) {
        const config = JSON.parse(saved);
        controls.forEach(id => { if(config[id]) elements[id].value = config[id]; });
    }
}

function initOpenCV() { loadConfig(); setTimeout(triggerCapture, 500); }
function resetWatchdog() { clearTimeout(watchdog); watchdog = setTimeout(() => { isBusy = false; triggerCapture(); }, 3000); }
function triggerCapture() { ShowImage.src = location.origin+'/?colorDetect='+Math.random(); }

filterToggle.onclick = function() { b_autoFilter = !b_autoFilter; this.innerHTML = "AUTO-FILTER: " + (b_autoFilter ? "ON" : "OFF"); this.style.background = b_autoFilter ? "#0277bd" : "#455a64"; };
lockToggle.onclick = function() { b_pathLock = !b_pathLock; this.innerHTML = "LOCK TO ZONE: " + (b_pathLock ? "ON" : "OFF"); this.style.background = b_pathLock ? "#4a148c" : "#6a1b9a"; };
morphToggle.onclick = function() { b_morphClean = !b_morphClean; this.innerHTML = "MORPH CLEAN: " + (b_morphClean ? "ON" : "OFF"); this.style.background = b_morphClean ? "#ff6d00" : "#e65100"; };
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

  // Update Logic
  let xLerpFactor = parseFloat(elements.x_lerp.value);
  let boxTopY = parseInt(elements.box_top.value);
  let boxWidthPct = parseInt(elements.box_width.value) / 100;
  let exitTolerance = parseInt(elements.exit_pct.value) / 100;

  // UI Updates
  document.getElementById("xLerpVal").innerHTML = xLerpFactor.toFixed(2);
  document.getElementById("btVal").innerHTML = boxTopY;
  document.getElementById("bwVal").innerHTML = Math.round(boxWidthPct * 100);
  document.getElementById("exitVal").innerHTML = Math.round(exitTolerance * 100);
  document.getElementById("rangeVal").innerHTML = Math.round(currentSmoothedRGBR);
  document.getElementById("refDisplay").innerHTML = elements.ref_x.value + ", " + elements.ref_y.value;
  document.getElementById("pDisplay").innerHTML = elements.x_p.value + ", " + elements.y_p.value;

  let rX = Math.max(0, Math.min(parseInt(elements.ref_x.value), src.cols - 1));
  let rY = Math.max(0, Math.min(parseInt(elements.ref_y.value), src.rows - 1));
  let refPixel = src.ucharPtr(rY, rX);
  
  let pX = Math.max(0, Math.min(parseInt(elements.x_p.value), src.cols - 1));
  let pY = Math.max(0, Math.min(parseInt(elements.y_p.value), src.rows - 1));
  let trackPixel = src.ucharPtr(pY, pX);
  
  let colorDist = Math.sqrt(Math.pow(refPixel[0]-trackPixel[0],2)+Math.pow(refPixel[1]-trackPixel[1],2)+Math.pow(refPixel[2]-trackPixel[2],2));
  document.getElementById("distDisplay").innerHTML = Math.round(colorDist);

  let mask = new cv.Mat();
  let low = new cv.Mat(src.rows, src.cols, src.type(), [trackPixel[0]-currentSmoothedRGBR, trackPixel[1]-currentSmoothedRGBR, trackPixel[2]-currentSmoothedRGBR, 0]);
  let high = new cv.Mat(src.rows, src.cols, src.type(), [trackPixel[0]+currentSmoothedRGBR, trackPixel[1]+currentSmoothedRGBR, trackPixel[2]+currentSmoothedRGBR, 255]);
  cv.inRange(src, low, high, mask);

  if(b_morphClean) {
      let M = cv.Mat.ones(5, 5, cv.CV_8U);
      cv.erode(mask, mask, M); cv.morphologyEx(mask, mask, cv.MORPH_CLOSE, M); M.delete();
  }

  let deadzoneY = src.rows * 0.955; 
  let marginW = (src.cols * (1 - boxWidthPct)) / 2;
  cv.line(src, new cv.Point(0, boxTopY), new cv.Point(src.cols, boxTopY), [150, 150, 150, 255], 1);
  cv.rectangle(src, new cv.Point(marginW, boxTopY), new cv.Point(src.cols - marginW, deadzoneY), [100, 100, 100, 100], 1);
  cv.circle(src, new cv.Point(rX, rY), 6, [255, 235, 59, 255], -1); 
  cv.circle(src, new cv.Point(pX, pY), 4, [0, 255, 0, 255], -1); 

  let contours = new cv.MatVector(), hierarchy = new cv.Mat();
  cv.findContours(mask, contours, hierarchy, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE);

  if(contours.size() > 0){
      let maxArea = 0, maxIdx = -1;
      for(let i=0; i<contours.size(); i++){
          let area = cv.contourArea(contours.get(i));
          if(area > maxArea && area < (src.rows * src.cols * 0.95)) { maxArea = area; maxIdx = i; }
      }
      if(maxIdx >= 0){
          let cnt = contours.get(maxIdx), M = cv.moments(cnt);
          let rawTX = Math.round(M.m10/M.m00), rawTY = Math.round(M.m01/M.m00);
          
          if(b_pathLock) {
              let clampedX = Math.max(marginW, Math.min(rawTX, src.cols - marginW));
              let clampedY = Math.max(boxTopY, Math.min(rawTY, deadzoneY));
              let smoothedX = pX + (clampedX - pX) * xLerpFactor;
              if(!elements.x_p.matches(':active')) elements.x_p.value = Math.round(smoothedX);
              if(!elements.y_p.matches(':active')) elements.y_p.value = Math.round(clampedY);
          }

          let rect = cv.boundingRect(cnt);
          if(b_autoFilter) {
              let pctOutX = (Math.max(0, marginW - rect.x) + Math.max(0, (rect.x + rect.width) - (src.cols - marginW))) / rect.width;
              let pctOutY = Math.max(0, boxTopY - rect.y) / rect.height;

              if(pctOutX <= exitTolerance && pctOutY <= exitTolerance && colorDist <= parseInt(elements.color_tether.value)) {
                  growthFrameCounter++;
                  if(growthFrameCounter >= parseInt(elements.growth_limit_frames.value) && currentSmoothedRGBR < 85) { 
                      currentSmoothedRGBR += 1; growthFrameCounter = 0; 
                  }
              } else {
                  currentSmoothedRGBR = Math.max(8, currentSmoothedRGBR - 1);
                  growthFrameCounter = 0;
              }
          }
          document.getElementById("XCMdemo").innerHTML = Math.round(elements.x_p.value);
          if(Date.now() - lastSendTime > 120){ fetch(location.origin+'/?cm='+elements.x_p.value+';'+elements.y_p.value+';stop'); lastSendTime = Date.now(); }
      }
  }
  cv.imshow('imageMask', mask); cv.imshow('imageCanvas', src);
  src.delete(); mask.delete(); low.delete(); high.delete(); contours.delete(); hierarchy.delete();
  isBusy = false; setTimeout(triggerCapture, 15); 
}
</script>
</body>
</html>
)rawliteral";

#endif
