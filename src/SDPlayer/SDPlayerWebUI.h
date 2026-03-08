#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <FS.h>
#include <functional>

// Forward declarations
class Audio;
class SDPlayerOLED;

// Minimal HTML/CSS/JS to mimic the screenshot layout.
// Page polls /sdplayer/api/list every ~1s.

static const char SDPLAYER_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>SD Player - OLED Style 8</title>
<style>
  * { box-sizing: border-box; }
  body { font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 10px; background: linear-gradient(135deg, #1e3c72, #2a5298); color: white; min-height: 100vh; }
  .container { max-width: 1200px; margin: 0 auto; }
  
  /* OLED Style 8 Header */
  .oled-header { background: rgba(0,0,0,0.8); border-radius: 12px; padding: 20px; margin-bottom: 20px; border: 2px solid #4CAF50; }
  .status-row { display: flex; align-items: center; gap: 15px; margin-bottom: 15px; flex-wrap: wrap; }
  .status-icon { font-size: 28px; }
  .playing { color: #4CAF50; } .paused { color: #FF9800; } .stopped { color: #f44336; }
  .artist-title { font-size: 18px; font-weight: bold; flex: 1; min-width: 200px; }
  .speaker-vol { display: flex; align-items: center; gap: 8px; font-weight: bold; }
  .speaker-icon { font-size: 20px; color: #4CAF50; }
  .speaker-muted { color: #f44336; }
  
  .track-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px; flex-wrap: wrap; }
  .track-info { font-size: 16px; flex: 1; }
  .codec-info { font-size: 14px; color: #bbb; }
  
  .progress-row { margin-bottom: 15px; }
  .progress-bar { width: 100%; height: 8px; background: #333; border-radius: 4px; overflow: hidden; cursor: pointer; position: relative; }
  .progress-bar:hover { height: 12px; transition: height 0.2s; }
  .progress-fill { height: 100%; background: linear-gradient(90deg, #4CAF50, #81C784); width: 0%; transition: width 0.3s; }
  .progress-time { display: flex; justify-content: space-between; font-size: 12px; color: #ccc; margin-top: 5px; }
  
  .album-row { color: #bbb; font-size: 14px; }
  
  /* Seek Controls with Double-Click */
  .seek-controls { background: rgba(0,0,0,0.6); border-radius: 10px; padding: 15px; margin-bottom: 20px; }
  .seek-title { text-align: center; margin-bottom: 15px; font-weight: bold; color: #4CAF50; }
  .seek-buttons { display: flex; justify-content: center; gap: 10px; flex-wrap: wrap; }
  .seek-btn { padding: 12px 16px; border: none; border-radius: 8px; color: white; cursor: pointer; font-weight: bold; transition: all 0.2s; min-width: 70px; }
  .seek-btn:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
  .seek-btn:active { transform: scale(0.95); }
  .seek-large { background: #2196F3; }
  .seek-medium { background: #4CAF50; }
  .seek-small { background: #FF9800; }
  
  /* Control Buttons */
  .control-panel { background: rgba(0,0,0,0.6); border-radius: 10px; padding: 20px; margin-bottom: 20px; }
  .control-row { display: flex; justify-content: center; gap: 10px; margin-bottom: 15px; flex-wrap: wrap; }
  .ctrl-btn { padding: 14px 20px; border: none; border-radius: 8px; color: white; cursor: pointer; font-weight: bold; transition: all 0.2s; }
  .ctrl-btn:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
  .play-btn { background: #4CAF50; }
  .pause-btn { background: #FF9800; }
  .stop-btn { background: #f44336; }
  .nav-btn { background: #2196F3; }
  .util-btn { background: #9C27B0; }
  
  /* Analog VU Meters */
  .vu-meters-section { background: rgba(0,0,0,0.6); border-radius: 10px; padding: 20px; margin-bottom: 20px; }
  .vu-container { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 20px; }
  .vu-meter { position: relative; width: 120px; height: 120px; }
  .vu-circle { width: 100%; height: 100%; border-radius: 50%; background: conic-gradient(from 180deg, #333 0deg, #333 180deg, transparent 180deg); position: relative; overflow: hidden; border: 3px solid #555; }
  .vu-fill { position: absolute; top: 0; left: 0; width: 100%; height: 100%; border-radius: 50%; background: conic-gradient(from 180deg, #f44336 0deg, #FF9800 60deg, #4CAF50 120deg, #333 180deg); clip-path: polygon(50% 50%, 50% 0%, 100% 0%, 100% 100%, 0% 100%, 0% 0%, 50% 0%); transform-origin: center; transition: transform 0.1s ease; }
  .vu-label { position: absolute; bottom: 15px; left: 50%; transform: translateX(-50%); color: #ccc; font-weight: bold; font-size: 14px; }
  .vu-value { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: white; font-weight: bold; font-size: 16px; }
  .vu-needle { position: absolute; top: 50%; left: 50%; width: 2px; height: 50px; background: #fff; transform-origin: bottom center; transform: translate(-50%, -100%) rotate(-90deg); border-radius: 1px; box-shadow: 0 0 5px rgba(255,255,255,0.5); transition: transform 0.1s ease; }
  .control-with-vu { display: flex; align-items: center; justify-content: space-between; gap: 20px; }
  .control-buttons { flex: 1; text-align: center; }
  
  /* Volume Control */
  .volume-control { display: flex; align-items: center; justify-content: center; gap: 15px; margin: 20px 0; }
  .vol-slider { flex: 1; max-width: 300px; height: 8px; border-radius: 4px; background: #333; outline: none; }
  .vol-slider::-webkit-slider-thumb { appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #4CAF50; cursor: pointer; }
  
  /* File Browser */
  .file-browser { background: rgba(0,0,0,0.6); border-radius: 10px; padding: 20px; margin-bottom: 20px; }
  .browser-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px; }
  .path-display { background: rgba(255,255,255,0.1); padding: 10px; border-radius: 6px; font-family: monospace; }
  .file-list { max-height: 400px; overflow-y: auto; border-radius: 8px; }
  .file-item { display: flex; align-items: center; gap: 12px; padding: 12px; margin: 4px 0; border-radius: 8px; cursor: pointer; transition: all 0.2s; }
  .file-item:hover { background: rgba(255,255,255,0.1); }
  .file-item.selected { background: rgba(76, 175, 80, 0.3); border: 1px solid #4CAF50; }
  .file-icon { font-size: 20px; width: 25px; text-align: center; }
  .file-name { flex: 1; }
  .track-number { color: #4CAF50; font-weight: bold; min-width: 40px; background: rgba(76, 175, 80, 0.2); padding: 4px 8px; border-radius: 4px; text-align: center; font-family: monospace; }
  .file-item:hover .track-number { background: rgba(76, 175, 80, 0.4); }
  .delete-btn { padding: 6px 12px; background: #f44336; border: none; border-radius: 6px; color: white; cursor: pointer; font-weight: bold; transition: all 0.2s; }
  .delete-btn:hover { background: #d32f2f; transform: scale(1.05); }
  .delete-btn:active { transform: scale(0.95); }
  
  /* DLNA Button */
  .dlna-btn { background: linear-gradient(135deg, #6a1b9a, #9c27b0) !important; border: 1px solid #ce93d8 !important; }
  .dlna-btn:hover { background: linear-gradient(135deg, #9c27b0, #ba68c8) !important; transform: translateY(-2px) !important; }
  .dlna-btn.active { background: linear-gradient(135deg, #1b5e20, #2e7d32) !important; border-color: #a5d6a7 !important; }
  .dlna-bar { background: rgba(106,27,154,0.25); border: 1px solid #9c27b0; border-radius: 8px; padding: 8px 14px; margin-top: 10px; display: none; font-size: 13px; color: #e1bee7; }
  .dlna-bar.show { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
  
  /* Keyboard Track Selection */
  .track-selector { position: fixed; top: 50%; left: 50%; transform: translate(-50%, -50%); background: rgba(0,0,0,0.9); color: white; padding: 20px 30px; border-radius: 12px; font-size: 24px; font-weight: bold; border: 2px solid #4CAF50; display: none; z-index: 1000; box-shadow: 0 8px 32px rgba(0,0,0,0.8); }
  .track-selector.show { display: block; animation: fadeIn 0.2s ease; }
  @keyframes fadeIn { from { opacity: 0; transform: translate(-50%, -50%) scale(0.8); } to { opacity: 1; transform: translate(-50%, -50%) scale(1); } }
  
  /* Back Button */
  .back-section { text-align: center; padding: 20px; }
  .back-btn { padding: 16px 32px; background: #f44336; border: none; border-radius: 8px; color: white; font-weight: bold; cursor: pointer; font-size: 16px; }
  .back-btn:hover { background: #d32f2f; transform: translateY(-2px); }
  
  /* Responsive */
  @media (max-width: 768px) {
    .status-row, .track-row { flex-direction: column; align-items: flex-start; }
    .seek-buttons { gap: 5px; }
    .seek-btn { min-width: 60px; padding: 10px 12px; font-size: 14px; }
    .control-with-vu { flex-direction: column; }
    .vu-meter { width: 80px; height: 80px; }
    .control-row { gap: 8px; }
    .ctrl-btn { padding: 12px 16px; font-size: 14px; }
  }
</style>
</head>
<body>
<div class="container">
  <!-- OLED Style 8 Header -->
  <div class="oled-header">
    <div class="status-row">
      <div id="statusIcon" class="status-icon stopped">⏹️</div>
      <div id="artistTitle" class="artist-title">ARTYSTA - Track Title</div>
      <div class="speaker-vol">
        <div id="speakerIcon" class="speaker-icon">🔊</div>
        <span id="volDisplay">7</span>
      </div>
    </div>
    
    <div class="track-row">
      <div id="trackInfo" class="track-info">🎵 1. Waiting for track...</div>
      <div id="codecInfo" class="codec-info">MP3/320kbps/44.1kHz</div>
    </div>
    
    <div class="progress-row">
      <div class="progress-bar" onclick="seekToPosition(event)">
        <div id="progressFill" class="progress-fill"></div>
      </div>
      <div class="progress-time">
        <span id="currentTime">0:00</span>
        <span id="totalTime">0:00</span>
      </div>
    </div>
    
    <div id="albumInfo" class="album-row">📁 ALBUM - /music</div>
  </div>
  
  <!-- Seek Controls with Double-Click -->
  <div class="seek-controls">
    <div class="seek-title">🎯 SEEK CONTROLS (1 click = seek, 2 quick clicks = track nav)</div>
    <div class="seek-buttons">
      <button class="seek-btn seek-large" onclick="handleSeek(-30)">⏪ -30s</button>
      <button class="seek-btn seek-medium" onclick="handleSeek(-10)">⏪ -10s</button>
      <button class="seek-btn seek-small" onclick="handleSeek(-5)">⬅️ -5s</button>
      <button class="seek-btn seek-small" onclick="handleSeek(5)">➡️ +5s</button>
      <button class="seek-btn seek-medium" onclick="handleSeek(10)">⏩ +10s</button>
      <button class="seek-btn seek-large" onclick="handleSeek(30)">⏩ +30s</button>
    </div>
  </div>
  
  <!-- Control Panel with Analog VU Meters -->
  <div class="control-panel">
    <div class="control-with-vu">
      <!-- Left VU Meter -->
      <div class="vu-meter">
        <div class="vu-circle">
          <div id="vuFillL" class="vu-fill"></div>
          <div id="vuNeedleL" class="vu-needle"></div>
          <div id="vuValueL" class="vu-value">0</div>
        </div>
        <div class="vu-label">L</div>
      </div>
      
      <!-- Control Buttons -->
      <div class="control-buttons">
        <div class="control-row">
          <button class="ctrl-btn play-btn" onclick="post('/sdplayer/api/playSelected')">▶️ Play</button>
          <button id="pauseBtn" class="ctrl-btn pause-btn" onclick="post('/sdplayer/api/pause')">⏸️ Pause</button>
          <button class="ctrl-btn stop-btn" onclick="post('/sdplayer/api/stop')">⏹️ Stop</button>
          <button class="ctrl-btn nav-btn" onclick="post('/sdplayer/api/prev')">⏮️ Prev</button>
          <button class="ctrl-btn nav-btn" onclick="post('/sdplayer/api/next')">⏭️ Next</button>
        </div>
        
        <!-- Control buttons without UP/DOWN scroll -->
        <div class="control-row">
          <button class="ctrl-btn util-btn" onclick="post('/sdplayer/api/up')">📁 Up Dir</button>
          <button class="ctrl-btn util-btn" onclick="refresh()">🔄 Refresh</button>
          <button class="ctrl-btn util-btn" onclick="post('/sdplayer/api/nextStyle')">🎨 Style</button>
          <button class="ctrl-btn dlna-btn" id="dlnaBtn" onclick="dlnaAction()">📡 DLNA</button>
        </div>
        <!-- DLNA info bar (widoczna gdy tryb DLNA aktywny) -->
        <div class="dlna-bar" id="dlnaBar">
          <span>📡 TRYB DLNA</span>
          <span id="dlnaTrackCount">0 utworów</span>
          <button class="ctrl-btn util-btn" style="padding:4px 10px;font-size:12px" onclick="dlnaBack()">⬅ Wróć do SD</button>
        </div>
        
        <!-- NOWY: Przełącznik stylów OLED -->
        <div class="control-row">
          <button class="ctrl-btn util-btn" onclick="setOledStyle(1)">S1</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(2)">S2</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(3)">S3</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(4)">S4</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(5)">S5</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(6)">S6</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(7)">S7</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(8)">S8</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(9)">S9</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(10)">S10</button>
        </div>
        <div class="control-row">
          <button class="ctrl-btn util-btn" onclick="setOledStyle(11)">S11</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(12)">S12</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(13)">S13</button>
          <button class="ctrl-btn util-btn" onclick="setOledStyle(14)">S14</button>
          <span id="currentStyleInfo" style="color: #4CAF50; font-weight: bold; margin-left: 10px;">Style: ?</span>
        </div>
      </div>
      
      <!-- Right VU Meter -->
      <div class="vu-meter">
        <div class="vu-circle">
          <div id="vuFillR" class="vu-fill"></div>
          <div id="vuNeedleR" class="vu-needle"></div>
          <div id="vuValueR" class="vu-value">0</div>
        </div>
        <div class="vu-label">R</div>
      </div>
    </div>
    
    <!-- Volume Control (moved below VU meters) -->
    <div class="volume-control">
      <div class="speaker-icon">🔊</div>
      <input id="volSlider" class="vol-slider" type="range" min="0" max="21" value="7" oninput="setVol(this.value)"/>
      <div id="volText">7</div>
    </div>
  </div>
  
  <!-- File Browser -->
  <div class="file-browser">
    <div class="browser-header">
      <h3>📁 File Browser</h3>
      <div style="font-size: 12px; color: #bbb;">⌨️ Press 1-9 or type track number to play</div>
    </div>
    <div class="path-display">📁 <span id="currentPath">/</span></div>
    <div id="fileList" class="file-list"></div>
  </div>
  
  <!-- Back Button -->
  <div class="back-section">
    <button class="back-btn" onclick="back()">🏠 Back to Menu</button>
  </div>
  
  <!-- Track Selector Overlay -->
  <div id="trackSelector" class="track-selector">
    <div>🎵 Track: <span id="trackNumber">1</span></div>
    <div style="font-size: 14px; margin-top: 8px; opacity: 0.7;">Press number to select track (1-100)</div>
  </div>
</div>

<script>
console.log('=== SD Player OLED Style 8 JavaScript ===');
let data = null;
let refreshTimer = null;
let lastClickTime = 0;
let clickTimeout = null;
let currentTrackIndex = 0;
let totalTracks = 0;

// Keyboard track selection
let typedNumber = '';
let numberTimeout = null;
let trackSelectorVisible = false;

// Double-click seek logic (matching IR remote)
function handleSeek(seconds) {
  const now = Date.now();
  const timeDiff = now - lastClickTime;
  
  console.log('Seek clicked:', seconds, 'Time diff:', timeDiff);
  
  if (timeDiff < 2000 && clickTimeout) { // Double click within 2 seconds
    console.log('Double-click detected! Track navigation');
    clearTimeout(clickTimeout);
    clickTimeout = null;
    
    // Double click = track navigation
    if (seconds < 0) {
      post('/sdplayer/api/prev');
    } else {
      post('/sdplayer/api/next');
    }
  } else {
    // Single click = seek (with delay to detect potential double-click)
    clickTimeout = setTimeout(() => {
      console.log('Single-click confirmed! Seeking:', seconds);
      fetch('/sdplayer/api/seek?s=' + seconds, {method: 'POST'})
        .then(() => refresh())
        .catch(e => console.error('Seek error:', e));
      clickTimeout = null;
    }, 500); // 500ms delay to allow double-click detection
  }
  
  lastClickTime = now;
}

// Seek to position by clicking progress bar
function seekToPosition(event) {
  if (!data || !data.totalTime || data.totalTime <= 0) {
    console.log('Cannot seek - no total time available');
    return;
  }
  
  const progressBar = event.currentTarget;
  const rect = progressBar.getBoundingClientRect();
  const clickX = event.clientX - rect.left;
  const progressPercent = clickX / rect.width;
  const targetSeconds = Math.floor(data.totalTime * progressPercent);
  
  console.log('Progress bar clicked:', progressPercent, 'Target:', targetSeconds, 's');
  
  fetch('/sdplayer/api/seek?pos=' + targetSeconds, {method: 'POST'})
    .then(() => {
      console.log('Absolute seek successful');
      refresh();
    })
    .catch(e => console.error('Seek error:', e));
}

function post(url) {
  console.log('POST:', url);
  fetch(url, {method: 'POST'})
    .then(r => {
      console.log('POST response:', r.status);
      refresh();
    })
    .catch(e => console.error('POST error:', e));
}

function setVol(v) {
  document.getElementById('volDisplay').innerText = v;
  document.getElementById('volText').innerText = v;
  
  // Update speaker icon based on volume
  const speakerIcon = document.getElementById('speakerIcon');
  if (v == 0) {
    speakerIcon.innerHTML = '🔇';
    speakerIcon.className = 'speaker-icon speaker-muted';
  } else {
    speakerIcon.innerHTML = '🔊';
    speakerIcon.className = 'speaker-icon';
  }
  
  fetch('/sdplayer/api/vol?v=' + encodeURIComponent(v), {method: 'POST'});
}

function back() {
  console.log('Back to menu clicked');
  if (refreshTimer) clearInterval(refreshTimer);
  fetch('/sdplayer/api/back', {method: 'POST'})
    .then(() => {
      console.log('Redirecting to /');
      location.href = '/';
    })
    .catch(e => console.error('Back error:', e));
}

// Artist/Title parsing (matching OLED Style 8)
function parseArtistTitle(filename) {
  if (!filename) return { artist: 'ARTYSTA', title: 'Track Title' };
  
  // Remove extension
  let name = filename.replace(/\.[^/.]+$/, '');
  
  // Try different separators
  const separators = [' - ', '_-_', ' – ', ' — '];
  for (let sep of separators) {
    if (name.includes(sep)) {
      const parts = name.split(sep);
      if (parts.length >= 2) {
        return {
          artist: parts[0].trim() || 'ARTYSTA',
          title: parts.slice(1).join(sep).trim() || 'Track Title'
        };
      }
    }
  }
  
  return { artist: 'ARTYSTA', title: name || 'Track Title' };
}

// Format time seconds to MM:SS
function formatTime(seconds) {
  if (!seconds || seconds < 0) return '0:00';
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  return `${mins}:${secs.toString().padStart(2, '0')}`;
}

// Update Analog VU Meters (like original radio style)
function updateVUMeters(vuL, vuR) {
  // Convert VU values from 0-255 to 0-100 percentage
  const vuLPercent = Math.min((vuL / 255) * 100, 100);
  const vuRPercent = Math.min((vuR / 255) * 100, 100);
  
  // When no music playing, set to 0
  const isPlaying = data && (data.status === 'Playing');
  const finalVuL = isPlaying ? vuLPercent : 0;
  const finalVuR = isPlaying ? vuRPercent : 0;
  
  // Update Left VU Meter
  const needleLangle = -90 + (finalVuL * 1.8); // -90 to +90 degrees
  const fillLangle = Math.max(finalVuL * 1.8, isPlaying ? 5 : 0); // Minimum 5 degrees when playing
  
  document.getElementById('vuNeedleL').style.transform = 
    `translate(-50%, -100%) rotate(${needleLangle}deg)`;
  document.getElementById('vuFillL').style.transform = 
    `rotate(${fillLangle}deg)`;
  document.getElementById('vuValueL').innerText = Math.round(finalVuL);
  
  // Update Right VU Meter  
  const needleRangle = -90 + (finalVuR * 1.8); // -90 to +90 degrees
  const fillRangle = Math.max(finalVuR * 1.8, isPlaying ? 5 : 0); // Minimum 5 degrees when playing
  
  document.getElementById('vuNeedleR').style.transform = 
    `translate(-50%, -100%) rotate(${needleRangle}deg)`;
  document.getElementById('vuFillR').style.transform = 
    `rotate(${fillRangle}deg)`;
  document.getElementById('vuValueR').innerText = Math.round(finalVuR);
  
  // Color coding based on level (like real VU meters)
  const vuFillL = document.getElementById('vuFillL');
  const vuFillR = document.getElementById('vuFillR');
  
  if (!isPlaying) {
    // Grey out when not playing
    vuFillL.style.background = 'conic-gradient(from 180deg, #666 0deg, #666 180deg, #333 180deg)';
    vuFillR.style.background = 'conic-gradient(from 180deg, #666 0deg, #666 180deg, #333 180deg)';
  } else {
    // Normal color zones when playing
    if (finalVuL > 80) {
      vuFillL.style.background = 'conic-gradient(from 180deg, #f44336 0deg, #f44336 180deg, #333 180deg)'; // Red zone
    } else if (finalVuL > 60) {
      vuFillL.style.background = 'conic-gradient(from 180deg, #f44336 0deg, #FF9800 120deg, #FF9800 180deg, #333 180deg)'; // Orange zone
    } else {
      vuFillL.style.background = 'conic-gradient(from 180deg, #f44336 0deg, #FF9800 60deg, #4CAF50 120deg, #333 180deg)'; // Green zone
    }
    
    if (finalVuR > 80) {
      vuFillR.style.background = 'conic-gradient(from 180deg, #f44336 0deg, #f44336 180deg, #333 180deg)';
    } else if (finalVuR > 60) {
      vuFillR.style.background = 'conic-gradient(from 180deg, #f44336 0deg, #FF9800 120deg, #FF9800 180deg, #333 180deg)';
    } else {
      vuFillR.style.background = 'conic-gradient(from 180deg, #f44336 0deg, #FF9800 60deg, #4CAF50 120deg, #333 180deg)';
    }
  }
}

// Show/Hide track selector overlay
function showTrackSelector(number) {
  const selector = document.getElementById('trackSelector');
  const numberDisplay = document.getElementById('trackNumber');
  
  numberDisplay.innerText = number;
  selector.classList.add('show');
  trackSelectorVisible = true;
  
  // Auto-hide after 3 seconds
  if (numberTimeout) clearTimeout(numberTimeout);
  numberTimeout = setTimeout(() => {
    hideTrackSelector();
  }, 3000);
}

function hideTrackSelector() {
  const selector = document.getElementById('trackSelector');
  selector.classList.remove('show');
  trackSelectorVisible = false;
  typedNumber = '';
  if (numberTimeout) {
    clearTimeout(numberTimeout);
    numberTimeout = null;
  }
}

// Play track by number (1-based index)
function playTrackByNumber(trackNum) {
  console.log('playTrackByNumber called with:', trackNum);
  console.log('Current data state:', data);
  
  if (!data || !data.items) {
    console.log('ERROR: No track data available - data:', data);
    return false;
  }
  
  console.log('Total items in data.items:', data.items.length);
  
  // Filter only music files (not directories)
  const musicFiles = data.items.filter(item => !item.d);
  console.log('Music files found:', musicFiles.length, musicFiles.map(f => f.n));
  
  if (trackNum < 1 || trackNum > musicFiles.length) {
    console.log('Track number out of range:', trackNum, 'Max tracks:', musicFiles.length);
    return false;
  }
  
  // Find the actual index in full items array
  let musicCount = 0;
  let targetIndex = -1;
  
  for (let i = 0; i < data.items.length; i++) {
    if (!data.items[i].d) { // Music file
      musicCount++;
      if (musicCount === trackNum) {
        targetIndex = i;
        break;
      }
    }
  }
  
  if (targetIndex >= 0) {
    console.log('Playing track', trackNum, '(index', targetIndex, '):', data.items[targetIndex].n);
    fetch('/sdplayer/api/play?i=' + targetIndex, {method: 'POST'})
      .then(() => refresh())
      .catch(e => console.error('Play error:', e));
    return true;
  }
  
  return false;
}

// Handle keyboard input for track selection
function handleKeyPress(event) {
  console.log('Key pressed:', event.key, event.code, 'keyCode:', event.keyCode);
  
  // Only handle numeric keys (0-9)
  if (event.key >= '0' && event.key <= '9') {
    console.log('Numeric key detected:', event.key);
    event.preventDefault();
    
    const digit = event.key;
    typedNumber += digit;
    
    console.log('Key pressed:', digit, 'Typed number:', typedNumber, 'Current data:', data ? 'available' : 'null');
    
    const number = parseInt(typedNumber);
    
    // Limit to 3 digits (max 100 tracks realistically)
    if (typedNumber.length > 3) {
      typedNumber = typedNumber.substring(1); // Keep last 3 digits
    }
    
    // Show the current typed number
    showTrackSelector(typedNumber);
    
    // Auto-execute if we have valid single digit (1-9)
    if (typedNumber.length === 1 && number >= 1 && number <= 9) {
      setTimeout(() => {
        if (typedNumber === digit) { // Still the same single digit after delay
          console.log('Single digit auto-execute:', number);
          if (playTrackByNumber(number)) {
            hideTrackSelector();
          }
        }
      }, 1500); // 1.5 second delay for single digits
    }
    
    return;
  }
  
  // Enter key - execute typed number
  if (event.key === 'Enter' && typedNumber.length > 0) {
    event.preventDefault();
    const number = parseInt(typedNumber);
    console.log('Enter pressed, executing number:', number);
    
    if (playTrackByNumber(number)) {
      hideTrackSelector();
    }
    
    return;
  }
  
  // Escape key - cancel
  if (event.key === 'Escape') {
    event.preventDefault();
    hideTrackSelector();
    return;
  }
  
  // Backspace - remove last digit
  if (event.key === 'Backspace' && trackSelectorVisible) {
    event.preventDefault();
    if (typedNumber.length > 0) {
      typedNumber = typedNumber.slice(0, -1);
      if (typedNumber.length > 0) {
        showTrackSelector(typedNumber);
      } else {
        hideTrackSelector();
      }
    }
    return;
  }
}

function refresh() {
  console.log('Refresh called');
  fetch('/sdplayer/api/list')
    .then(r => {
      if (!r.ok) throw new Error('HTTP error! status: ' + r.status);
      return r.json();
    })
    .then(j => {
      console.log('Data received:', j);
      data = j;
      render();
    })
    .catch(e => {
      console.error('Refresh error:', e);
    });
}

function render() {
  if (!data) return;
  
  try {
    // Update OLED Style 8 header
    const statusIcon = document.getElementById('statusIcon');
    const artistTitle = document.getElementById('artistTitle');
    
    // Status icon
    if (data.status === 'Playing') {
      statusIcon.innerHTML = '▶️';
      statusIcon.className = 'status-icon playing';
    } else if (data.status === 'Paused') {
      statusIcon.innerHTML = '⏸️';
      statusIcon.className = 'status-icon paused';
    } else {
      statusIcon.innerHTML = '⏹️';
      statusIcon.className = 'status-icon stopped';
    }
    
    // Artist - Title
    if (data.now && data.now !== 'None') {
      const parsed = parseArtistTitle(data.now);
      artistTitle.innerText = parsed.artist + ' - ' + parsed.title;
    } else {
      artistTitle.innerText = 'ARTYSTA - Waiting for track...';
    }
    
    // Track info with numbering
    const trackInfo = document.getElementById('trackInfo');
    const codecInfo = document.getElementById('codecInfo');
    
    if (data.items && Array.isArray(data.items)) {
      const musicFiles = data.items.filter(item => !item.d);
      totalTracks = musicFiles.length;
      
      // Find current track index
      if (data.now) {
        currentTrackIndex = musicFiles.findIndex(item => item.n === data.now) + 1;
      }
      
      if (currentTrackIndex > 0) {
        const parsed = parseArtistTitle(data.now);
        trackInfo.innerText = `🎵 ${currentTrackIndex}. ${parsed.title}`;
      } else {
        trackInfo.innerText = `🎵 1. Waiting for track... (${totalTracks} total)`;
      }
    }
    
    // Update codec/technical info
    if (data.codec && data.bitRate && data.sampleRate) {
      const channels = data.channels === 1 ? 'Mono' : 'Stereo';
      codecInfo.innerText = `${data.codec}/${data.bitRate}kbps/${(data.sampleRate/1000).toFixed(1)}kHz/${channels}`;
    } else {
      codecInfo.innerText = 'MP3/320kbps/44.1kHz/Stereo';
    }
    
    // Update progress bar and times
    const currentTime = data.currentTime || 0;
    const totalTime = data.totalTime || 0;
    
    document.getElementById('currentTime').innerText = formatTime(currentTime);
    document.getElementById('totalTime').innerText = formatTime(totalTime);
    
    // Progress bar
    const progressPercent = totalTime > 0 ? (currentTime / totalTime) * 100 : 0;
    document.getElementById('progressFill').style.width = `${progressPercent}%`;
    
    // Volume & Speaker
    const vol = data.vol || 0;
    document.getElementById('volDisplay').innerText = vol;
    document.getElementById('volText').innerText = vol;
    document.getElementById('volSlider').value = vol;
    setVol(vol); // Update speaker icon
    
    // Update Analog VU Meters
    updateVUMeters(data.vuL || 0, data.vuR || 0);
    
    // Album/Directory info
    document.getElementById('albumInfo').innerText = '📁 ALBUM - ' + (data.cwd || '/');
    document.getElementById('currentPath').innerText = data.cwd || '/';
    
    // Update pause button text
    const pauseBtn = document.getElementById('pauseBtn');
    if (data.status === 'Playing') {
      pauseBtn.innerHTML = '⏸️ Pause';
    } else if (data.status === 'Paused') {
      pauseBtn.innerHTML = '▶️ Resume';
    } else {
      pauseBtn.innerHTML = '⏸️ Pause';
    }
    
    // Render file list with numbering
    const fileList = document.getElementById('fileList');
    fileList.innerHTML = '';
    
    if (data.items && Array.isArray(data.items)) {
      let trackNum = 1;
      
      data.items.forEach((item, idx) => {
        const fileItem = document.createElement('div');
        fileItem.className = 'file-item';
        
        // Check if this is the currently playing track
        if (item.n === data.now && data.status !== 'Stopped') {
          fileItem.classList.add('selected');
        }
        
        if (item.d) {
          // Directory
          fileItem.innerHTML = `
            <div class="file-icon">📁</div>
            <div class="file-name">/${item.n}</div>
          `;
        } else {
          // Music file with track number - IMPROVED with better numbering
          const trackDisplay = trackNum.toString().padStart(2, '0'); // 01, 02, 03...
          const isStream = item.s === true;
          const fileIcon = isStream ? '📡' : '🎵';
          
          // Pokaż przycisk DELETE tylko w folderze RECORDINGS (nie dla DLNA streamów)
          const isRecordingsFolder = (data.cwd || '').includes('/RECORDINGS');
          const deleteButton = (isRecordingsFolder && !isStream) ? 
            `<button class="delete-btn" onclick="event.stopPropagation(); deleteFile('${item.n}')" title="Delete recording">🗑️</button>` : '';
          
          fileItem.innerHTML = `
            <div class="track-number" title="Press ${trackNum} to play">${trackDisplay}.</div>
            <div class="file-icon">${fileIcon}</div>
            <div class="file-name">${item.n}</div>
            ${deleteButton}
          `;
          trackNum++;
        }
        
        fileItem.onclick = () => {
          if (item.d) {
            const newPath = data.cwd === '/' ? ('/' + item.n) : (data.cwd + '/' + item.n);
            fetch('/sdplayer/api/cd?p=' + encodeURIComponent(newPath))
              .then(() => refresh());
          } else {
            fetch('/sdplayer/api/play?i=' + idx, {method: 'POST'})
              .then(() => refresh());
          }
        };
        
        fileList.appendChild(fileItem);
      });
    }
    
  } catch (e) {
    console.error('Render error:', e);
  }
}

// Initialize
console.log('SD Player OLED Style 8 loaded');

// Add keyboard event listener for track selection
document.addEventListener('keydown', handleKeyPress);
console.log('Keyboard track selection enabled (1-100)');

// Funkcja usuwania pliku (tylko RECORDINGS)
function deleteFile(filename) {
  if (!confirm('Czy na pewno chcesz usunąć nagranie: ' + filename + '?')) {
    return;
  }
  
  console.log('Deleting file:', filename);
  fetch('/sdplayer/api/delete?f=' + encodeURIComponent(filename), {method: 'POST'})
    .then(r => r.json())
    .then(data => {
      if (data.success) {
        console.log('File deleted successfully');
        refresh(); // Odśwież listę
      } else {
        alert('Błąd usuwania: ' + (data.error || 'Unknown error'));
      }
    })
    .catch(e => {
      console.error('Delete error:', e);
      alert('Błąd usuwania pliku!');
    });
}

// NOWE FUNKCJE: Przełączanie stylów OLED
function setOledStyle(styleNumber) {
  console.log('Setting OLED style to:', styleNumber);
  fetch('/sdplayer/api/setStyle?style=' + styleNumber, {method: 'POST'})
    .then(r => r.json())
    .then(data => {
      console.log('Style set to:', data.style);
      updateStyleInfo(data.style);
    })
    .catch(e => console.error('Style set error:', e));
}

function getCurrentStyle() {
  fetch('/sdplayer/api/getStyle')
    .then(r => r.json())
    .then(data => {
      console.log('Current style:', data.style);
      updateStyleInfo(data.style);
    })
    .catch(e => console.error('Get style error:', e));
}

function updateStyleInfo(styleNumber) {
  const styleInfo = document.getElementById('currentStyleInfo');
  if (styleInfo) {
    styleInfo.textContent = 'Style: ' + styleNumber;
  }
}

// Pobierz aktualny styl przy ładowaniu
setTimeout(getCurrentStyle, 1000);

// ===== DLNA INTEGRACJA =====
let dlnaModeActive = false;

async function dlnaAction() {
  if (dlnaModeActive) {
    // Już w trybie DLNA - nic nie rób (użyj "Wróć do SD")
    showDlnaBar(true);
    return;
  }
  // Sprawdź status DLNA
  try {
    const r = await fetch('/sdplayer/api/dlna_status');
    const d = await r.json();
    if (!d.enabled) {
      alert('DLNA nie jest skompilowane (USE_DLNA wyłączone).');
      return;
    }
    if (!d.ready) {
      if (confirm('Brak playlisty DLNA.\nChcesz przejść do panelu DLNA aby ją zbudować?')) {
        window.location.href = '/dlna';
      }
      return;
    }
    if (!confirm('Załadować ' + d.tracks + ' utworów z serwera DLNA do SDPlayera?')) return;
    const r2 = await fetch('/sdplayer/api/dlna_load', {method: 'POST'});
    const d2 = await r2.json();
    if (d2.ok) {
      dlnaModeActive = true;
      document.getElementById('dlnaBtn').classList.add('active');
      document.getElementById('dlnaBtn').innerText = '📡 DLNA ✓';
      showDlnaBar(true, d2.tracks);
      refresh();
    } else {
      alert('Błąd ładowania DLNA: ' + (d2.err || 'nieznany'));
    }
  } catch(e) {
    alert('Błąd połączenia: ' + e);
  }
}

async function dlnaBack() {
  if (!confirm('Wrócić do listy plików SD i wyłączyć tryb DLNA?')) return;
  try {
    await fetch('/sdplayer/api/dlna_back', {method: 'POST'});
    dlnaModeActive = false;
    document.getElementById('dlnaBtn').classList.remove('active');
    document.getElementById('dlnaBtn').innerText = '📡 DLNA';
    showDlnaBar(false);
    refresh();
  } catch(e) {
    console.error('dlnaBack error:', e);
  }
}

function showDlnaBar(visible, tracks) {
  const bar = document.getElementById('dlnaBar');
  if (visible) {
    bar.classList.add('show');
    if (tracks !== undefined) {
      document.getElementById('dlnaTrackCount').innerText = tracks + ' utworów';
    }
  } else {
    bar.classList.remove('show');
  }
}

// Przy ładowaniu strony - sprawdź czy tryb DLNA był aktywny
fetch('/sdplayer/api/dlna_status')
  .then(r => r.json())
  .then(d => {
    if (d.enabled && d.dlna_active) {
      dlnaModeActive = true;
      document.getElementById('dlnaBtn').classList.add('active');
      document.getElementById('dlnaBtn').innerText = '📡 DLNA ✓';
      showDlnaBar(true, d.tracks);
    }
  })
  .catch(() => {});
// ===== KONIEC DLNA =====

refresh();
refreshTimer = setInterval(refresh, 3000); // Refresh every 3 seconds
</script>
</body>
</html>
)HTML";

class SDPlayerWebUI {
public:
    SDPlayerWebUI();
    void begin(AsyncWebServer* server, Audio* audioPtr = nullptr);
    void setExitCallback(std::function<void()> callback);
    void setOLED(SDPlayerOLED* oled);  // Ustaw OLED display
    
    // Synchronizacja callbacks
    typedef std::function<void(int)> IndexChangeCallback;
    typedef std::function<void(const String&)> FileChangeCallback;
    typedef std::function<void(bool)> PlayStateCallback;
    
    void setIndexChangeCallback(IndexChangeCallback callback) { _indexChangeCallback = callback; }
    void setFileChangeCallback(FileChangeCallback callback) { _fileChangeCallback = callback; }
    void setPlayStateCallback(PlayStateCallback callback) { _playStateCallback = callback; }
    
    // Kontrola odtwarzacza
    void playFile(const String& path);
    void playIndex(int index);
    void pause();
    void stop();
    void next();
    void prev();
    void seekRelative(int deltaSeconds); // Przewijanie względne (+/- sekundy)
    void seekAbsolute(int targetSeconds); // Przewijanie bezwzględne (do konkretnej sekundy)
    void playNextAuto(); // Automatyczne odtwarzanie następnego utworu (zapętlenie)
    void setVolume(int vol);
    
    // Zarządzanie katalogiem
    void changeDirectory(const String& path);
    void upDirectory();
    String getCurrentDirectory() { return _currentDir; }
    void scanDirectory();  // Publiczne skanowanie katalogu (wywołane z loop())
    
    // Status
    bool isPlaying() { return _isPlaying; }
    bool isPaused() { return _isPaused; }
    String getCurrentFile() { return _currentFile; }
    void setCurrentFile(const String& path) { _currentFile = path; }  // Ustaw nazwę aktualnie odtwarzanego pliku
    void setIsPlaying(bool playing) { _isPlaying = playing; _isPaused = false; }  // Ustaw stan odtwarzania
    int getVolume() { return _volume; }
    int getSelectedIndex() const { return _selectedIndex; }  // Zwraca aktualny indeks zaznaczonego pliku
    
    // Funkcje synchronizacji z OLED
    void syncOLEDDisplay();  // Odświeża OLED po zmianach
    void syncFileListWithOLED();  // Synchronizuje listę plików z OLED aby uniknąć lazy loading
    void notifyIndexChange(int newIndex);  // Powiadamia o zmianie indeksu
    void notifyFileChange(const String& newFile);  // Powiadamia o zmianie pliku
    void notifyPlayStateChange(bool playing);  // Powiadamia o zmianie stanu odtwarzania

private:
    AsyncWebServer* _server;
    Audio* _audio;
    SDPlayerOLED* _oled;
    
    // Callbacks dla synchronizacji
    IndexChangeCallback _indexChangeCallback;
    FileChangeCallback _fileChangeCallback;
    PlayStateCallback _playStateCallback;
    std::function<void()> _exitCallback;
    
    String _currentDir;
    String _currentFile;
    String _pausedFilePath;
    uint32_t _pausedPositionSec;
    uint32_t _pausedFilePositionBytes;
    int _volume;
    bool _isPlaying;
    bool _isPaused;
    int _selectedIndex;
    
    // Throttling dla skanowania katalogu
    unsigned long _lastScanTime = 0;
    const unsigned long _minScanInterval = 2000; // Minimalne 2 sekundy między skanowaniami
    
    struct FileItem {
        String name;
        bool isDir;
        bool isStream = false;   // DLNA strumień HTTP
        String streamUrl;        // URL strumienia (tylko gdy isStream=true)
    };
    std::vector<FileItem> _fileList;
    bool _dlnaMode = false;  // True = lista z DLNA (nie z SD)
    
    void scanCurrentDirectory();
    void buildFileList(JsonArray& items);
    bool isAudioFile(const String& filename);
    void sortFileList();
    
    // Handler functions
    void handleRoot(AsyncWebServerRequest *request);
    void handleList(AsyncWebServerRequest *request);
    void handlePlay(AsyncWebServerRequest *request);
    void handlePlaySelected(AsyncWebServerRequest *request);
    void handlePause(AsyncWebServerRequest *request);
    void handleStop(AsyncWebServerRequest *request);
    void handleNext(AsyncWebServerRequest *request);
    void handlePrev(AsyncWebServerRequest *request);
    void handleVol(AsyncWebServerRequest *request);
    void handleSeek(AsyncWebServerRequest *request);
    void handleCd(AsyncWebServerRequest *request);
    void handleUp(AsyncWebServerRequest *request);
    void handleBack(AsyncWebServerRequest *request);
    
    // NOWE: Handlery przełączania stylów OLED
    void handleNextStyle(AsyncWebServerRequest *request);
    void handleSetStyle(AsyncWebServerRequest *request);
    void handleGetStyle(AsyncWebServerRequest *request);
    
    // Usuwanie plików (RECORDINGS)
    void handleDelete(AsyncWebServerRequest *request);

    // DLNA integracja
    void handleDlnaStatus(AsyncWebServerRequest *request);
    void handleDlnaLoad(AsyncWebServerRequest *request);
    void handleDlnaBack(AsyncWebServerRequest *request);
};
