let curDir = "/";
let lastStatus = "";
let selectedIndex = -1; // PUNKT 13: Zaznaczony element (dla podwójnego kliknięcia)

async function api(url, method = 'GET'){
  const options = {
    method: method
  };
  
  const r = await fetch(url, options);
  if(!r.ok) throw new Error(await r.text());
  
  // Sprawdź czy response ma content - niektóre POST endpointy zwracają tylko "OK"
  const contentType = r.headers.get('content-type');
  if (contentType && contentType.includes('application/json')) {
    return r.json();
  } else {
    return r.text(); // Zwróć tekst dla prostych odpowiedzi
  }
}

function esc(s){
  return s.replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;");
}

function upDir(dir) {
  if (dir === "/") return "/";
  const lastSlash = dir.lastIndexOf('/');
  if (lastSlash === 0) return "/";
  return dir.substring(0, lastSlash);
}

function formatTime(seconds) {
  if (!seconds || seconds <= 0) return "00:00";
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  return `${String(mins).padStart(2,'0')}:${String(secs).padStart(2,'0')}`;
}

function updateInfoPanel(data) {
  // Jeśli odtwarzamy - pokaż informacje jak w Stylu 8
  if (data.status === "Playing" || data.status === "Paused") {
    // Tytuł utworu (z folderu lub z now)
    const fileName = data.now || "-";
    document.getElementById("trackTitle").textContent = fileName;
    
    // Album (ostatni folder ze ścieżki)
    const album = data.album || "-";
    document.getElementById("trackAlbum").textContent = `ALBUM - ${album}`;
    
    // Informacje techniczne
    const codec = data.codec || "MP3";
    const bitrate = data.bitRate || 0;
    const sampleRate = data.sampleRate || 0;
    const bps = data.bitsPerSample || 0;
    
    document.getElementById("codec").textContent = codec;
    document.getElementById("bitrate").textContent = `${bitrate}kbps`;
    document.getElementById("samplerate").textContent = `${(sampleRate/1000).toFixed(1)}kHz`;
    document.getElementById("bps").textContent = `${bps}bit`;
    
    // Czas odtwarzania
    const currentTime = formatTime(data.currentTime);
    const totalTime = formatTime(data.totalTime);
    document.getElementById("currentTime").textContent = currentTime;
    document.getElementById("totalTime").textContent = totalTime;
  } else {
    // Stopped - wyczyść informacje
    document.getElementById("trackTitle").textContent = "-";
    document.getElementById("trackAlbum").textContent = "Album: -";
    document.getElementById("codec").textContent = "-";
    document.getElementById("bitrate").textContent = "0kbps";
    document.getElementById("samplerate").textContent = "0kHz";
    document.getElementById("bps").textContent = "0bit";
    document.getElementById("currentTime").textContent = "00:00";
    document.getElementById("totalTime").textContent = "00:00";
  }
}

async function loadDir(dir){
  curDir = dir;
  document.getElementById("path").textContent = curDir;

  const data = await api(`/sdplayer/api/list?dir=${encodeURIComponent(curDir)}`, "GET");
  const ul = document.getElementById("list");
  ul.innerHTML = "";

  // Update status from list response
  if (data.status && data.now) {
    const statusText = data.status === "Playing" ? `PLAY: ${data.now}` : 
                      data.status === "Paused" ? `PAUSE: ${data.now}` : "STOP";
    document.getElementById("status").textContent = "Status: " + statusText;
    lastStatus = statusText;
  }
  
  // Aktualizuj panel informacyjny
  updateInfoPanel(data);

  // Renderuj listę plików i folderów
  data.items.forEach((it, index)=>{
    const li = document.createElement("li");
    
    // it.d = isDir, it.n = name, it.num = track number (tylko dla plików)
    const isDir = it.d;
    const name = it.n;
    const trackNum = it.num || null;
    
    li.className = isDir ? "dir" : "file";
    
    // PUNKT 13: Zaznacz element jeśli jest selectedIndex
    if (index === selectedIndex && !isDir) {
      li.classList.add("selected");
    }
    
    // Dodaj numerację dla plików muzycznych
    const displayText = isDir ? `📁 ${name}` : `🎵 ${trackNum}. ${name}`;
    li.innerHTML = esc(displayText);

    li.onclick = async ()=>{
      if(isDir){
        // Zmień katalog (folder = pojedyncze kliknięcie)
        selectedIndex = -1; // Reset zaznaczenia
        const newDir = curDir === "/" ? `/${name}` : `${curDir}/${name}`;
        await loadDir(newDir);
      }else{
        // PUNKT 13: Podwójne kliknięcie dla plików
        if (selectedIndex === index) {
          // DRUGIE KLIKNIĘCIE - Odtwórz plik
          await api(`/sdplayer/api/play?i=${index}`, "POST");
          setTimeout(()=>pollStatus(), 500);
        } else {
          // PIERWSZE KLIKNIĘCIE - Zaznacz plik
          selectedIndex = index;
          // Odśwież listę aby pokazać zaznaczenie
          await loadDir(curDir);
        }
      }
    };
    ul.appendChild(li);
  });
}

async function pollStatus(){
  try{
    // Get status from list API instead of dedicated status endpoint (GET method)
    const data = await api(`/sdplayer/api/list?dir=${encodeURIComponent(curDir)}`, "GET");
    if (data.status && data.now) {
      const statusText = data.status === "Playing" ? `PLAY: ${data.now}` : 
                        data.status === "Paused" ? `PAUSE: ${data.now}` : "STOP";
      if (statusText !== lastStatus) {
        lastStatus = statusText;
        document.getElementById("status").textContent = "Status: " + statusText;
      }
    }
    
    // Aktualizuj panel informacyjny
    updateInfoPanel(data);
  }catch(e){
    document.getElementById("status").textContent = "Status: błąd API";
  }
}

document.getElementById("btnPrev").onclick = ()=>api("/sdplayer/api/prev", "POST").then(()=>setTimeout(pollStatus, 300));
document.getElementById("btnNext").onclick = ()=>api("/sdplayer/api/next", "POST").then(()=>setTimeout(pollStatus, 300));
document.getElementById("btnPause").onclick = ()=>api("/sdplayer/api/pause", "POST").then(()=>setTimeout(pollStatus, 300));
document.getElementById("btnStop").onclick = ()=>api("/sdplayer/api/stop", "POST").then(()=>setTimeout(pollStatus, 300));
document.getElementById("btnRefresh").onclick = ()=>loadDir(curDir);
document.getElementById("btnRoot").onclick = ()=>loadDir("/");
document.getElementById("btnUp").onclick = ()=>api(`/sdplayer/api/up?dir=${encodeURIComponent(curDir)}`, "POST").then(()=>loadDir(upDir(curDir)));

loadDir("/");
setInterval(pollStatus, 1000);
pollStatus();
