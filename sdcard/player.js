let curDir = "/";
let lastStatus = "";

async function api(url){
  const r = await fetch(url);
  if(!r.ok) throw new Error(await r.text());
  return r.json();
}

function esc(s){
  return s.replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;");
}

async function loadDir(dir){
  curDir = dir;
  document.getElementById("path").textContent = curDir;

  const data = await api(`/player/api/list?dir=${encodeURIComponent(curDir)}`);
  const ul = document.getElementById("list");
  ul.innerHTML = "";

  data.items.forEach(it=>{
    const li = document.createElement("li");
    li.className = it.type;
    li.innerHTML = (it.type==="dir" ? "📁 " : "🎵 ") + esc(it.name);

    li.onclick = async ()=>{
      if(it.type==="dir"){
        await loadDir(it.path);
      }else{
        await api(`/player/api/play?file=${encodeURIComponent(it.path)}`);
      }
    };
    ul.appendChild(li);
  });
}

async function pollStatus(){
  try{
    const st = await api("/player/api/status");
    const txt = st.playing ? `PLAY: ${st.name}` : "STOP";
    if(txt !== lastStatus){
      lastStatus = txt;
      document.getElementById("status").textContent = "Status: " + txt;
    }
  }catch(e){
    document.getElementById("status").textContent = "Status: błąd API";
  }
}

document.getElementById("btnStop").onclick = ()=>api("/player/api/stop");
document.getElementById("btnRefresh").onclick = ()=>loadDir(curDir);
document.getElementById("btnRoot").onclick = ()=>loadDir("/");
document.getElementById("btnUp").onclick = ()=>api(`/player/api/up?dir=${encodeURIComponent(curDir)}`).then(x=>loadDir(x.dir));

loadDir("/");
setInterval(pollStatus, 1000);
pollStatus();
