// The page the board serves. Self-contained on purpose: the bench is a closed network with no
// route to a CDN, so every byte of chart code is here rather than linked.
//
// What it has to communicate, in order of how badly it is needed: whether the link is up and
// at what speed, whether packets are arriving at all, and only then what their timing looks
// like. A board that shows a beautiful jitter histogram of zero packets has answered nothing.
#pragma once
#include <Arduino.h>

const char kPage[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>esp32-lidar probe</title>
<style>
:root{
  color-scheme:light dark;
  --plane:#f9f9f7; --surface:#fcfcfb; --ink:#0b0b0b; --ink2:#52514e; --muted:#898781;
  --grid:#e1e0d9; --axis:#c3c2b7; --border:rgba(11,11,11,.10);
  --series:#2a78d6; --series-soft:rgba(42,120,214,.14);
  --good:#0ca30c; --warning:#fab219; --critical:#d03b3b;
}
@media (prefers-color-scheme:dark){:root{
  --plane:#0d0d0d; --surface:#1a1a19; --ink:#fff; --ink2:#c3c2b7; --muted:#898781;
  --grid:#2c2c2a; --axis:#383835; --border:rgba(255,255,255,.10);
  --series:#3987e5; --series-soft:rgba(57,135,229,.18);
}}
*{box-sizing:border-box}
body{margin:0;background:var(--plane);color:var(--ink);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
.wrap{max-width:1100px;margin:0 auto;padding:20px 16px 48px}
h1{font-size:17px;font-weight:600;margin:0 0 2px}
.sub{color:var(--ink2);font-size:13px;margin:0 0 18px}
.card{background:var(--surface);border:1px solid var(--border);border-radius:10px;padding:14px 16px;margin-bottom:14px}
.banner{display:flex;align-items:center;gap:10px;font-weight:600}
.dot{width:10px;height:10px;border-radius:50%;flex:none}
.banner .note{font-weight:400;color:var(--ink2);font-size:13px}
.verdict{margin-top:10px;padding-top:10px;border-top:1px solid var(--border);
  color:var(--ink2);font-size:13px;display:none}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(126px,1fr));gap:2px;
  background:var(--border);border:1px solid var(--border);border-radius:10px;overflow:hidden;margin-bottom:14px}
.tile{background:var(--surface);padding:12px 14px}
.tile .k{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.04em}
.tile .v{font-size:22px;margin-top:3px;font-variant-numeric:tabular-nums}
.tile .u{font-size:12px;color:var(--ink2);margin-left:3px}
.chart h2{font-size:13px;font-weight:600;margin:0 0 1px}
.chart .cap{color:var(--muted);font-size:12px;margin:0 0 10px}
canvas{width:100%;height:180px;display:block;touch-action:none}
.tip{position:fixed;pointer-events:none;opacity:0;transition:opacity .08s;
  background:var(--surface);border:1px solid var(--border);border-radius:7px;
  padding:6px 9px;font-size:12px;font-variant-numeric:tabular-nums;
  box-shadow:0 4px 14px rgba(0,0,0,.18);z-index:9}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;color:var(--ink2);font-size:13px}
input,button{font:inherit;background:var(--plane);color:var(--ink);
  border:1px solid var(--border);border-radius:7px;padding:5px 10px}
button{cursor:pointer}
table{border-collapse:collapse;width:100%;font-size:12px;font-variant-numeric:tabular-nums}
th,td{text-align:right;padding:4px 8px;border-bottom:1px solid var(--grid)}
th:first-child,td:first-child{text-align:left;color:var(--ink2)}
th{color:var(--muted);font-weight:500}
details summary{cursor:pointer;color:var(--ink2);font-size:13px}
code{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px;color:var(--ink2);word-break:break-all}
</style></head><body><div class="wrap">

<h1>esp32-lidar probe</h1>
<p class="sub">ESP32-S3 + W5500 &middot; <span id="ident">&mdash;</span></p>

<div class="card">
  <div class="banner"><span class="dot" id="linkdot"></span><span id="linktext">reading PHY&hellip;</span>
    <span class="note" id="linknote"></span></div>
  <div class="verdict" id="verdict"></div>
</div>

<div class="tiles">
  <div class="tile"><div class="k">packets</div><div class="v" id="t-pkt">0</div></div>
  <div class="tile"><div class="k">rate</div><div class="v" id="t-rate">0<span class="u">/s</span></div></div>
  <div class="tile"><div class="k">mean gap</div><div class="v" id="t-mean">&mdash;<span class="u">&micro;s</span></div></div>
  <div class="tile"><div class="k">jitter (SD)</div><div class="v" id="t-sd">&mdash;<span class="u">&micro;s</span></div></div>
  <div class="tile"><div class="k">p99 gap</div><div class="v" id="t-p99">&mdash;<span class="u">&micro;s</span></div></div>
  <div class="tile"><div class="k">payload</div><div class="v" id="t-size">&mdash;<span class="u">B</span></div></div>
</div>

<div class="card chart">
  <h2>Packets per second</h2>
  <p class="cap">Last 120 seconds on UDP 7502. A flat line at the sensor's nominal rate is the goal; dips are loss.</p>
  <canvas id="c-rate"></canvas>
</div>

<div class="card chart">
  <h2>Gap between packets</h2>
  <p class="cap">Last 1024 arrivals, newest at the right, microseconds. The dashed hairline is the median.</p>
  <canvas id="c-gap"></canvas>
</div>

<div class="card chart">
  <h2>Distribution of gaps</h2>
  <p class="cap">How tightly the sensor is actually spaced &mdash; a second lump means something is bunching packets.<span id="clip"></span></p>
  <canvas id="c-hist"></canvas>
</div>

<div class="card">
  <div class="row">
    <span>listen on another port:</span>
    <input id="port" type="number" min="0" max="65535" placeholder="e.g. 2368" style="width:110px">
    <button id="go">listen</button>
    <button id="clear">reset counters</button>
    <span id="scanstate"></span>
  </div>
</div>

<div class="card">
  <details><summary>Table view &amp; first bytes</summary>
    <table id="tbl"><thead><tr><th>stream</th><th>port</th><th>packets</th><th>bytes</th>
      <th>size</th><th>min &micro;s</th><th>p50</th><th>p99</th><th>max</th><th>source</th></tr></thead>
      <tbody></tbody></table>
    <p style="margin:10px 0 0"><code id="head"></code></p>
  </details>
</div>

<div class="tip" id="tip"></div>
</div>
<script>
const $=s=>document.querySelector(s);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt=n=>n===undefined||n===null?'—':n.toLocaleString();

// Canvas is sized in CSS pixels but drawn at device resolution; without this the 1px grid and
// 2px line both land on half-pixels and the whole chart reads as blurry.
function fit(cv){const r=cv.getBoundingClientRect(),d=devicePixelRatio||1;
  cv.width=r.width*d;cv.height=r.height*d;const x=cv.getContext('2d');x.setTransform(d,0,0,d,0,0);
  return {x,w:r.width,h:r.height};}

const PAD={l:52,r:10,t:10,b:20};
function axes(x,w,h,ymax,ylabel){
  const g=css('--grid'),m=css('--muted');
  x.clearRect(0,0,w,h);
  x.font='11px system-ui,sans-serif';x.textBaseline='middle';
  for(let i=0;i<=4;i++){
    const v=ymax*i/4, y=h-PAD.b-(h-PAD.t-PAD.b)*i/4;
    x.strokeStyle=g;x.lineWidth=1;x.beginPath();
    x.moveTo(PAD.l,Math.round(y)+.5);x.lineTo(w-PAD.r,Math.round(y)+.5);x.stroke();
    x.fillStyle=m;x.textAlign='right';
    x.fillText(v>=1000?(v/1000).toFixed(v>=10000?0:1)+'k':Math.round(v),PAD.l-8,y);
  }
  x.strokeStyle=css('--axis');x.beginPath();
  x.moveTo(PAD.l,h-PAD.b+.5);x.lineTo(w-PAD.r,h-PAD.b+.5);x.stroke();
  if(ylabel){x.fillStyle=m;x.textAlign='left';x.fillText(ylabel,PAD.l,PAD.t-2);}
}

function line(cv,data,ymax,label,fill){
  const {x,w,h}=fit(cv); if(!data.length){axes(x,w,h,1,label);return;}
  ymax=ymax||Math.max(1,...data)*1.15;
  axes(x,w,h,ymax,label);
  const iw=w-PAD.l-PAD.r, ih=h-PAD.t-PAD.b;
  const px=i=>PAD.l+(data.length<2?iw:iw*i/(data.length-1));
  const py=v=>h-PAD.b-ih*Math.min(v,ymax)/ymax;
  if(fill){
    x.beginPath();x.moveTo(px(0),h-PAD.b);
    data.forEach((v,i)=>x.lineTo(px(i),py(v)));
    x.lineTo(px(data.length-1),h-PAD.b);x.closePath();
    x.fillStyle=css('--series-soft');x.fill();
  }
  x.beginPath();data.forEach((v,i)=>i?x.lineTo(px(i),py(v)):x.moveTo(px(i),py(v)));
  x.strokeStyle=css('--series');x.lineWidth=2;x.lineJoin='round';x.stroke();
  cv._geom={px,py,ymax,data};
}

function hairline(cv,v,text){
  const cx=cv.getContext('2d'),g=cv._geom;if(!g)return;
  const r=cv.getBoundingClientRect(),y=g.py(v);
  cx.save();cx.setLineDash([4,4]);cx.strokeStyle=css('--axis');cx.lineWidth=1;
  cx.beginPath();cx.moveTo(PAD.l,y);cx.lineTo(r.width-PAD.r,y);cx.stroke();cx.restore();
  cx.fillStyle=css('--muted');cx.font='11px system-ui,sans-serif';cx.textAlign='right';
  cx.fillText(text,r.width-PAD.r,y-8);
}

function bars(cv,counts,edges){
  const {x,w,h}=fit(cv); const ymax=Math.max(1,...counts);
  axes(x,w,h,ymax,'packets');
  const iw=w-PAD.l-PAD.r, ih=h-PAD.t-PAD.b, bw=iw/counts.length;
  x.fillStyle=css('--series');
  counts.forEach((c,i)=>{
    if(!c)return;
    const bh=ih*c/ymax, bx=PAD.l+i*bw, by=h-PAD.b-bh;
    // 2px of surface between neighbours, and rounded only at the data end.
    const r=Math.min(4,bh,(bw-2)/2);
    x.beginPath();x.moveTo(bx+1,h-PAD.b);x.lineTo(bx+1,by+r);
    x.quadraticCurveTo(bx+1,by,bx+1+r,by);x.lineTo(bx+bw-1-r,by);
    x.quadraticCurveTo(bx+bw-1,by,bx+bw-1,by+r);x.lineTo(bx+bw-1,h-PAD.b);x.fill();
  });
  x.fillStyle=css('--muted');x.font='11px system-ui,sans-serif';x.textAlign='center';
  if(edges.length){x.fillText(Math.round(edges[0])+' µs',PAD.l+20,h-6);
    x.fillText(Math.round(edges[1])+' µs',w-PAD.r-24,h-6);}
  cv._hist={counts,edges,bw};
}

// Hover: a crosshair and a value, on the two charts where a single point means something.
function hover(cv,label,unit){
  const tip=$('#tip');
  cv.addEventListener('pointermove',e=>{
    const g=cv._geom;if(!g||!g.data.length){tip.style.opacity=0;return;}
    const r=cv.getBoundingClientRect(),iw=r.width-PAD.l-PAD.r;
    let i=Math.round((e.clientX-r.left-PAD.l)/iw*(g.data.length-1));
    i=Math.max(0,Math.min(g.data.length-1,i));
    tip.textContent=label(i,g.data[i])+' '+unit;
    tip.style.left=(e.clientX+12)+'px';tip.style.top=(e.clientY-30)+'px';tip.style.opacity=1;
  });
  cv.addEventListener('pointerleave',()=>$('#tip').style.opacity=0);
}
hover($('#c-rate'),(i,v)=>(120-i)+'s ago: '+fmt(v)+' packets','');
hover($('#c-gap'),(i,v)=>fmt(v),'µs');

let quietFor=0;
async function tick(){
  let s;
  try{s=await (await fetch('/api/stats')).json();}catch(e){return;}
  const L=s.link, l=s.lidar;

  // Link first. "Up at 100M with nothing arriving" is a specific diagnosis, not a generic
  // failure, so it gets said out loud rather than left for someone to infer from two numbers.
  const up=L.up;
  $('#linkdot').style.background=up?(l.packets?css('--good'):css('--warning')):css('--critical');
  $('#linktext').textContent=up?('Link up — '+L.speed+'M '+L.duplex+' duplex')
                              :'Link down — no carrier';
  $('#linknote').textContent='W5500 VERSIONR 0x'+L.versionr.toString(16).toUpperCase().padStart(2,'0')
    +' · up '+s.uptime+'s';
  $('#ident').textContent=s.ip+' · '+s.mac;

  quietFor = l.packets ? 0 : quietFor+1;
  const v=$('#verdict');
  if(up && L.speed===100 && quietFor>10){
    v.style.display='block';
    v.innerHTML='Link negotiated at <b>100M</b> and nothing has arrived on 7502 for '+quietFor+
      's. W5500 is 10/100 only, and an Ouster OS1 refuses to transmit unless it gets '+
      '1000BASE-T full duplex — which is exactly this symptom. Put the switch in the path '+
      'so the sensor gets its gigabit link, or check the port with the scanner below.';
  } else if(!up && quietFor>3){
    v.style.display='block';
    v.textContent='No carrier. Check the cable and that the sensor is powered.';
  } else v.style.display='none';

  const rate=s.history[s.history.length-1] ? s.history[s.history.length-1][0] : 0;
  $('#t-pkt').textContent=fmt(l.packets);
  $('#t-rate').innerHTML=fmt(rate)+'<span class="u">/s</span>';
  $('#t-mean').innerHTML=fmt(l.mean)+'<span class="u">µs</span>';
  $('#t-sd').innerHTML=fmt(l.stddev)+'<span class="u">µs</span>';
  $('#t-p99').innerHTML=fmt(l.p99)+'<span class="u">µs</span>';
  $('#t-size').innerHTML=fmt(l.lastSize)+'<span class="u">B</span>';

  line($('#c-rate'),s.history.map(h=>h[0]),0,'packets/s',true);
  const gaps=l.intervals.slice().reverse();   // oldest left, newest right
  line($('#c-gap'),gaps,0,'µs',false);
  if(l.p50) hairline($('#c-gap'),l.p50,'p50 '+fmt(l.p50)+' µs');

  // Histogram over the observed range, clipped at p99 so one stall does not compress every
  // real bar into the leftmost bin.
  if(gaps.length){
    const lo=l.min, hi=Math.max(l.p99,lo+1), bins=48, counts=new Array(bins).fill(0);
    // Anything past p99 is dropped rather than piled into the last bin: a clamp there draws a
    // bar that looks exactly like the second lump this chart exists to reveal.
    let clipped=0;
    gaps.forEach(g=>{const b=Math.floor((g-lo)/(hi-lo)*bins);
      if(b>=0&&b<bins)counts[b]++; else clipped++;});
    bars($('#c-hist'),counts,[lo,hi]);
    $('#clip').textContent=clipped?(' '+clipped+' of '+gaps.length+' gaps fall past p99 and are not drawn.'):'';
  }

  const tb=$('#tbl').querySelector('tbody');tb.innerHTML='';
  [['lidar',7502,s.lidar],['imu',7503,s.imu],['scan',s.scanPort,s.scan]].forEach(([n,p,d])=>{
    if(n==='scan'&&!s.scanPort)return;
    tb.insertAdjacentHTML('beforeend','<tr><td>'+n+'</td><td>'+p+'</td><td>'+fmt(d.packets)+
      '</td><td>'+fmt(d.bytes)+'</td><td>'+fmt(d.lastSize)+'</td><td>'+fmt(d.min)+'</td><td>'+
      fmt(d.p50)+'</td><td>'+fmt(d.p99)+'</td><td>'+fmt(d.max)+'</td><td>'+d.source+'</td></tr>');
  });
  $('#head').textContent=l.head?('first 32 bytes: '+l.head.replace(/(..)/g,'$1 ')):'no packet seen yet';
  $('#scanstate').textContent=s.scanPort?('listening on '+s.scanPort+' — '+s.scan.packets+' packets'):'';
}

$('#go').onclick=()=>fetch('/api/listen?port='+($('#port').value||0)).then(tick);
$('#clear').onclick=()=>fetch('/api/reset').then(tick);
addEventListener('resize',()=>tick());
tick();setInterval(tick,1000);
</script></body></html>)rawliteral";
