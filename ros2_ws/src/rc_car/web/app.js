'use strict';
const $ = id => document.getElementById(id);
const fmt = (x, d=2) => typeof x === 'number' && Number.isFinite(x) ? x.toFixed(d) : '—';
let ws, sequence=0, serverTime=null, state=null, session=null, lastState=0, grid=null, gridCanvas=null;
const keys=new Set();
function log(message){$('events').textContent=(new Date().toLocaleTimeString()+' '+message+'\n'+$('events').textContent).slice(0,4000);}
function send(type, extra={}){if(ws?.readyState===WebSocket.OPEN)ws.send(JSON.stringify({v:1,seq:++sequence,type,server_time:serverTime,...extra}));}
function stop(){keys.clear();send('stop');}
function limits(){return {pwm:+$('pwm').value,rpm:[+$('rpm1').value,+$('rpm2').value],steering:+$('steer').value};}
function labels(){const l=limits();$('driveValue').textContent=$('drive').value+'%';$('pwmValue').textContent=fmt(l.pwm*100,0)+'%';$('rpm1Value').textContent=fmt(l.rpm[0],1);$('rpm2Value').textContent=fmt(l.rpm[1],1);$('steerValue').textContent=fmt(l.steering)+' rad';}
function connection(text, good=false){$('connection').textContent=text;$('connection').className='badge '+(good?'ok':'warn');}
function connect(){
  ws=new WebSocket(`ws://${location.host}/ws`);
  ws.onopen=()=>{connection('접속 · 상태 대기');keys.clear();};
  ws.onclose=()=>{connection('연결 끊김 · 출력 정지');keys.clear();state=null;serverTime=null;session=null;lastState=0;render();setTimeout(connect,2000);};
  ws.onerror=()=>connection('연결 오류');
  ws.onmessage=event=>{
    const msg=JSON.parse(event.data);
    if(msg.type==='hello'){
      session=msg.session;
      for(const [id,value] of [['pwm',msg.limits.pwm],['rpm1',msg.limits.rpm[0]],['rpm2',msg.limits.rpm[1]],['steer',msg.limits.steering]]){$(id).max=value;$(id).value=value;}
      labels();log('접속 완료. 제어권 획득 후 활성화하십시오.');
    } else if(msg.type==='state'){
      state=msg;serverTime=msg.server_time;lastState=performance.now();connection('WebSocket 연결',true);render();
    } else if(msg.type==='ack'){
      log(msg.ok?`명령 ${msg.seq} 완료${msg.saved?' · '+msg.saved.join(', '):''}`:`거부: ${msg.error}`);
    } else if(msg.type==='map'){
      grid=msg;gridCanvas=document.createElement('canvas');gridCanvas.width=msg.width;gridCanvas.height=msg.height;
      const context=gridCanvas.getContext('2d'), pixels=context.createImageData(msg.width,msg.height);
      for(let y=0;y<msg.height;y++)for(let x=0;x<msg.width;x++){
        const value=msg.data[y*msg.width+x], index=((msg.height-1-y)*msg.width+x)*4;
        const grey=value<0?64:Math.round(235-value*2.1);pixels.data.set([grey,grey+(value<0?8:0),grey+(value<0?12:0),255],index);
      }
      context.putImageData(pixels,0,0);$('mapEmpty').hidden=true;drawMap();
    }
  };
}
function render(){
  const data=state?.data||{}, command=data.command||{}, vehicle=data.vehicle||{};
  const fresh=state&&performance.now()-lastState<600&&(state.age_s.vehicle??999)<0.6;
  const commandFresh=state&&(state.age_s.command??999)<0.5;
  $('mode').textContent=command.mode||'MANUAL';
  if(command.control&&document.activeElement!==$('control'))$('control').value=command.control;
  $('owner').textContent=command.owner===session&&session?'이 브라우저에 제어권 있음':command.owner?'다른 클라이언트가 제어 중':'관찰 모드 · 제어권 없음';
  $('reason').textContent=`${commandFresh?(command.armed?'활성화':'출력 차단'):'제어 상태 오래됨'} · ${command.reason||'상태 대기'}${command.estop?' · 비상정지 래치':''}`;
  $('arm').disabled=!fresh||!commandFresh||command.owner!==session||!vehicle.ready;
  $('fresh').textContent=fresh?'실시간 수신':'데이터 없음 / 오래됨';$('fresh').className='badge '+(fresh?'ok':'warn');
  $('slam').textContent=fresh&&state?.slam_ready?'SLAM 데이터 준비됨':'SLAM 준비 안 됨';$('slam').className='badge '+(fresh&&state?.slam_ready?'ok':'warn');
  $('fault').textContent=vehicle.fault||vehicle.sensor_error||state?.map_error||'';
  for(let i=0;i<2;i++){
    const m=fresh?vehicle.motors?.[i]||{}:{};
    const rows=[['목표 / 측정 RPM',`${fmt(m.target_rpm,1)} / ${fmt(m.rpm,1)}`],['PWM 출력',fmt(m.pwm*100,1)+'%'],['원시 카운트',m.count??'—'],['모터축 누적 회전',fmt(m.revolutions)],['이동거리 (부호)',fmt(m.distance_m)+' m'],['누적 경로',fmt(m.path_m)+' m'],['방향',m.direction===1?'전진':m.direction===-1?'후진':m.direction===0?'정지':'미확인']];
    $('motor'+i).replaceChildren(...rows.flatMap(([label,value])=>{const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=label;dd.textContent=value;return [dt,dd];}));
  }
  const trailer=fresh?vehicle.trailer:null, servo=fresh?vehicle.servo:null;
  $('trailer').textContent=fmt(trailer?.angle_rad)+' rad';$('trailerStatus').textContent=trailer?.zeroed?`영점 설정됨 · Z ${trailer.index_count}회`:'영점 미설정 / 센서 없음';
  $('servo').textContent=`명령 ${fmt(servo?.command_rad)} rad · 펄스 ${fmt(servo?.pulse_us,0)} μs · 실제각 미측정`;
  $('ranges').replaceChildren(...[0,1,2,3].flatMap(i=>{const r=fresh?vehicle.ultrasonic?.[i]:null;const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent='US-'+(i+1);dd.textContent=r?.status==='ok'?fmt(r.m*100,1)+' cm':r?.status||'—';return [dt,dd];}));
  const pose=fresh&&state?.slam_ready?state.map_pose:null;
  $('pose').textContent=pose?`MAP · x ${fmt(pose.x)} m / y ${fmt(pose.y)} m / yaw ${fmt(pose.yaw)} rad`:'MAP 위치 데이터 없음';
  $('mapAge').textContent=grid?`지도 수신 ${fmt(state?.age_s.map,1)}초 전`:'';
  drawMap();
}
function drawMap(){
  const canvas=$('map'), context=canvas.getContext('2d');canvas.width=canvas.clientWidth*devicePixelRatio;canvas.height=canvas.clientHeight*devicePixelRatio;
  if(!gridCanvas)return;
  const scale=Math.min(canvas.width/grid.width,canvas.height/grid.height)*0.94, ox=(canvas.width-grid.width*scale)/2,oy=(canvas.height-grid.height*scale)/2;
  context.imageSmoothingEnabled=false;context.drawImage(gridCanvas,ox,oy,grid.width*scale,grid.height*scale);
  const pose=performance.now()-lastState<600&&state?.slam_ready?state.map_pose:null;if(!pose)return;
  const [gx,gy,ga]=grid.origin,dx=pose.x-gx,dy=pose.y-gy;
  const px=(Math.cos(ga)*dx+Math.sin(ga)*dy)/grid.resolution,py=(-Math.sin(ga)*dx+Math.cos(ga)*dy)/grid.resolution;
  context.save();context.translate(ox+px*scale,oy+(grid.height-py)*scale);context.rotate(-(pose.yaw-ga));
  context.fillStyle='#19d6a0';context.strokeStyle='#102a30';context.lineWidth=2*devicePixelRatio;
  const r=12*devicePixelRatio;context.beginPath();context.moveTo(r,0);context.lineTo(-r*.7,-r*.65);context.lineTo(-r*.4,0);context.lineTo(-r*.7,r*.65);context.closePath();context.fill();context.stroke();context.restore();
}
$('estop').onclick=()=>{keys.clear();send('estop');};$('stop').onclick=stop;
$('claim').onclick=()=>send('claim');$('release').onclick=()=>{keys.clear();send('release');};
$('arm').onclick=()=>{keys.clear();send('arm');};
$('control').onchange=()=>{keys.clear();send('control',{control:$('control').value});};
$('zero').onclick=()=>send('zero_trailer');$('save').onclick=()=>send('save_map',{name:$('mapName').value});
for(const id of ['drive','pwm','rpm1','rpm2','steer']){$(id).oninput=labels;if(id!=='drive')$(id).onchange=()=>{keys.clear();send('limits',limits());};}
const drivingKeys=['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright'];
document.addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','SELECT'].includes(e.target.tagName)){e.preventDefault();keys.clear();send('estop');return;}if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;const k=e.key.toLowerCase();if(drivingKeys.includes(k)){e.preventDefault();keys.add(k);}});
document.addEventListener('keyup',e=>{keys.delete(e.key.toLowerCase());drive();});
document.addEventListener('focusin',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))stop();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});window.addEventListener('blur',stop);window.addEventListener('pagehide',stop);window.addEventListener('resize',drawMap);
function drive(){
  const c=state?.data.command;if(!c?.armed||c.owner!==session||document.hidden||!document.hasFocus()||performance.now()-lastState>350)return;
  const forward=(keys.has('w')||keys.has('arrowup')?1:0)-(keys.has('s')||keys.has('arrowdown')?1:0);
  const turn=(keys.has('a')||keys.has('arrowleft')?1:0)-(keys.has('d')||keys.has('arrowright')?1:0);
  const l=limits(), amount=+$('drive').value/100, extra={steering_rad:turn*l.steering};
  if(c.control==='PI')extra.rpm=l.rpm.map(r=>r*forward*amount);else extra.pwm=[1,1].map(()=>forward*amount*l.pwm);
  send('drive',extra);
}
setInterval(drive,100);setInterval(()=>{if(performance.now()-lastState>600){keys.clear();connection('상태 수신 지연 · 정지');render();}},200);labels();connect();
