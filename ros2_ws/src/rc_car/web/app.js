'use strict';
const $ = id => document.getElementById(id);
const fmt = (x, d=2) => typeof x === 'number' && Number.isFinite(x) ? x.toFixed(d) : '—';
let ws, sequence=0, serverTime=null, state=null, session=null, lastState=0, grid=null, gridCanvas=null;
const keys=new Set();
let parkingPaths=null, parkingGeometry={}, goal=null, dragStart=null;
let mapParts=null;
function log(message){$('events').textContent=(new Date().toLocaleTimeString()+' '+message+'\n'+$('events').textContent).slice(0,4000);}
function send(type, extra={}){if(ws?.readyState===WebSocket.OPEN)ws.send(JSON.stringify({v:1,seq:++sequence,type,server_time:serverTime,...extra}));}
function stop(){keys.clear();send('stop');}
function stopManual(){keys.clear();const c=state?.data.command;if(c?.owner===session&&c.mode==='MANUAL'&&c.armed)send('stop');}
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
      $('control').querySelector('[value="PI"]').disabled=msg.wheel_pi_configured===false;
      parkingGeometry=msg.parking_config?.parking||{};
      grid=null;gridCanvas=null;goal=null;parkingPaths=null;$('mapEmpty').hidden=false;
      mapParts=null;
      $('goalText').textContent='목표 미지정';
      for(const [id,value] of [['pwm',msg.limits.pwm],['rpm1',msg.limits.rpm[0]],['rpm2',msg.limits.rpm[1]],['steer',msg.limits.steering]]){$(id).max=value;$(id).value=value;}
      labels();log('접속 완료. 제어권 획득 후 활성화하십시오.');
    } else if(msg.type==='state'){
      state=msg;serverTime=msg.server_time;lastState=performance.now();connection('WebSocket 연결',true);render();
    } else if(msg.type==='parking_paths'){
      parkingPaths=msg;drawMap();
    } else if(msg.type==='ack'){
      log(msg.ok?`명령 ${msg.seq} 완료${msg.saved?' · '+msg.saved.join(', '):''}`:`거부: ${msg.error}`);
    } else if(msg.type==='map_chunk'){
      if(!mapParts||mapParts.version!==msg.version||mapParts.next!==msg.offset){
        mapParts={version:msg.version,width:msg.width,height:msg.height,resolution:msg.resolution,
          origin:msg.origin,frame:msg.frame,next:0,canvas:document.createElement('canvas')};
        mapParts.canvas.width=msg.width;mapParts.canvas.height=msg.height;
        mapParts.pixels=mapParts.canvas.getContext('2d').createImageData(msg.width,msg.height);
      }
      if(mapParts.next===msg.offset&&msg.total===msg.width*msg.height){
        paintMapCells(mapParts,msg.data,msg.offset);
        mapParts.next+=msg.data.length;
        if(mapParts.next===msg.total){
          mapParts.canvas.getContext('2d').putImageData(mapParts.pixels,0,0);
          grid={version:mapParts.version,width:mapParts.width,height:mapParts.height,
            resolution:mapParts.resolution,origin:mapParts.origin,frame:mapParts.frame};
          gridCanvas=mapParts.canvas;
          $('mapEmpty').hidden=true;drawMap();
          mapParts=null;
        }
      }
    } else if(msg.type==='map'){
      showMap(msg);
    }
  };
}
function showMap(msg){
  grid=msg;gridCanvas=document.createElement('canvas');gridCanvas.width=msg.width;gridCanvas.height=msg.height;
  const context=gridCanvas.getContext('2d'), pixels=context.createImageData(msg.width,msg.height);
  paintMapCells({width:msg.width,height:msg.height,pixels},msg.data,0);
  context.putImageData(pixels,0,0);$('mapEmpty').hidden=true;drawMap();
}
function paintMapCells(target,values,offset){
  const {width,height,pixels}=target;
  for(let i=0;i<values.length;i++){
    const value=values[i],cell=offset+i;
    const index=((height-1-Math.floor(cell/width))*width+cell%width)*4;
    const grey=value<0?64:Math.round(235-value*2.1);
    pixels.data.set([grey,grey+(value<0?8:0),grey+(value<0?12:0),255],index);
  }
}
function render(){
  const data=state?.data||{}, command=data.command||{}, vehicle=data.vehicle||{};
  const fresh=state&&performance.now()-lastState<600&&(state.age_s.vehicle??999)<0.6;
  const commandFresh=state&&(state.age_s.command??999)<0.5;
  $('mode').textContent=command.mode||'MANUAL';
  const parking=command.parking||{};
  const phase={idle:'대기',planning:'경로 계산 중',planned:'경로 준비됨',tracking:'경로 추종 중',stopped:'정지',failed:'실패',complete:'주차 완료'};
  $('parkStatus').textContent=[parking.online?'노트북 연결됨':'노트북 오프라인',phase[parking.phase],
    fresh&&state.odom_source!=='lidar'&&vehicle.motors?.filter(m=>m.feedback_valid).length!==2?'양쪽 엔코더 속도 확인 필요':'',
    parking.detail,parking.blocked,parking.error_m!=null?'트레일러 추종 오차 '+fmt(parking.error_m)+' m':''].filter(Boolean).join(' · ');
  const owner=fresh&&commandFresh&&session&&command.owner===session;
  $('plan').disabled=!(owner&&parking.online&&goal&&grid&&goal.version===grid.version&&!command.armed);
  $('clearParkStop').disabled=!(owner&&command.estop&&!command.armed);
  $('startPark').disabled=!(owner&&command.auto_available&&!command.estop&&!command.armed&&parking.job&&parking.job.map_version===grid?.version
    &&goal&&JSON.stringify(parking.job.goal)===JSON.stringify([...goal.pose,Number($('goalBeta').value)*Math.PI/180]));
  if(command.control&&document.activeElement!==$('control'))$('control').value=command.control;
  $('owner').textContent=command.owner===session&&session?'이 브라우저에 제어권 있음':command.owner?'다른 클라이언트가 제어 중':'관찰 모드 · 제어권 없음';
  $('reason').textContent=`${commandFresh?(command.armed?'활성화':'출력 차단'):'제어 상태 오래됨'} · ${command.reason||'상태 대기'}${command.estop?' · 비상정지 래치':''}`;
  $('arm').disabled=!fresh||!commandFresh||command.owner!==session||!vehicle.ready;
  $('fresh').textContent=fresh?'실시간 수신':'데이터 없음 / 오래됨';$('fresh').className='badge '+(fresh?'ok':'warn');
  const slamReady=performance.now()-lastState<600&&state?.slam_ready;
  $('slam').textContent=slamReady?'SLAM 데이터 준비됨':'SLAM 준비 안 됨';$('slam').className='badge '+(slamReady?'ok':'warn');
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
  const pose=slamReady?state.map_pose:null;
  const match=state?.odom_source==='lidar'?null:state?.data.scan_match;
  const previewPose=match&&performance.now()-lastState<600&&state.ros_time_s-match.stamp_s>=0&&state.ros_time_s-match.stamp_s<0.5?match:null;
  const shown=pose||previewPose;
  $('pose').textContent=shown?`${pose?'MAP':'스캔 매칭 · 계획 전용'} · x ${fmt(shown.x)} m / y ${fmt(shown.y)} m / yaw ${fmt(shown.yaw)} rad`:'MAP 위치 데이터 없음';
  if(state?.odom_source==='lidar'&&performance.now()-lastState<600){
    const odom=data.odometry_status;
    $('pose').textContent += odom?.valid?' · 라이다 추정 속도 '+fmt(data.odom?.speed_m_s)+' m/s':' · '+(odom?.reason||'라이다 대기');
  }
  $('mapAge').textContent=grid?`지도 수신 ${fmt(state?.age_s.map,1)}초 전`:'';
  drawMap();
}
function drawMap(){
  const canvas=$('map'), context=canvas.getContext('2d');canvas.width=canvas.clientWidth*devicePixelRatio;canvas.height=canvas.clientHeight*devicePixelRatio;
  if(!gridCanvas)return;
  const scale=Math.min(canvas.width/grid.width,canvas.height/grid.height)*0.94, ox=(canvas.width-grid.width*scale)/2,oy=(canvas.height-grid.height*scale)/2;
  context.imageSmoothingEnabled=false;context.drawImage(gridCanvas,ox,oy,grid.width*scale,grid.height*scale);
  const [gx,gy,ga]=grid.origin;
  function pixel(x,y){
    const dx=x-gx,dy=y-gy;
    return [ox+(Math.cos(ga)*dx+Math.sin(ga)*dy)*scale/grid.resolution,
            oy+(grid.height-(-Math.sin(ga)*dx+Math.cos(ga)*dy)/grid.resolution)*scale];
  }
  function line(points,color,width){
    context.beginPath();
    points.forEach(([x,y],i)=>context[i?'lineTo':'moveTo'](...pixel(x,y)));
    context.strokeStyle=color;context.lineWidth=width*devicePixelRatio;context.stroke();
  }
  const job=state?.data.command?.parking?.job;
  if(parkingPaths&&job?.id===parkingPaths.job_id&&job.map_version===grid.version){
    line(parkingPaths.hybrid,'#ffad49',5);line(parkingPaths.reference,'#43c6ff',2);
  }
  if(goal&&goal.version===grid.version){
    const [x,y,a]=goal.pose;
    line([[x,y],[x+Math.cos(a)*grid.resolution*12,y+Math.sin(a)*grid.resolution*12]],'#fd7bc8',3);
    context.beginPath();context.arc(...pixel(x,y),4*devicePixelRatio,0,2*Math.PI);context.fillStyle='#fd7bc8';context.fill();
  }
  const match=state?.odom_source==='lidar'?null:state?.data.scan_match;
  const pose=performance.now()-lastState<600?(state?.map_pose||(match&&state.ros_time_s-match.stamp_s>=0&&state.ros_time_s-match.stamp_s<0.5?match:null)):null;
  if(!pose)return;
  const t=state?.data.vehicle?.trailer,h=parkingGeometry.hitch_back_m,b=parkingGeometry.drawbar_m;
  if(state.age_s.vehicle<0.3&&t?.zeroed&&!t.invalid&&Number.isFinite(t.angle_rad)&&Number.isFinite(h)&&Number.isFinite(b)){
    const hitch=[pose.x-h*Math.cos(pose.yaw),pose.y-h*Math.sin(pose.yaw)],a=pose.yaw-t.angle_rad;
    line([[pose.x,pose.y],hitch,[hitch[0]-b*Math.cos(a),hitch[1]-b*Math.sin(a)]],'#dda5ff',5);
  }
  const dx=pose.x-gx,dy=pose.y-gy;
  const px=(Math.cos(ga)*dx+Math.sin(ga)*dy)/grid.resolution,py=(-Math.sin(ga)*dx+Math.cos(ga)*dy)/grid.resolution;
  context.save();context.translate(ox+px*scale,oy+(grid.height-py)*scale);context.rotate(-(pose.yaw-ga));
  context.fillStyle='#19d6a0';context.strokeStyle='#102a30';context.lineWidth=2*devicePixelRatio;
  const r=12*devicePixelRatio;context.beginPath();context.moveTo(r,0);context.lineTo(-r*.7,-r*.65);context.lineTo(-r*.4,0);context.lineTo(-r*.7,r*.65);context.closePath();context.fill();context.stroke();context.restore();
}
$('estop').onclick=()=>{keys.clear();send('estop');};$('stop').onclick=stop;
function mapPoint(event){
  const box=$('map').getBoundingClientRect(),scale=Math.min(box.width/grid.width,box.height/grid.height)*.94;
  const dx=((event.clientX-box.left-(box.width-grid.width*scale)/2)/scale)*grid.resolution;
  const dy=(grid.height-(event.clientY-box.top-(box.height-grid.height*scale)/2)/scale)*grid.resolution;
  const [x,y,a]=grid.origin;
  return [x+dx*Math.cos(a)-dy*Math.sin(a),y+dx*Math.sin(a)+dy*Math.cos(a)];
}
$('map').onpointerdown=e=>{if(grid){dragStart={xy:mapPoint(e),version:grid.version};$('map').setPointerCapture(e.pointerId);}};
$('map').onpointerup=e=>{
  if(!grid||dragStart?.version!==grid.version)return;
  const [x,y]=dragStart.xy,[ex,ey]=mapPoint(e);dragStart=null;
  if(Math.hypot(ex-x,ey-y)<grid.resolution)return;
  goal={version:grid.version,pose:[x,y,Math.atan2(ey-y,ex-x)]};
  $('goalText').textContent='트레일러 차축 목표 '+goal.pose.map(v=>fmt(v)).join(' / ')+' (m, m, rad)';render();
};
$('plan').onclick=()=>{if(goal)send('park_plan',{goal:[...goal.pose,Number($('goalBeta').value)*Math.PI/180],map_version:goal.version});};
$('clearParkStop').onclick=()=>send('park_reset');
$('startPark').onclick=()=>{keys.clear();send('mode',{mode:'AUTO_PARK'});};
$('cancelPark').onclick=stop;
$('goalBeta').oninput=render;
$('claim').onclick=()=>send('claim');$('release').onclick=()=>{keys.clear();send('release');};
$('arm').onclick=()=>{keys.clear();send('arm');};
$('control').onchange=()=>{keys.clear();send('control',{control:$('control').value});};
$('zero').onclick=()=>send('zero_trailer');$('save').onclick=()=>send('save_map',{name:$('mapName').value});
for(const id of ['drive','pwm','rpm1','rpm2','steer']){$(id).oninput=labels;if(id!=='drive')$(id).onchange=()=>{keys.clear();send('limits',limits());};}
const drivingKeys=['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright'];
document.addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','SELECT'].includes(e.target.tagName)){e.preventDefault();keys.clear();send('estop');return;}if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;const k=e.key.toLowerCase();if(drivingKeys.includes(k)){e.preventDefault();keys.add(k);}});
document.addEventListener('keyup',e=>{keys.delete(e.key.toLowerCase());drive();});
document.addEventListener('focusin',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))stopManual();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopManual();});window.addEventListener('blur',stopManual);window.addEventListener('pagehide',stopManual);window.addEventListener('resize',drawMap);
function drive(){
  const c=state?.data.command;if(!c?.armed||c.owner!==session||document.hidden||!document.hasFocus()||performance.now()-lastState>350)return;
  const forward=(keys.has('w')||keys.has('arrowup')?1:0)-(keys.has('s')||keys.has('arrowdown')?1:0);
  const turn=(keys.has('a')||keys.has('arrowleft')?1:0)-(keys.has('d')||keys.has('arrowright')?1:0);
  const l=limits(), amount=+$('drive').value/100, extra={steering_rad:turn*l.steering};
  if(c.control==='PI')extra.rpm=l.rpm.map(r=>r*forward*amount);else extra.pwm=[1,1].map(()=>forward*amount*l.pwm);
  send('drive',extra);
}
setInterval(drive,100);setInterval(()=>{if(performance.now()-lastState>600){keys.clear();connection('상태 수신 지연 · 정지');render();}},200);labels();connect();
