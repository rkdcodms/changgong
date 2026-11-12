// No browser, network, camera or device access: exercise status rendering in a small DOM double.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '..', 'jetson', 'preview.html'), 'utf8');
const script = html.split('<script>')[1].split('</script>')[0];
const nodes = new Map();
function element() {
  return {textContent:'', className:'', hidden:false, children:[], classList:{toggle(){}, add(){}},
    replaceChildren(){this.children=[];}, append(child){this.children.push(child);}, removeAttribute(){}};
}
const document = {getElementById(id){if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);}, createElement:element};
const context = vm.createContext({document, Date, AbortController, setTimeout:()=>0, clearTimeout(){},
  setInterval(){}, fetch:()=>new Promise(()=>{})});
vm.runInContext(script, context);
const state = {ok:true, serial_ready:true, model_ready:true, inference_fps:7, fps:7,
  frame_sequence:42, rotation_ccw:90, stop_zone:{left:.4,right:.6}, stop_confidence_threshold:.55,
  stop_min_span_seconds:.1, stop_min_frames:3,
  detections:[{class_id:0,class_name:'Calypso',confidence:.85,stop_eligible:true}],
  control:{mega_mode:'STOPPED',stop_seq:1,stop_reason:'VISION',heartbeat_allowed:true,
    last_ack:{event:1,class_id:0,class_name:'Calypso',stop_seq:1}}};
function show(s){context.sample=s;vm.runInContext('show(sample)', context);}
show(state);
assert.equal(nodes.get('mode').textContent,'5초 정지 중');
assert.match(nodes.get('objects').children[0].textContent,/85.0% · 5초 정지 중/);
assert.match(nodes.get('ack').textContent,/정지 확인 기록/);
assert.match(nodes.get('zone').textContent,/3프레임 · 0.1초 이상 미리 확인/);
assert.match(nodes.get('zone').textContent,/중심이 노란 구역 40~60%에 들어오면/);
show({...state,stop_min_span_seconds:.15,stop_min_frames:5});
assert.match(nodes.get('zone').textContent,/5프레임 · 0.15초 이상/);
show({...state,serial_ready:false});
assert.match(nodes.get('mode').textContent,/현재 미확인/);
assert.equal(nodes.get('mode').className,'value warn');
show({...state,control:{...state.control,mega_mode:'FOLLOW'},detections:[{class_id:1,class_name:'Cattle',confidence:.3,stop_eligible:false}]});
assert.match(nodes.get('objects').children[0].textContent,/신뢰도 기준 미달/);
show({...state,control:{...state.control,mega_mode:'FOLLOW'},detections:[{class_id:1,class_name:'Cattle',confidence:.8,stop_eligible:false}]});
assert.match(nodes.get('objects').children[0].textContent,/중앙 진입 대기/);
show({...state, control:{...state.control,mega_mode:'FAULT',fatal:'LINK_FAULT'}});
assert.equal(nodes.get('mode').className,'value bad');
assert.equal(nodes.get('alert').hidden,false);
assert.match(nodes.get('alert').textContent,/LINK_FAULT/);
nodes.get('pause').onclick();
assert.match(nodes.get('notice').textContent,/AI 제어는 계속됩니다/);
show({...state,sensor_values:[900,900,900,100,100,900,900,900],white_count:2,normal_line:true,sensor_age_seconds:.1});
assert.equal(nodes.get('sensors').textContent,'흰색 2/8');
assert.match(nodes.get('line-alignment').textContent,/중앙 센서에 선이 맞습니다/);
show({...state,white_count:8,normal_line:false,sensor_age_seconds:.1});
assert.match(nodes.get('line-alignment').textContent,/선 맞춤 필요/);
show({...state,white_count:2,normal_line:true,sensor_age_seconds:2});
assert.match(nodes.get('sensors').textContent,/최근/);
assert.match(nodes.get('line-alignment').textContent,/현재 선 위치 미확인/);
assert.match(html,/\.stage img\{position:absolute;inset:0;display:block/);
const cyclops={class_id:2,class_name:'Cyclopes',confidence:.982,stop_eligible:true};
const following={...state,control:{mega_mode:'FOLLOW',stop_reason:'VISION',stop_seq:2,latched_classes:[],fatal:null,
  total_accepted_stops:2,last_ack:{event:2,class_id:2,class_name:'Cyclopes',stop_seq:2}}};
function label(item,s){context.itemCase=item;context.stateCase=s;return vm.runInContext('detectionState(itemCase,stateCase)',context);}
assert.equal(label(cyclops,following),'정지 조건 확인 중');
assert.equal(label(cyclops,{...following,control:{...following.control,latched_classes:[2]}}),'이미 정지함 · 재인식 대기');
assert.equal(label(cyclops,{...following,control:{...following.control,pending_event:{event:3,class_id:2},latched_classes:[2]}}),'정지 응답 대기');
assert.equal(label(cyclops,{...following,control:{...following.control,mega_mode:'STOPPED'}}),'5초 정지 중');
assert.equal(label({...cyclops,class_id:1},{...following,control:{...following.control,mega_mode:'STOPPED'}}),'다른 동상 정지 중');
assert.equal(label(cyclops,{...following,control:{...following.control,mega_mode:'STOPPED',stop_seq:3}}),'정지 상태 확인 중');
assert.equal(label(cyclops,{...following,control:{...following.control,mega_mode:'STOPPED',last_ack:null}}),'정지 상태 확인 중');
assert.equal(label({...cyclops,confidence:.3},following),'신뢰도 기준 미달');
assert.equal(label({...cyclops,stop_eligible:false},following),'중앙 진입 대기');
assert.equal(label(cyclops,{...following,ok:false}),'현재 상태 확인 불가');
assert.equal(label(cyclops,{...following,serial_ready:false}),'현재 상태 확인 불가');
assert.equal(label(cyclops,{...following,serial_ready:false,control:{...following.control,mega_mode:'FAULT'}}),'현재 상태 확인 불가');
assert.equal(label(cyclops,{...following,model_ready:undefined}),'현재 상태 확인 불가');
assert.equal(label(cyclops,{...following,control:{...following.control,mega_mode:'FAULT'}}),'제어 오류 · 정지 유지');
assert.equal(label(cyclops,{...following,control:{...following.control,fatal:'RESET'}}),'제어 오류 · 정지 확인 필요');
assert.ok(!html.includes('정지 후보'));
assert.match(html,/화면 v3/);
assert.match(html,/화면 밖으로 완전히 빼고 2초 이상/);
show({...following,detections:[cyclops],control:{...following.control,latched_classes:[2]}});
assert.match(nodes.get('objects').children[0].textContent,/이미 정지함 · 재인식 대기/);
assert.match(html,/<p id="view-version" class="small">화면 v3<\/p>/);
const syntheticFixture=JSON.parse(fs.readFileSync(path.join(__dirname,'fixtures','status.json'),'utf8'));
show(syntheticFixture);
assert.match(nodes.get('objects').children[0].textContent,/Cyclopes · 신뢰도 98.2% · 이미 정지함 · 재인식 대기/);
async function checkNetworkFailure(){
  context.fetch=()=>Promise.reject(new Error('offline'));
  vm.runInContext('polling=false',context);
  await vm.runInContext('poll()',context);
  await vm.runInContext('poll()',context);
  assert.match(nodes.get('objects').children[0].textContent,/현재 상태 확인 불가/);
  console.log('Preview v3: 42 assertions passed');
}
checkNetworkFailure().catch(error=>{console.error(error);process.exitCode=1;});
