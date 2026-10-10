'use strict';
const $ = (id) => document.getElementById(id);
const token = document.querySelector('meta[name="powerdesk-token"]').content;
let state = {connected:false, opened:false, caps:{}, telemetry:{}, age:{}, samples:[], events:[]};
let cursor = 0, history = [], busy = false, capturing = false, pidKey = '', initialTargets = new Set();
let toastTimer, pollFailures = 0, quitting = false, lastTrace = null, currentSession = -1, lastGateRead = 0;
const stateNames = ['启动中','自检中','输出关闭','自举预充','软启动','恒压 CV','恒流 CC','正在停止','故障锁存','电流调试'];
const faults = [[1,'硬件过流'],[2,'传感器诊断'],[4,'输出过压'],[8,'过温'],[16,'通信超时'],[32,'采样异常'],[64,'输入异常'],[128,'内部保护']];
const pidFields = [['kp','Kp · 比例'],['ki','Ki · 积分'],['kd','Kd · 微分'],['tau','τ · D 滤波时间 / s'],['out_min','控制量下限'],['out_max','控制量上限']];

async function api(path, body) {
  const response = await fetch(path, {method: body === undefined ? 'GET' : 'POST', headers:{'X-PowerDesk-Token':token, ...(body === undefined ? {} : {'Content-Type':'application/json'})}, body: body === undefined ? undefined : JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `请求失败 ${response.status}`);
  return data;
}
function toast(text, error=false) {
  $('toast').textContent=text; $('toast').className=`toast show${error?' error':''}`;
  clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('toast').classList.remove('show'), error?7000:3500);
}
async function work(fn) {
  if (busy) return;
  busy=true; renderControls();
  try { await fn(); } catch(e) { toast(e.message,true); }
  finally { busy=false; renderControls(); }
}
async function action(name, channel=0, values={}) { return (await api('/api/action',{name,channel,values})).result; }
function number(id) {
  const raw=$(id).value.trim();
  if (!raw || !Number.isFinite(Number(raw))) throw new Error('请填写有效数值');
  return Number(raw);
}
function milli(id) {
  const n=number(id)*1000;
  if (Math.abs(n-Math.round(n))>1e-6) throw new Error('电压、电流最多保留三位小数');
  return Math.round(n);
}
function ready(ch) { return state.connected && !state.lease_expired && state.age[ch] !== undefined && state.age[ch] <= .5; }function off(ch) { const t=state.telemetry[ch]; return t && !t.enabled && [2,8].includes(t.state); }
function supports(ch, feature) { return Boolean((state.caps[ch]?.features || 0) & feature); }
function faultText(bits) { return faults.filter(([mask])=>bits&mask).map(([,name])=>name).join('、') || (bits ? `未知故障 0x${bits.toString(16)}` : '无故障'); }
function valid(t,bit) { return t && Boolean(t.valid & (1<<bit)); }
function measurement(t,key,bit,scale=1000,digits=3) { return valid(t,bit) ? (t[key]/scale).toFixed(digits) : '—'; }
function text(id,value) { $(id).textContent=value; }

// ---------------------------------------------------------------- 输出电压模式
// 30 V 是分界点：<= 30 V 为大电流模式（可用到 5 A），> 30 V 为高压模式（电流上限收到 3 A）。
// 模式由设定电压自动决定，不单独设一个开关 —— 这样设定值和电流上限不可能互相矛盾。
const MODE_SPLIT_V=30, MODE_LV_MAX_A=5, MODE_HV_MAX_A=3, WHEEL_STEP_V=0.1, WHEEL_STEP_A=0.1;
function modeFor(v){
  const hv=Number.isFinite(v)&&v>MODE_SPLIT_V;
  return {hv, cap:hv?MODE_HV_MAX_A:MODE_LV_MAX_A,
          name:hv?'高压模式':'大电流模式'};
}
// 把模式规则落到界面：电流框的 max、被顶到上限的值、以及那一行文字提示。
function applyModeLimits(ch,announce){
  const vEl=$(`vset-${ch}`), iEl=$(`iset-${ch}`), note=$(`mode-note-${ch}`);
  if(!vEl||!iEl) return;
  const devMax=Number(iEl.dataset.devMax||Infinity);
  const m=modeFor(Number(vEl.value));
  const cap=Math.min(m.cap,devMax);
  iEl.max=cap;
  const i=Number(iEl.value);
  if(Number.isFinite(i)&&i>cap){
    iEl.value=cap;
    if(announce) toast(`CH${ch} ${m.name}：输出电流上限最多 ${cap} A，已由 ${i} A 收到 ${cap} A`);
  }
  if(note){
    note.className='mode-note '+(m.hv?'hv':'lv');
    note.textContent=m.hv
      ? `高压模式 · 设定电压 > ${MODE_SPLIT_V} V，输出电流上限限制为 ${cap} A`
      : `大电流模式 · 设定电压 ≤ ${MODE_SPLIT_V} V，输出电流可用到 ${cap} A`;
  }
}
// 滚轮按固定步长调整，并夹在 min/max 之间；同时把模式规则重新落一遍。
function wheelStep(id,step,after){
  const el=$(id); if(!el) return;
  el.addEventListener('wheel',e=>{
    if(el.disabled) return;
    e.preventDefault();
    const cur=Number(el.value); if(!Number.isFinite(cur)) return;
    const min=el.min===''?-Infinity:Number(el.min), max=el.max===''?Infinity:Number(el.max);
    const dir=e.deltaY<0?1:-1;
    // 先归到步长网格再走一格，避免 0.1 反复累加出 3.0000000000000004 这种尾巴。
    let next=Math.round((cur+dir*step)/step)*step;
    next=Math.min(max,Math.max(min,Number(next.toFixed(3))));
    if(next!==cur){ el.value=next; if(after) after(); }
  },{passive:false});
}

for (const ch of [1,2]) {
  const article=document.createElement('article'); article.className='panel channel';
  article.innerHTML=`<div class="channel-top"><span class="channel-number">CH ${ch.toString().padStart(2,'0')}</span><strong>独立输出 ${ch}</strong><span class="state" id="state-${ch}">等待连接</span></div>
    <div class="measurements"><div><div class="measure-label">输出电压</div><div class="reading"><span id="vout-${ch}">—</span><small>V</small></div></div><div><div class="measure-label">输出电流</div><div class="reading"><span id="iout-${ch}">—</span><small>A</small></div></div></div>
    <div class="secondary-metrics"><div><span>输出功率</span><b id="power-${ch}">— W</b></div><div><span>输入电压 / 电流</span><b id="input-${ch}">— / —</b></div><div><span>电感电流</span><b id="il-${ch}">— A</b></div></div>
    <div class="targets"><div class="form-row"><label>设定电压 / V<input id="vset-${ch}" aria-label="CH${ch}设定电压" type="number" min="0" step="0.001" placeholder="等待设备量程" disabled></label><label>输出电流上限 / A<input id="iset-${ch}" aria-label="CH${ch}电流上限" type="number" min="0" step="0.001" placeholder="等待设备量程" disabled></label></div><div class="mode-note lv" id="mode-note-${ch}">大电流模式 · 设定电压 ≤ 30 V，输出电流可用到 5 A</div><div class="channel-bottom"><span id="target-readback-${ch}">设备目标：—</span><span id="range-${ch}">范围待上报</span></div><div class="target-actions"><button class="ghost apply" id="apply-${ch}" disabled>应用设定</button><button class="primary" id="start-${ch}" disabled>开启输出</button><button class="ghost" id="stop-${ch}" disabled>停止</button></div></div>
    <div class="channel-bottom"><span id="temperatures-${ch}">MOS — °C · 电感 — °C</span><span id="fault-${ch}">等待设备</span><button id="clear-${ch}" class="clear-fault" disabled>清除故障</button></div>`;
  $('channel-grid').append(article);
  $(`apply-${ch}`).onclick=()=>work(async()=>{
    // 这里是唯一真正决定"发什么出去"的地方，所以模式限制必须在这里再校验一次：
    // input.max 只是浏览器提示，拦不住手输或滚轮的结果。
    const v=Number($(`vset-${ch}`).value), i=Number($(`iset-${ch}`).value);
    if(!Number.isFinite(v)||!Number.isFinite(i)){ toast(`CH${ch} 设定值不是有效数字，未下发`); return; }
    applyModeLimits(ch,true);                       // 按当前电压把电流框收到模式上限
    const sendV=Number($(`vset-${ch}`).value), sendI=Number($(`iset-${ch}`).value);
    const m=modeFor(sendV);
    if(sendI>m.cap){ toast(`CH${ch} ${m.name} 只允许 ${m.cap} A，${sendI} A 未下发`); return; }
    const d=await action('target',ch,{vref_mv:milli(`vset-${ch}`),iref_ma:milli(`iset-${ch}`)});
    toast(`CH${ch} 目标已确认：${d.vref_mv/1000} V / ${d.iref_ma/1000} A · ${m.name}`);
  });
  $(`start-${ch}`).onclick=()=>work(async()=>{await action('output',ch,{enabled:1});toast(`CH${ch} 启动命令已确认，等待设备运行状态`);});
  $(`stop-${ch}`).onclick=()=>stop(ch);
  $(`clear-${ch}`).onclick=()=>work(async()=>{await action('clear_fault',ch);toast(`CH${ch} 清故障命令已确认；不会自动启动`);});
  // 电压滚轮每格 0.1 V；电流也给同样的手感。两者都会重新落实模式规则。
  wheelStep(`vset-${ch}`,WHEEL_STEP_V,()=>applyModeLimits(ch,true));
  wheelStep(`iset-${ch}`,WHEEL_STEP_A,()=>applyModeLimits(ch,false));
  $(`vset-${ch}`).addEventListener('input',()=>applyModeLimits(ch,false));
  $(`iset-${ch}`).addEventListener('input',()=>applyModeLimits(ch,false));
}
for(const [key,label] of pidFields){const el=document.createElement('label');el.textContent=label;const input=document.createElement('input');input.id=`pid-${key}`;input.type='number';input.step='any';input.placeholder='先读取设备参数';input.disabled=true;el.append(input);$('pid-fields').append(el);}

function renderControls() {
  const connected=state.connected;
  $('connect').textContent=state.opened?'断开连接':'连接设备';
  $('connect').disabled=busy || (!state.opened && !$('port').value);
  $('port').disabled=state.opened||busy; $('refresh').disabled=state.opened||busy;
  $('demo').disabled=state.opened||busy;
  $('stop-all').disabled=!state.opened;
  const zeroUsable=ready(1)&&supports(1,64)&&!busy&&!capturing;
  $('zero-read').disabled=!zeroUsable;
  for(const id of ['zero-iout','zero-il','zero-reset-iout','zero-reset-il'])
    $(id).disabled=!zeroUsable||!off(1)||state.telemetry[1]?.state!==2||state.telemetry[1]?.mode!==0;
  for(const ch of [1,2]){
    const t=state.telemetry[ch], usable=ready(ch)&&!busy;
    $(`apply-${ch}`).disabled=!usable||!supports(ch,1);
    $(`vset-${ch}`).disabled=$(`iset-${ch}`).disabled=!usable||!supports(ch,1);
    $(`start-${ch}`).disabled=!usable||!supports(ch,1)||!t?.ready||Boolean(t?.fault)||t?.enabled||t?.mode!==0||!off(ch);
    $(`stop-${ch}`).disabled=!state.opened;
    $(`clear-${ch}`).disabled=!usable||!off(ch)||!t?.fault;
  }
  const ch=Number($('debug-channel').value), usable=ready(ch)&&!busy, t=state.telemetry[ch];
  const pidAvailable=usable&&supports(ch,4);
  $('read-pid').disabled=!pidAvailable;
  $('write-pid').disabled=!pidAvailable||!off(ch)||t?.state!==2||pidKey!==`${ch}:${$('loop').value}`;
  for(const [key] of pidFields) $(`pid-${key}`).disabled=!pidAvailable||pidKey!==`${ch}:${$('loop').value}`;
  $('capture').disabled=!ready(ch)||!supports(ch,8)||capturing||busy;
  $('export-trace').disabled=!lastTrace;
  $('apply-mode').disabled=!usable||!off(ch)||t?.state!==2||!supports(ch,Number($('mode').value)?2:1);
  $('apply-debug').disabled=!usable||!off(ch)||t?.state!==2||!supports(ch,2)||t?.mode!==1;
  $('debug-start').disabled=!usable||!t?.ready||Boolean(t?.fault)||t?.enabled||!off(ch)||!supports(ch,t?.mode===1?2:1);
  const gateUsable=ready(1)&&!busy&&!state.telemetry[1]?.fault&&off(1);
  for(const id of ['gate-1','gate-2','gate-3','gate-4','gate-confirm','gate-clear']) $(id).disabled=!gateUsable;
  $('debug-stop').disabled=!state.opened;
  $('demo-fault').disabled=!connected||!state.demo||busy;
  $('export').disabled=!history.length;
}

function render() {
  const anyStale=state.connected && [1,2].some(ch=>!ready(ch));
  $('connection-pill').className=`connection-pill${state.connected&&!anyStale?' online':''}`;
  $('connection-pill').querySelector('span').textContent=state.connected?(state.demo?'演示设备已连接':`${state.port} 已连接`):state.opened?'连接已中断':'未连接设备';
  if(state.lease_expired){ $('banner').className='banner error';text('banner','会话已失效（界面心跳或设备遥测超时）：已请求停止输出，设备心跳不再续期。请断开并重新连接，输出不会自动恢复。'); }
  else if(anyStale || (!state.connected&&state.opened)){ $('banner').className='banner error';text('banner','通信中断或测量数据已过期。保留值仅供参考，实际输出状态未知；控制已禁止，请检查设备。'); }
  else if(state.demo && state.connected){$('banner').className='banner demo';text('banner','演示模式 · 所有数据、量程、PID 与波形均为合成示例，不连接功率硬件，也不能用于验证环路稳定性。');}
  else if(state.connected){$('banner').className='banner neutral';const staging=Object.values(state.caps).some(c=>c.features&16),nominal=Object.values(state.caps).some(c=>c.features&32);const c1=state.caps[1];text('banner',staging?`已连接 MCU · CH1 受限 Buck ${c1?`${c1.vmin_mv/1000}～${c1.vmax_mv/1000} V（设备上报）`:'量程待读取'}：启动前需完成栅极实测确认与输出电流校零。${nominal?'测量使用标称参考电压，尚未实测校准。':''}`:'已连接真实设备 · 输出状态以 MCU 回报为准。关闭、长时间挂起此页面或断开串口会请求停机；硬件保护由 MCU 独立执行。');}
  else {$('banner').className='banner neutral';text('banner','尚未连接。可进入演示模式查看界面，或选择 CH340 对应的串口。');}
  for(const ch of [1,2]){
    const t=state.telemetry[ch], c=state.caps[ch];
    const stale=!ready(ch);
    text(`state-${ch}`,t?(stale?'状态未确认':stateNames[t.state]||`状态 ${t.state}`):'等待连接');
    $(`state-${ch}`).className=`state${!stale&&t?.fault?' fault':!stale&&t?.enabled?' running':''}`;
    text(`vout-${ch}`,measurement(t,'vout_mv',2));text(`iout-${ch}`,measurement(t,'iout_ma',3));
    text(`power-${ch}`,valid(t,2)&&valid(t,3)?`${(t.vout_mv*t.iout_ma/1e6).toFixed(2)} W`:'— W');
    text(`input-${ch}`,`${measurement(t,'vin_mv',0,1000,1)} V / ${measurement(t,'iin_ma',1,1000,2)} A`);
    text(`il-${ch}`,`${measurement(t,'il_ma',4)} A`);
    text(`temperatures-${ch}`,`MOS ${measurement(t,'tmos_cdeg',5,100,1)} °C · 电感 ${measurement(t,'tl_cdeg',6,100,1)} °C`);
    text(`fault-${ch}`,t?faultText(t.fault):'等待设备');
    text(`target-readback-${ch}`,t?`设备目标：${(t.vref_mv/1000).toFixed(3)} V / ${(t.iref_ma/1000).toFixed(3)} A`:'设备目标：—');
    if(c){ text(`range-${ch}`,`${c.features&16?'预置 ':''}${c.vmin_mv/1000}–${c.vmax_mv/1000} V / ≤${c.imax_ma/1000} A`);$(`vset-${ch}`).min=c.vmin_mv/1000;$(`vset-${ch}`).max=c.vmax_mv/1000;$(`iset-${ch}`).max=c.imax_ma/1000;$(`iset-${ch}`).dataset.devMax=c.imax_ma/1000;applyModeLimits(ch,false); }
    if(ch===1&&c){ text('ch1-intro',`CH1 受限 Buck：设备上报 ${c.vmin_mv/1000}～${c.vmax_mv/1000} V（运行中还会按实测输入电压复核，超出即拒绝并说明原因）、电压外环 1 kHz、慢速限流。未完成栅极实测确认前，固件拒绝启动功率级。`); }
    if(t&&state.connected&&!initialTargets.has(ch)){ $(`vset-${ch}`).value=t.vref_mv/1000;$(`iset-${ch}`).value=t.iref_ma/1000;initialTargets.add(ch);applyModeLimits(ch,false); }
  }
  text('updated',state.connected?`${state.demo?'演示数据 · ':''}${anyStale?'数据已过期':'遥测接收中'}`:'等待设备测量数据');
  if(state.connected&&!state.demo&&!busy&&Date.now()-lastGateRead>3000){lastGateRead=Date.now();refreshGate();}
  const ch=Number($('debug-channel').value),c=state.caps[ch],t=state.telemetry[ch];
  text('debug-state',t?`CH${ch} · ${ready(ch)?stateNames[t.state]||t.state:'状态未确认'} · ${t.mode?'电流调试':'正常模式'}`:'等待设备');
  text('debug-limits',c?`MCU 上报：电流调试 ≤ ${c.ilmax_ma/1000} A，最长 ${c.max_debug_ms} ms。当前目标 ${t?.ilref_ma/1000||0} A。到时由 MCU 停机，改变页面不会延长运行。`:'未收到硬件能力信息，控制暂不可用。');
  updateLoopUnits();
  const rows=[['串口',state.port||'未连接'],['固件版本',state.info?.firmware||'—'],['设备标识',state.info?.device_id||'—'],['协议定义校验',state.schema_crc||'—'],['通信看门狗',state.info?.watchdog_ms?`${state.info.watchdog_ms} ms`:'—'],['帧解析错误',String(state.crc_errors||0)],['数据来源',state.opened?(state.demo?'演示合成数据':'真实 MCU'):'未连接']];
  $('device-info').replaceChildren(...rows.flatMap(([k,v])=>{const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v;return[dt,dd];}));
  text('log-count',`${state.events.length} 条`);
  showRestore(state.restore);
  if(state.events.length){$('events').replaceChildren(...state.events.slice(-100).reverse().map(e=>{const row=document.createElement('div');row.className=`event ${e.level}`;const time=document.createElement('time');time.textContent=e.time;const message=document.createElement('span');message.textContent=e.message;row.append(time,message);return row;}));}
  text('footer-status',state.connected?`${state.demo?'DEMO / ':''}${state.port} · 115200 baud · CRC16`:'本机 USB 串口 / 等待连接');
  text('trend-note',history.length?`${state.demo?'演示 · ':''}CH${$('trend-channel').value} · ${history.filter(p=>p.channel===Number($('trend-channel').value)).length} 个缓存点`:'等待测量数据');
  const shadow=state.pid_shadow?.[Number($('debug-channel').value)];
  text('pid-shadow',!state.connected?'未连接 · 无实时影子计算':shadow&&shadow.age<2?`${shadow.message} · 仅计算，未驱动 PWM`:'暂无新记录：需 MCU 0.2.0、CH1 IL 调试模式；记录仅供慢速观察');
  renderControls();drawTrend();drawCapture();
}

function updateLoopUnits(){
  const ch=Number($('debug-channel').value),loop=Number($('loop').value),c=state.caps[ch];
  const names=[['电流误差 A → 电感电压修正 V','current_hz'],['电压误差 V → 电感电流目标 A','voltage_hz'],['输出电流误差 A → 电感电流限制 A','cc_hz']];
  text('loop-units',`${names[loop][0]} · 计算频率 ${c?(c[names[loop][1]]?c[names[loop][1]]+' Hz':'尚未调度'):'待上报'}`);
  const hz=c?c.trace_base_hz/Number($('decimation').value):0;
  text('capture-rate',hz?`有效采样率 ${hz.toLocaleString()} S/s · ${Number($('points').value)} 点，覆盖约 ${(Number($('points').value)/hz*1000).toFixed(2)} ms。串口上传速率不改变采样时间轴。`:'采样率由设备能力决定');
}
function invalidatePID(){pidKey='';for(const [key] of pidFields)$(`pid-${key}`).value='';text('pid-status','尚未读取此通道、此环路的参数');updateLoopUnits();renderControls();}
function showRestore(r){
  if(!r||!r.state){text('restore-status','');return;}
  const gate={pending:'栅极映射：等待中',already:'栅极映射：已确认',restored:`栅极映射：已按上次实测自动确认（${gateMaps[r.gate_map]||r.gate_map}）`,manual:'栅极映射：需要手动实测确认',failed:'栅极映射：自动确认失败'}[r.gate]||'栅极映射：—';
  const zero={pending:'电流零点：等待中',already:'电流零点：已应用',remeasured:'电流零点：已自动重新测量',unsafe:'电流零点：未自动测量（需停机、切正常模式且无电流）',failed:'电流零点：自动测量失败'}[r.zero]||'电流零点：—';
  if(r.state==='running'){text('restore-status',`自动恢复中…（设备 UID ${r.uid||'未知'}）`);return;}
  if(r.state==='error'){text('restore-status',`自动恢复未完成：${r.message}。请手动确认栅极映射并校零。`);return;}
  const v=r.zero_values||{};
  const z=v.iout_uv!==undefined?`（IOUT ${(v.iout_uv/1e6).toFixed(6)} V，IL ${(v.il_uv/1e6).toFixed(6)} V）`:'';
  text('restore-status',`自动恢复完成：${gate} · ${zero}${zero.startsWith('电流零点：已')?z:''}${r.known?'':' · 本设备首次出现，已记住本次结果'}`);
}
function resetSession(){text('zero-status','尚未读取本次设备的零点；MCU 复位后 PowerDesk 会自动重新测量。');text('restore-status','');text('capture-stats','尚无本次连接的快照统计；不会自动校准。');cursor=0;history=[];lastTrace=null;initialTargets.clear();invalidatePID();text('capture-badge','未采集');text('capture-status','尚无快照。停止输出时也可采集，用于检查采样零点。');}
function showZero(z){
  const names=['未进行校准','等待稳定','正在采样','采集完成并已应用','未应用本次结果'];
  const reasons=['','停机、供电、残压或采样条件不满足','采样越界或零点超出允许窗口','波动或分段均值变化过大','采样超时','已被停止或故障取消'];
  const hint=z.state===0?'（这是查询结果，未发生校准；要真正采集请点“开始校准输出电流”）':z.state===3?'（已应用到 MCU；MCU 复位后 PowerDesk 会自动重新测量）':'';
  text('zero-status',`${names[z.state]}${z.reason?'：'+reasons[z.reason]:''}。输出电流零点 ${(z.iout_uv/1e6).toFixed(6)} V（${z.applied&1?'已校准':'标称'}），电感电流零点 ${(z.il_uv/1e6).toFixed(6)} V（${z.applied&2?'已校准':'标称'}）。本次 ${z.samples}/4096 点，采样峰峰 ${(z.pp_uv/1000).toFixed(2)} mV，分段均值跨度 ${(z.span_uv/1000).toFixed(2)} mV。${hint}`);
}
async function zeroAction(value){
  const session=state.session_id;
  try {
    let z=await action('zero',1,{action:value});
    const deadline=Date.now()+10000;
    while(true){
      if(!state.connected||state.session_id!==session) throw new Error('连接已改变，请重新读取零点');
      showZero(z);
      if(z.state!==1&&z.state!==2) break;
      if(Date.now()>deadline) throw new Error('校零结果等待超时，请读取零点确认；不会自动重试校准');
      await new Promise(resolve=>setTimeout(resolve,500));
      z=await action('zero',1,{action:0});
    }
  } catch(e) { text('zero-status',`未确认当前校零结果：${e.message}。重新连接后请读取零点。`); throw e; }
}
for(const [id,value] of [['zero-read',0],['zero-iout',1],['zero-il',2],['zero-reset-iout',3],['zero-reset-il',4]])
  $(id).onclick=()=>work(()=>zeroAction(value));
const gateNames=['PA8 / CHA1','PA9 / CHA2','PA10 / CHB1','PA11 / CHB2'];
const gateMaps=['设计意图：PA8→输入高侧、PA10→输出高侧','输入桥反相：PA8→输入低侧','输出桥反相：PA10→输出低侧','两桥都反相：PA8→输入低侧、PA10→输出低侧'];
function showGate(z){
  const route=z.active_index?`正在驱动 ${gateNames[z.active_index-1]}，剩余约 ${z.remaining_ms} ms`:'当前没有栅极测试在运行';
  text('gate-status',`${z.confirmed?'映射已确认：'+gateMaps[z.mapping]:'映射未确认，功率级被锁'} · ${route}`);
}
for(const [id,index] of [['gate-1',1],['gate-2',2],['gate-3',3],['gate-4',4]])
  $(id).onclick=()=>work(async()=>{const z=await action('gate_test',1,{action:1,index,duration_ms:600});showGate(z);toast(`正在驱动 ${gateNames[index-1]}；用示波器记录哪颗 MOS 栅极在动`);});
$('gate-confirm').onclick=()=>work(async()=>{const z=await action('gate_test',1,{action:2,mapping:Number($('gate-map').value)});showGate(z);toast('栅极映射已确认，功率级启动前置条件之一已满足');});
$('gate-clear').onclick=()=>work(async()=>{const z=await action('gate_test',1,{action:3});showGate(z);toast('已清除栅极映射确认，功率级重新上锁');});
async function refreshGate(){if(!state.connected)return;try{showGate(await action('gate_test',1,{action:0}));}catch(e){text('gate-status',`栅极状态读取失败：${e.message}`);}}
async function refreshPorts(){try{const ports=await api('/api/ports');$('port').replaceChildren();if(!ports.length){const o=new Option('未发现串口，请插入 USB','');$('port').add(o);}else for(const p of ports)$('port').add(new Option(`${p.port} · ${p.description}`,p.port));renderControls();}catch(e){toast(e.message,true);}}
$('refresh').onclick=refreshPorts;$('port').onchange=renderControls;
const AUTO_KEY='powerdesk.auto_restore';
if(localStorage.getItem(AUTO_KEY)==='0') $('auto-restore').checked=false;
$('auto-restore').onchange=()=>localStorage.setItem(AUTO_KEY,$('auto-restore').checked?'1':'0');
$('connect').onclick=()=>work(async()=>{if(state.opened){const r=await api('/api/disconnect',{});toast(r.result.stop_ack?'已断开；停机命令已确认':'已断开；停机未确认，实际输出状态未知',!r.result.stop_ack);}else{resetSession();await api('/api/connect',{port:$('port').value,demo:false,auto_restore:$('auto-restore').checked});toast($('auto-restore').checked?'串口握手成功；正在自动恢复栅极映射与电流零点':'串口握手成功；自动恢复已关闭');}});
$('demo').onclick=()=>work(async()=>{resetSession();await api('/api/connect',{demo:true,auto_restore:$('auto-restore').checked});toast('演示模式已连接，数据为合成示例');});
async function stop(ch=0){try{await action(ch?'output':'stop_all',ch,ch?{enabled:0}:{});toast(ch?`CH${ch} 停止命令已确认`:'全部停止命令已确认，等待状态回报');}catch(e){toast(`停机未确认：${e.message}`,true);}}
$('stop-all').onclick=()=>stop();
$('debug-stop').onclick=()=>stop(Number($('debug-channel').value));
document.addEventListener('keydown',e=>{if(e.key==='Escape'&&state.opened){e.preventDefault();stop();}});
$('debug-channel').onchange=()=>{invalidatePID();render();};$('loop').onchange=invalidatePID;
$('mode').onchange=renderControls;$('decimation').onchange=updateLoopUnits;$('points').onchange=updateLoopUnits;
$('read-pid').onclick=()=>work(async()=>{const ch=Number($('debug-channel').value),loop=Number($('loop').value);const p=await action('get_pid',ch,{loop});if(ch===Number($('debug-channel').value)&&loop===Number($('loop').value)){for(const [key] of pidFields)$(`pid-${key}`).value=Number(p[key].toPrecision(7));pidKey=`${ch}:${loop}`;text('pid-status',`已读取 CH${ch} 参数 · ${new Date().toLocaleTimeString()}`);}});
$('write-pid').onclick=()=>work(async()=>{const ch=Number($('debug-channel').value),loop=Number($('loop').value),values={loop};for(const [key] of pidFields)values[key]=number(`pid-${key}`);text('pid-status','正在写入并回读…');try{const p=await action('set_pid',ch,values);if(`${ch}:${loop}`===`${$('debug-channel').value}:${$('loop').value}`){for(const [key] of pidFields)$(`pid-${key}`).value=Number(p[key].toPrecision(7));text('pid-status',`CH${ch} 写入并回读一致 · 仅 RAM 生效`);}toast('PID 参数已写入并回读验证');}catch(e){pidKey='';text('pid-status','写入或回读失败，参数状态未验证；请重新读取');throw e;}});
$('apply-mode').onclick=()=>work(async()=>{await action('mode',Number($('debug-channel').value),{mode:Number($('mode').value)});toast('模式切换已确认；输出保持关闭');});
$('apply-debug').onclick=()=>work(async()=>{const d=await action('debug_ref',Number($('debug-channel').value),{ilref_ma:milli('il-ref'),duration_ms:number('debug-duration')});toast(`调试目标 ${d.ilref_ma/1000} A / ${d.duration_ms} ms 已确认，尚未启动`);});
$('debug-start').onclick=()=>work(async()=>{await action('output',Number($('debug-channel').value),{enabled:1});toast('启动命令已确认，等待设备状态');});
$('demo-fault').onclick=()=>work(async()=>{await action('demo_fault',Number($('debug-channel').value));toast('演示过流故障已注入');});
$('capture').onclick=async()=>{if(capturing)return;capturing=true;renderControls();text('capture-status','正在采集并下载…');try{lastTrace=(await api('/api/capture',{channel:Number($('debug-channel').value),points:Number($('points').value),decimation:Number($('decimation').value)})).result;text('capture-badge',`${lastTrace.sample_hz.toLocaleString()} S/s`);text('capture-status',`${lastTrace.demo?'演示合成数据 · ':''}CH${lastTrace.channel} · #${lastTrace.capture_id} · ${lastTrace.points} 点。所有点已接收。`);const vals=lastTrace.samples.map(p=>p.il_ma/1000),mean=vals.reduce((a,b)=>a+b,0)/vals.length,pp=Math.max(...vals)-Math.min(...vals),sd=Math.sqrt(vals.reduce((a,b)=>a+(b-mean)**2,0)/vals.length);text('capture-stats',`${lastTrace.demo?'演示数据 · ':''}CH${lastTrace.channel} 电流均值 ${mean.toFixed(4)} A · 峰峰值 ${pp.toFixed(4)} A · 标准差 ${sd.toFixed(4)} A。只有确认无电流时，均值才可用于零点评估；抽取会漏掉高频成分。`);drawCapture();}catch(e){text('capture-status',`采集失败：${e.message}。若图中有曲线，属于上次快照。`);toast(e.message,true);}finally{capturing=false;renderControls();}};
async function download(path,name){try{const response=await fetch(path,{headers:{'X-PowerDesk-Token':token}});if(!response.ok){const d=await response.json();throw new Error(d.error);}const url=URL.createObjectURL(await response.blob()),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(e){toast(e.message,true);}}
$('export-logs').onclick=()=>download('/api/logs.csv','PowerDesk_诊断日志.csv');
$('export').onclick=()=>download('/api/export','PowerDesk_测量记录.csv');$('export-trace').onclick=()=>download('/api/trace.csv','PowerDesk_快速波形.csv');
$('quit').onclick=async()=>{try{const result=await api('/api/quit',{});quitting=true;state.connected=false;state.opened=false;render();text('banner',`本机服务已退出，可以关闭此页面。${result.result.stop_ack?'停机命令已确认。':'停机未确认或未连接设备。'}`);}catch(e){toast(e.message,true);}};
document.querySelectorAll('.nav').forEach(button=>button.onclick=()=>{document.querySelectorAll('.nav,.page').forEach(el=>el.classList.remove('active'));button.classList.add('active');$(button.dataset.page).classList.add('active');const titles={monitor:['电源监测','连接、观察，然后控制。'],debug:['PID 调试','从测量出发，逐环验证。'],protocol:['通信与记录','每条命令，都有据可查。']};const [title,sub]=titles[button.dataset.page];$('page-title').replaceChildren(document.createTextNode(title));const span=document.createElement('span');span.textContent=sub;$('page-title').append(span);drawTrend();drawCapture();});
$('trend-channel').onchange=drawTrend;

function draw(canvas, rows, series, {empty='等待数据',timeLabel='时间 / s',fixedWindow=null,sharedAxis=false}={}){
  const box=canvas.getBoundingClientRect();if(!box.width||!box.height)return;
  const ratio=window.devicePixelRatio||1,w=box.width,h=box.height;canvas.width=w*ratio;canvas.height=h*ratio;
  const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);ctx.clearRect(0,0,w,h);
  const left=48,right=w-48,top=24,bottom=h-29,pw=right-left,ph=bottom-top;
  ctx.font='10px Segoe UI';ctx.fillStyle='#8a9b8d';ctx.textAlign='left';ctx.fillText(series[0].unit,left,13);ctx.textAlign='right';ctx.fillText(series.length>1?series[1].unit:'',right,13);
  const ranges=series.map(s=>{const vals=rows.map(s.value).filter(v=>v!==null&&Number.isFinite(v));let lo=Math.min(0,...vals),hi=Math.max(.1,...vals);const margin=(hi-lo)*.12;return[lo<0?lo-margin:0,hi+margin];});
  if(sharedAxis){const lo=Math.min(...ranges.map(r=>r[0])),hi=Math.max(...ranges.map(r=>r[1]));ranges.forEach(r=>{r[0]=lo;r[1]=hi;});}
  ctx.lineWidth=1;for(let i=0;i<=4;i++){const y=top+i*ph/4;ctx.strokeStyle='#e9eee7';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#8a9b8d';ctx.textAlign='right';ctx.fillText((ranges[0][1]-(ranges[0][1]-ranges[0][0])*i/4).toFixed(2),left-9,y+3);if(series.length>1){ctx.textAlign='left';ctx.fillText((ranges[1][1]-(ranges[1][1]-ranges[1][0])*i/4).toFixed(2),right+8,y+3);}}
  let minX=fixedWindow?fixedWindow[0]:(rows[0]?.x??0),maxX=fixedWindow?fixedWindow[1]:(rows.at(-1)?.x??1);if(maxX===minX)maxX=minX+1;
  for(let i=0;i<=5;i++){const x=left+i*pw/5;ctx.fillStyle='#8a9b8d';ctx.textAlign='center';ctx.fillText((minX+(maxX-minX)*i/5).toFixed(maxX<1?3:1),x,bottom+17);}
  if(!rows.length){ctx.fillStyle='#96a79a';ctx.textAlign='center';ctx.font='12px Microsoft YaHei';ctx.fillText(empty,w/2,top+ph/2);return;}
  ctx.save();ctx.beginPath();ctx.rect(left,top,pw,ph);ctx.clip();
  series.forEach((s,i)=>{ctx.strokeStyle=s.color;ctx.lineWidth=1.8;ctx.setLineDash(s.dash?[5,4]:[]);ctx.beginPath();let drawing=false;for(const row of rows){const val=s.value(row);if(val===null||!Number.isFinite(val)){drawing=false;continue;}const x=left+(row.x-minX)/(maxX-minX)*pw,y=bottom-(val-ranges[i][0])/(ranges[i][1]-ranges[i][0])*ph;if(drawing)ctx.lineTo(x,y);else{ctx.moveTo(x,y);drawing=true;}}ctx.stroke();});ctx.restore();
  ctx.textAlign='center';ctx.fillStyle='#96a79a';ctx.font='9px Segoe UI';ctx.fillText(timeLabel,(left+right)/2,h-1);
}
function drawTrend(){const ch=Number($('trend-channel').value),latest=history.at(-1)?.host_time||Date.now()/1000;const rows=history.filter(p=>p.channel===ch&&p.host_time>=latest-30).map(p=>({...p,x:p.host_time-latest}));draw($('trend'),rows,[{color:'#287758',unit:'V',value:p=>valid(p,2)?p.vout_mv/1000:null},{color:'#ce9249',unit:'A',value:p=>valid(p,3)?p.iout_ma/1000:null}],{empty:'连接设备后，电压与电流趋势将在这里显示',fixedWindow:[-30,0]});}
function drawCapture(){const rows=lastTrace?lastTrace.samples.map((p,i)=>({...p,x:i/lastTrace.sample_hz*1000})):[];draw($('capture-chart'),rows,[{color:'#287758',unit:'A',value:p=>p.il_ma/1000},{color:'#ce9249',unit:'A',dash:true,value:p=>p.ilref_ma/1000}],{empty:'采集后显示电感电流与目标电流',timeLabel:'时间 / ms',sharedAxis:true});}
new ResizeObserver(()=>{drawTrend();drawCapture();}).observe(document.querySelector('main'));
async function poll(){if(quitting)return;try{const next=await api(`/api/state?since=${cursor}`);if(next.session_id!==currentSession){resetSession();currentSession=next.session_id;}state=next;cursor=next.cursor;history.push(...next.samples);const newest=history.at(-1)?.host_time||0;history=history.filter(p=>p.host_time>newest-300).slice(-12000);pollFailures=0;render();}catch(e){pollFailures++;if(pollFailures>=2){state.connected=false;render();$('banner').className='banner error';text('banner','本机服务无法访问，实际输出状态未知。请检查服务；MCU 必须独立执行通信超时停机。');}}setTimeout(poll,200);}
updateLoopUnits();refreshPorts();poll();
