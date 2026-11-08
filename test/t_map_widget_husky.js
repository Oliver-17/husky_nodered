#!/usr/bin/env node
// =============================================================================
//  t_map_widget_husky.js — 地面車的 status 餵給**無人機那份地圖 widget** 會怎樣
//
//  跑法（不需要 ROS、不需要 Node-RED、不需要瀏覽器）：
//      node ~/ros2_ws/src/husky_nodered/test/t_map_widget_husky.js
//
//  為什麼要有這支：
//      H4 打算讓地面車沿用無人機那份手工做的地圖 widget —— 不重寫，只是把
//      /husky/api/status 餵給同一組「拆成畫面欄位 → 地圖」。前提是那段 JS
//      吃到「少了無人機專屬欄位」的 status 不會炸。
//      而 widget 壞掉時**畫面只是一片空白、終端機不會有任何錯誤**，
//      所以不能用眼睛確認，要在這裡先跑過。
//
//      地面車的 status 少了這些（無人機才有）：
//          altitude_m  track  patrol  target_map_x/y  target_age_s
//          target_lost_s  last_message  armed  landed  mode
//      多了：vehicle: "husky"
// =============================================================================
const fs = require('fs')
const path = require('path')
const vm = require('vm')

const FLOW = path.join(__dirname, '..', '..', 'drone_nodered', 'nodered', 'flows',
  'drone_basic.json')
const MAP_ID = 'd0c0ffee0000020f'      // ui-template「地圖」
const SPLIT_ID = 'd0c0ffee00000209'    // function「拆成畫面欄位」

let bad = 0
function check (ok, what, detail) {
  console.log(`  [${ok ? '通' : '不通'}] ${what}${detail ? '　' + detail : ''}`)
  if (!ok) bad++
}

const flows = JSON.parse(fs.readFileSync(FLOW, 'utf8'))
const mapNode = flows.find(n => n.id === MAP_ID)
const splitNode = flows.find(n => n.id === SPLIT_ID)
if (!mapNode || !splitNode) {
  console.log('✗ 在無人機的流程裡找不到地圖 widget 或「拆成畫面欄位」')
  process.exit(1)
}

// 倉庫的場地方框（和無人機那支測試同一組）
const BOXES = [
  [-20, -14, -18, 14], [18, -14, 20, 14],
  [-20, -14, 20, -12], [-20, 12, 20, 14],
  [-9.2, -7, -7.6, 7], [-3.6, -7, -2, 7],
  [2, -7, 3.6, 7], [7.6, -7, 9.2, 7]
]

// 真的從 husky_api_node 發出來的那種結構
const HUSKY_STATUS = {
  connected: true,
  map_x: -14.0, map_y: 9.0, map_heading_deg: 0.0, speed_mps: 0.0,
  ready: { platform: true, odom: true, nav2: true, bridge: true },
  task: { seq: 3, name: '導航到座標', state: 'running', message: '' },
  nav: { state: 'running', goal: [5.6, -9.0], seq: 3, distance_remaining: 12.3, message: '' },
  watchdog: { tripped: false, reason: '' },
  scene: { boxes: BOXES, home: [-14.0, 9.0] },
  vehicle: 'husky'
}

console.log('① 「拆成畫面欄位」吃地面車的 status')
const _g = new Map()
const FAKE_GLOBAL = { get: k => _g.get(k), set: (k, v) => _g.set(k, v) }

function split (status) {
  // ⚠️ 要餵 global：Node-RED 的 function 節點本來就有這個物件，而「拆成畫面
  //    欄位」2026-11-07 起會用它合併兩台載具的地圖資料。少餵一個參數，
  //    測試就會以 global.set is not a function 當掉 —— 那是測試替身不完整，
  //    不是程式壞掉。
  const fn = new Function('msg', 'node', 'global',
    splitNode.func + '\nreturn null;')
  return fn({ payload: { data: JSON.stringify(status) } },
    { warn () {}, error () {} }, FAKE_GLOBAL)
}
let out = null
try {
  out = split(HUSKY_STATUS)
  check(true, '沒有丟例外')
} catch (e) {
  check(false, '沒有丟例外', e.message)
  console.log('\n✗ 無法繼續 —— 地面車不能直接沿用這份 widget')
  process.exit(1)
}
check(Array.isArray(out) && out.length === 6, '六個出口都有', `得到 ${out && out.length}`)

const mp = out[5] && out[5].payload
check(!!mp, '地圖那個出口有 payload')
check(mp && mp.x === -14.0 && mp.y === 9.0, '車的位置有傳到地圖',
  mp ? JSON.stringify({ x: mp.x, y: mp.y, hdg: mp.hdg }) : '')
check(mp && Array.isArray(mp.boxes) && mp.boxes.length === 8, '貨架與外牆有傳到地圖',
  mp ? `${mp.boxes && mp.boxes.length} 個方框` : '')
check(mp && (mp.tx === undefined || mp.tx === null), '沒有視覺目標時不會畫紅點',
  mp ? `tx=${mp.tx}` : '')
check(mp && (!Array.isArray(mp.loop) || mp.loop.length === 0), '沒有搜尋環線',
  mp ? `loop=${JSON.stringify(mp.loop)}` : '')

// 🔴 已知差異：這個 adapter 是看 ready.px4 / ready.position 判斷就緒的，
//    地面車的 status 用的是 ready.platform / ready.odom，所以就緒燈永遠顯示
//    「未就緒（等 PX4）」。地圖可以沿用，**就緒文字不行** ——
//    make_husky_basic.py 會產生地面車自己的 adapter（地圖邏輯相同、就緒改掉）。
//    這條檢查是為了「哪天有人把 adapter 改通用了，這裡要跟著拿掉」。
check(/未就緒/.test(String(out[0] && out[0].payload)),
  '（已知差異）無人機的 adapter 看不懂地面車的 ready 欄位',
  String(out[0] && out[0].payload))

console.log('\n② 其他五個出口（儀表板的文字與儀表）')
out.forEach((o, i) => {
  const v = o && o.payload
  console.log(`     出口${i}: ${typeof v === 'object' ? JSON.stringify(v).slice(0, 72) : String(v).slice(0, 72)}`)
})

console.log('\n③ 把 widget 本體跑起來，餵地圖 payload')
const m = /<script>([\s\S]*?)<\/script>/.exec(mapNode.format)
const src = m[1].replace(/export\s+default\s*{/, 'module.exports = {')
const sandbox = { module: { exports: {} }, console }
vm.createContext(sandbox)
vm.runInContext(src, sandbox, { filename: 'map-widget' })
const comp = sandbox.module.exports
const self = Object.assign({}, comp.data(), comp.methods)
for (const [k, fn] of Object.entries(comp.computed)) {
  Object.defineProperty(self, k, { get: () => fn.call(self), enumerable: true })
}
self.$nextTick = (f) => { if (f) { f.call(self) } }
self.$refs = {}
try {
  comp.watch.msg.call(self, { payload: mp })
  check(true, 'watch.msg 沒有丟例外')
} catch (e) {
  check(false, 'watch.msg 沒有丟例外', e.message)
}
// ⚠️ droneTf / ugvTf 只有在對應的資料存在時才會被求值（範本上有 v-if 擋著），
//    沒有資料時丟例外是**正常的**。把它們無條件求值會誤報 ——
//    2026-11-07 加上地面車之後這支測試就被自己騙過一次。
const guarded = { droneTf: 'hasPose', ugvTf: 'hasUgv' }
try {
  const touched = Object.keys(comp.computed)
    .filter(k => !(k in guarded) || self[guarded[k]])
    .map(k => [k, self[k]])
  check(true, `${touched.length} 個 computed 都算得出來`,
    touched.map(([k]) => k).join(' '))
} catch (e) {
  check(false, 'computed 算得出來', e.message)
}

console.log('\n④ 同一張地圖同時畫兩台')
const both = { boxes: BOXES, x: -14, y: -9, h: 0, ux: -14, uy: 9, uh: 90 }
try {
  comp.watch.msg.call(self, { payload: both })
  check(self.hasPose === true, '無人機畫得出來', `pose=${JSON.stringify(self.pose)}`)
  check(self.hasUgv === true, '地面車畫得出來', `ugv=${JSON.stringify(self.ugv)}`)
  // 兩台的 transform 不能一樣 —— 一樣就代表其中一個讀錯欄位
  check(self.droneTf !== self.ugvTf, '兩台的位置是分開算的')
  check(/translate\(-14 .*\) rotate\(-90\)/.test(self.ugvTf),
    '地面車的車頭角度有換算（逆時針 → rotate(-h)）', self.ugvTf)
} catch (e) {
  check(false, '同時餵兩台不會丟例外', e.message)
}

console.log()
if (bad) {
  console.log(`✗ 地面車沿用這份地圖 widget 有 ${bad} 個問題`)
  process.exit(1)
}
console.log('✓ 地面車可以直接沿用無人機那份地圖 widget（餵 status 就會自己畫）')
