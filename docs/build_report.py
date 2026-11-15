#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""產生「地面無人載具接入 Low-Code 任務編排工具」報告（HTML → ODT）。"""
import os
import subprocess
import sys

FIG = "/home/oliver/ros2_ws/src/husky_nodered/docs/report"
OUTDIR = "/home/oliver/Downloads"
NAME = "地面車接入 Low-Code 工具 成果報告"

# 兩台同場的實測數字由命令列帶入（跑完才知道）
BOTH = sys.argv[1] if len(sys.argv) > 1 else "（待補）"


def img(f, cap, n, w=5.4):
    """⚠️ 只給 width 的話 LibreOffice 會用預設高度，圖會被壓成一條。
    高度一定要照原圖比例算出來一起給。"""
    from PIL import Image
    iw, ih = Image.open(os.path.join(FIG, f)).size
    h = round(w * ih / iw, 2)
    return (f'<p class="fig"><img src="file://{FIG}/{f}" '
            f'style="width:{w}in;height:{h}in"/></p>'
            f'<p class="cap">圖 {n}　{cap}</p>')


CSS = """
@page { size: 8.5in 11in; margin: 0.85in 0.9in 0.85in 0.9in; }
body { font-family: Calibri, "Noto Sans CJK TC", sans-serif; font-size: 11pt;
       line-height: 1.28; }
/* ⚠️ 主標題不能用 <h1>：LibreOffice 會把 Heading 1 綁上另一個頁面樣式，
   結果是文件最前面多一張空白頁。用一般段落自己做標題就不會。 */
.title { font-size: 18pt; font-weight: bold; text-align: center;
         margin: 0 0 4pt 0; }
h2 { font-size: 14pt; margin: 13pt 0 5pt 0; border-bottom: 1.2pt solid #555;
     padding-bottom: 3pt; }
h3 { font-size: 12.5pt; margin: 10pt 0 3pt 0; }
p  { margin: 4pt 0; text-align: justify; }
.sub { text-align: center; font-size: 11pt; color: #444; margin-bottom: 2pt; }
.fig { text-align: center; margin: 7pt 0 2pt 0; }
.cap { text-align: center; font-size: 9.5pt; color: #333; margin: 0 0 8pt 0; }
table { border-collapse: collapse; width: 100%; font-size: 9.5pt; margin: 5pt 0; }
th { background: #eceff1; border: 0.5pt solid #90a4ae; padding: 3pt 5pt;
     text-align: left; }
td { border: 0.5pt solid #b0bec5; padding: 3pt 5pt; vertical-align: top; }
code { font-family: Courier, monospace; font-size: 10pt; }
ul, ol { margin: 5pt 0 5pt 22pt; }
li { margin: 3pt 0; }
.box { border: 0.8pt solid #90a4ae; background: #f7f9fa; padding: 6pt 9pt;
       margin: 7pt 0; font-size: 10.5pt; }
.key { background: #fff8e1; border-left: 3pt solid #f9a825; padding: 5pt 9pt;
       margin: 7pt 0; font-size: 10.5pt; }
"""

HTML = f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>{NAME}</title><style>{CSS}</style></head><body>

<p class="title">地面無人載具接入 Low-Code 任務編排工具</p>
<p class="sub">Clearpath Husky A200　｜　模擬環境概念驗證</p>
<p class="sub">2026-11-09</p>

<div class="box">
<b>一句話</b>：把既有的 Low-Code 任務編排工具接上第二種載具（地面車），
<b>編排層與橋接層一行程式都沒有改</b>，使用者用同一套積木與同一個儀表板
操作兩種完全不同的載具；本次全部在模擬環境中完成。
</div>

<h2>一、緣起與目標</h2>

<h3>1.1 為什麼需要第二種載具</h3>
<p>這套 Low-Code 工具的設計主張是「換一種載具，不必改編排層」。在只有無人機
的時候，這句話只是設計上的宣稱，沒有辦法證明。要證明它，必須真的接上第二種
載具，而且這台載具和無人機的差異要夠大，結論才有意義。</p>
<p>地面車正好提供了這個落差：可控自由度從四個減為兩個、不能側移、沒有高度、
沒有起飛與降落的概念，而且<b>沒有飛控</b>。如果在這樣的落差下，上層仍然
不需要修改，那麼「載具差異被隔離在下層」這件事就成立。</p>

<h3>1.2 範圍</h3>
<table>
<tr><th style="width:28%">項目</th><th>內容</th></tr>
<tr><td>載具</td><td>Clearpath Husky A200（四輪滑移轉向地面車）</td></tr>
<tr><td>場景</td><td>沿用無人機的倉庫場景，兩台載具同場</td></tr>
<tr><td>執行環境</td><td>Gazebo Harmonic 模擬，<b>不涉及實體車輛</b></td></tr>
<tr><td>做的事</td><td>模型與驅動、導航、服務層、操作積木與儀表板、兩台共存</td></tr>
<tr><td>不做的事</td><td>影像感知（地面車不接視覺追蹤）、實體車輛、實機定位</td></tr>
</table>

<h2>二、載具與系統架構</h2>

<h3>2.1 車輛模型從哪裡來</h3>
<p>模型取自 Gazebo Fuel 由 OpenRobotics 發布的 Clearpath Husky A200（CC0 授權）。
需要先釐清一件事：<b>下載回來的模型只有外形，沒有「會動」也沒有「知道自己在哪」</b>。
原廠模型附的驅動是舊版 Gazebo 的外掛，現行版本載不動。</p>
<table>
<tr><th style="width:38%">下載包本身提供</th><th>本次補上</th></tr>
<tr><td>車體與輪子的外觀模型、碰撞幾何、質量與慣量、四個輪子的接點、
輪距 0.5709 m 與輪半徑 0.17775 m</td>
<td>以現行 Gazebo 內建的差速驅動插件取代原廠舊外掛；另外加上位姿輸出插件
（原廠沒有）；修正滑移轉向造成的轉角誤差</td></tr>
</table>
<p>也就是說，這台車身上<b>沒有任何感測器</b> —— 沒有光達、沒有慣性量測單元、
沒有相機、沒有衛星定位。這一點在第六章會說明它對結論的影響。</p>

<h3>2.2 分層架構：哪幾層沿用、哪幾層換掉</h3>
{img("h01_分層架構.png", "地面車接入後的分層架構（綠＝沿用未改，黃＝只改參數，紅＝本次新寫）", 1)}
<p>重點在顏色：越上層越綠。操作層、編排層、橋接層完全沿用，只有決策層以下
需要為新載具重做。</p>

<h3>2.3 和無人機的關鍵差異</h3>
<table>
<tr><th style="width:24%">項目</th><th style="width:33%">無人機 x500</th><th>地面車 Husky A200</th></tr>
<tr><td>可控自由度</td><td>4（前後、左右、升降、機頭各自獨立）</td>
    <td>2（前進速度、轉向角速度）</td></tr>
<tr><td>能否側移</td><td>可以</td><td><b>不行</b>；車頭就是移動方向</td></tr>
<tr><td>控制核心</td><td>PX4 飛控韌體（四層閉環）</td>
    <td>無飛控；模擬器的驅動插件直接換算輪速</td></tr>
<tr><td>起飛／降落／高度</td><td>有</td><td>沒有這些概念</td></tr>
<tr><td>終點朝向</td><td>不在意（相機朝下）</td><td>有意義（車頭方向）</td></tr>
<tr><td>速度指令的接收者</td><td>需要轉譯節點送進飛控</td><td>平台直接接受速度指令</td></tr>
<tr><td>操作服務數量</td><td>8 個</td><td>3 個</td></tr>
</table>

<h2>三、怎麼接上</h2>

<h3>3.1 速度指令最後送到哪裡</h3>
<p>使用者按下按鈕之後，指令要一路走到輪子。下圖把兩種載具的路徑並排，
可以看出前半段完全相同，分岔點在決策層以下。</p>
{img("h02_控制鏈路.png", "速度指令的傳遞路徑：無人機與地面車對照", 2)}
<p>最大的差別是：無人機那一端的 PX4 是<b>真正的控制器</b>，內含位置、速度、
姿態、角速率四層閉環，會自己抗擾動；地面車這一端的驅動插件只做一次運動學
換算，把速度換成四個輪子的轉速，沒有車身層的回授。地面車負擔得起這樣，
因為它不會掉下來。</p>
<div class="key">
<b>沒有飛控，也就沒有平台級的保護。</b>無人機的指令中斷會觸發飛控本身的
失效保護；地面車的驅動插件<b>沒有指令逾時</b>，送過一次速度之後就算再也
沒人送，它也會沿用最後那一筆一直開下去，直到撞牆。因此本次必須自行補上
一層看門狗：0.5 秒沒有新指令就持續送出零速度。
</div>

<h3>3.2 兩台同場怎麼不互相干擾</h3>
{img("h03_TF與話題.png", "兩台載具同場時的座標樹與命名隔離", 3)}
<p>兩台共用同一張地圖是正確的 —— 它們本來就在同一個倉庫裡。會互相覆蓋的是
座標系名稱與節點名稱，解法是給地面車整組加上前綴，並把它的導航堆疊推進
獨立的命名空間。</p>

<h3>3.3 五個接點</h3>
<table>
<tr><th style="width:22%">接點</th><th>做了什麼</th></tr>
<tr><td>車輛模型</td><td>換上可用的驅動插件、補上位姿輸出、修正滑移轉向</td></tr>
<tr><td>模擬器橋接</td><td>雙向轉換速度指令、位姿、座標轉換與模擬時鐘</td></tr>
<tr><td>導航</td><td>從無人機的導航參數衍生出差速車版本（見第四章）</td></tr>
<tr><td>服務層</td><td>新寫一支節點，對外提供三個動作與一個狀態輸出</td></tr>
<tr><td>操作層</td><td>新增四個積木與一個儀表板分區，沿用既有的地圖元件</td></tr>
</table>

<h2>四、要改什麼</h2>

<h3>4.1 導航參數：差速車和全向載具不一樣</h3>
<p>導航參數檔是從無人機那一份衍生的，地圖、走道幾何、路徑規劃、復原行為
全部照抄，只改下列項目。</p>
<table>
<tr><th style="width:24%">項目</th><th style="width:16%">無人機</th><th style="width:18%">地面車</th><th>為什麼</th></tr>
<tr><td>機體外形</td><td>以圓形近似</td><td>長方形輪廓</td>
    <td>車體 0.99×0.67 m，用外接圓近似會過不了走道或擦到角</td></tr>
<tr><td>側移速度</td><td>1.2 m/s</td><td><b>0</b></td><td>差速車不能側移</td></tr>
<tr><td>前進速度上限</td><td>1.2 m/s</td><td>1.0 m/s</td><td>原廠規格上限</td></tr>
<tr><td>轉向速度上限</td><td>0.8 rad/s</td><td>1.0 rad/s</td><td>只能靠轉向改變行進方向</td></tr>
<tr><td>終點朝向容許值</td><td>不限制</td><td>15°</td><td>車頭方向有意義</td></tr>
<tr><td>評分項目</td><td>懲罰旋轉</td><td>獎勵車頭對準路徑</td>
    <td>懲罰旋轉是為全向載具設計的，用在差速車上會讓車子不願意轉而卡住</td></tr>
<tr><td>起步行為</td><td>直接依路徑前進</td><td>先原地轉向再前進</td>
    <td>差速車若邊轉邊走會畫出大弧，在 4 m 走道內會擦到膨脹區</td></tr>
</table>

<h3>4.2 兩支新寫的程式</h3>
<ol>
<li><b>速度指令看門狗</b>：0.5 秒沒有新指令就持續送零速度。對應無人機那邊
轉譯節點內建的逾時機制 —— 安全保護要用強制機制達成，不能假設上游一定會
記得送出停止指令。</li>
<li><b>地面車服務層</b>：對外提供「導航到座標」「停止」「狀態查詢」三個動作，
並以每秒 5 次的頻率輸出一份狀態。無人機有 8 個服務，地面車只要 3 個 ——
差別不是功能縮水，而是起飛、降落、調整高度、盤旋這些動作對地面車不存在。
「返航」則是編排層把「導航到座標」和場景的起點組合出來的，兩種載具都一樣，
決策層不提供這個動作。</li>
</ol>

<h3>4.3 共用層被第二種載具暴露出來的四個問題</h3>
<p>這一節是接第二種載具最主要的收穫：下列問題只有在「同時存在兩種載具」時
才會出現，單一載具時完全看不到，而且四個都不會報錯。</p>
<table>
<tr><th style="width:30%">問題</th><th>後果</th><th style="width:26%">處理</th></tr>
<tr><td>流程合併工具把套件名稱寫死</td>
    <td>單獨安裝地面車的流程時，會把無人機既有的分頁與積木整批刪除</td>
    <td>改成辨識任意套件</td></tr>
<tr><td>積木名稱重複</td><td>合併時被當成同一個積木</td>
    <td>地面車的積木一律加「車：」前綴</td></tr>
<tr><td>共用積木的定義不一致</td>
    <td>兩份流程裡的同一個共用積木若有細微差異，後載入的會覆蓋前一個而且不報錯</td>
    <td>統一從同一份複製，安裝時檢查</td></tr>
<tr><td>服務層掛在場景的啟動檔</td>
    <td>沒有開模擬時，儀表板整區沒有資料、按鈕沒有反應，三個症狀都不報錯</td>
    <td>改為常駐</td></tr>
</table>

<h3>4.4 滑移轉向：輪子算出來的位置不能用</h3>
<p>Husky 是四輪滑移轉向，轉彎時四個輪子都在地面上側滑，因此用輪子轉速推算
出來的轉向角度會明顯高估。實測原地轉向時，輪子推算出的角度是實際的 1.46 倍，
直線前進則是準的 —— 這是滑移轉向的典型特徵。</p>
<p>處理方式有兩步：一是在驅動插件中改用「等效輪距」補償，使指令下得準、
推算也接近真值；二是定位改用模擬器直接輸出的真實位姿，輪子推算的那一份
保留下來只作為比較基準。</p>

<h2>五、驗收成果</h2>

<h3>5.1 操作介面</h3>
{img("h06_積木.png", "地面車的四個積木，以及與無人機共用的部分（依實際流程檔繪製）", 4)}
<p>使用者串接任務時不需要自己寫等待邏輯：每個積木有「完成」與「失敗」兩個
出口，判定依據是服務層輸出的任務序號與狀態。<b>這份判定協定與無人機完全相同，
因此同一個「等待完成」的邏輯可以同時服務兩種載具</b>，這正是本次要證明的事。</p>

<h3>5.2 導航實測</h3>
{img("h05_導航軌跡.png", "導航實測軌跡（位置取自模擬器真實位姿，2026-11-09）", 5)}
<p>起點與目標之間的直線會穿過四排貨架（圖中虛線），車輛實際由西側走道南下、
再沿南側走道東行繞過貨架。去程與回程的軌跡幾乎重合，重現性良好。</p>
<table>
<tr><th>路段</th><th>結果</th><th>路徑長度</th><th>耗時</th><th>平均速度</th><th>落點誤差</th></tr>
<tr><td>起點 (−14, 9) → 目標 (5.6, −9)</td><td>成功</td><td>32.5 m</td>
    <td>39.4 s</td><td>0.83 m/s</td><td>0.39 m</td></tr>
<tr><td>目標 → 返航 (−14, 9)</td><td>成功</td><td>32.2 m</td>
    <td>39.2 s</td><td>0.83 m/s</td><td>0.38 m</td></tr>
</table>
<p>落點誤差都落在 0.38–0.39 m，原因是終點容許範圍設定為 0.40 m，車輛一進入
範圍即停止。容許值曾試過縮小為 0.25 m，結果車輛在終點反覆微調朝向而無法
結束，因此維持目前設定。</p>
<p class="cap" style="text-align:left">量測說明：位置取自模擬器直接輸出的真實位姿，
並在每段結束時以另一條查詢路徑獨立核對落點，兩者完全一致。<b>不使用車輛
自己由輪速推算的位置當作證據</b>（該數值在滑移轉向下會失真）。</p>

<h3>5.3 服務層與停止行為</h3>
<table>
<tr><th style="width:42%">檢查項目</th><th>結果</th></tr>
<tr><td>四項就緒條件（平台、位置、導航、橋接）</td><td>全部滿足</td></tr>
<tr><td>狀態輸出的位置 vs 真實位姿</td><td>一致</td></tr>
<tr><td>導航途中呼叫「停止」</td><td>停止後 3 秒內位移 0.000 m</td></tr>
<tr><td>被停止時的任務狀態</td><td>回報失敗，並附上「被停止指令取消」的理由</td></tr>
<tr><td>指令被拒絕時</td><td>理由寫進狀態，儀表板「最近回覆」看得到</td></tr>
</table>
<p>停止不是瞬間的：從呼叫到完全靜止約有 1.1 m 的位移（服務往返時間加上減速
距離）。無人機的風險是零速度指令不等於停在原地，地面車的風險則是慣性。</p>

<h3>5.4 兩台載具同場</h3>
{img("h04_儀表板_地面車.png", "兩台載具同時上線的操作畫面（地面車導航進行中）", 6, 4.3)}
<p>兩台載具在同一個倉庫、同一張地圖、同一個儀表板上運作：左欄是無人機，
右欄是地面車，中間的地圖同時畫出兩台的位置與朝向，下方是各自的指令按鈕。
地圖元件<b>完全沿用無人機既有的版本，沒有為地面車另寫一份</b>。</p>
<p>{BOTH}</p>
<div class="key">
啟動順序有一個限制必須遵守：兩套導航堆疊<b>不能同時初始化</b>，否則後啟動
的那一套會停在初始化階段而且不會自行重試。目前的做法是錯開 35 秒，整體
約 95 秒後全部就緒。
</div>

<h2>六、達成的目標，以及尚未證明的事</h2>

<h3>6.1 達成</h3>
<table>
<tr><th style="width:46%">目標</th><th>證據</th></tr>
<tr><td>換一種載具不必修改編排層</td><td>橋接層套件本次改動 <b>0 行</b></td></tr>
<tr><td>同一組積木能服務兩種載具</td><td>完成判定協定一致，並以自動測試綁定檢查</td></tr>
<tr><td>通用介面其實很薄</td><td>無人機 8 個服務、地面車 3 個，上層不需區分</td></tr>
<tr><td>組合型動作屬於編排層</td><td>「返航」兩種載具都由編排層組合，決策層不提供</td></tr>
<tr><td>兩台載具可同場運作</td><td>同一場景、同一儀表板，可同時下達任務</td></tr>
</table>

<h3>6.2 尚未證明的四件事</h3>
<p>這一節是為了讓結論站得住，不要把成果講過頭。</p>
<ol>
<li><b>只證明了同一類運動學之內可行。</b>若換成有最小轉彎半徑、不能原地
轉向的車輛（例如一般轎車），「先原地轉向再前進」的前提就不成立，路徑規劃
也必須更換。分界線是運動學類別，不是「車」或「飛機」。</li>
<li><b>定位是直接取用模擬環境的真實位姿。</b>本次的車輛模型沒有任何感測器，
載具之間最困難的感知與定位問題並未處理（這一點與無人機在模擬中的條件相同）。</li>
<li><b>沒有證明實機可行。</b>要用於實體車輛，必須加裝感測器並改用真正的
定位方法。</li>
<li><b>沒有證明感知功能可以換載具。</b>無人機的影像追蹤未接到地面車，
需求上也沒有要求，因此相關套件的零改動只代表「未使用」。</li>
</ol>

<div class="box">
<b>結論</b>：載具差異被隔離在決策層以下，操作層與編排層不需修改即可驅動
第二種載具。代價是模擬中採用真實位姿定位，且結論僅適用於同一類運動學的
載具；要跨到不同轉向型式或實體車輛，要換的是定位方法與路徑規劃，不是這套工具。
</div>

</body></html>
"""

import tempfile
src = os.path.join(tempfile.gettempdir(), "husky_report_src.html")
with open(src, "w", encoding="utf-8") as f:
    f.write(HTML)
# ⚠️ --infilter 一定要指定「HTML (StarWriter)」。
#    不指定的話 soffice 會把 HTML 載成 Writer/Web 文件，存出來的 .odt 雖然
#    副檔名是 odt，實際上仍是網頁模式：沒有真正的頁面概念，分頁與頁面樣式
#    都不可靠（實際症狀是多一張空白首頁、主標題整個不見）。
subprocess.run(["soffice", "--headless",
                "--infilter=HTML (StarWriter)",
                "--convert-to", "odt:writer8",
                "--outdir", OUTDIR, src], check=True,
               capture_output=True, timeout=180)
dst = os.path.join(OUTDIR, NAME + ".odt")
# soffice 用來源檔名決定輸出檔名，所以要照 src 的檔名去找
produced = os.path.join(
    OUTDIR, os.path.splitext(os.path.basename(src))[0] + ".odt")
os.replace(produced, dst)
HERE = os.path.dirname(os.path.abspath(__file__))
# 轉檔之後一定要跑 finalize_odt：soffice 轉出來的 odt 圖片只是「連結」，
# 而且第一段會被掛上換頁面樣式（空白首頁 + 標題消失）。
subprocess.run([sys.executable, os.path.join(HERE, "finalize_odt.py"),
                dst, FIG], check=True)
print("→", dst)
