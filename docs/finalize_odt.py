#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 soffice 從 HTML 轉出來的 ODT 修成可以直接交出去的檔案。

    python3 docs/finalize_odt.py <報告.odt> [圖檔目錄]

做兩件事，兩件都是 HTML→ODT 轉檔一定會踩到的：

1. 把「連結的圖片」真的塞進 odt 裡（Pictures/ 與 manifest）。
   轉出來的 odt 只是用相對路徑指向原始 PNG，檔案一搬走圖就全破。
   ⚠️ 不要改用 UNO 的 .uno:EmbedAllImages：實測它會回報成功、
      但 odt 裡根本沒有 Pictures 資料夾（而且不報錯）。

2. 拿掉第一段的 style:master-page-name 與 style:page-number。
   那是「換頁面樣式」的指示，等於在文件最前面插一個分頁，
   症狀是多一張空白首頁、而且主標題不會被畫出來。
"""
import os
import re
import shutil
import sys
import urllib.parse
import zipfile

MIME = {".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
        ".gif": "image/gif", ".svg": "image/svg+xml"}


def main(odt, figdir=None):
    base = os.path.dirname(os.path.abspath(odt))
    with zipfile.ZipFile(odt) as z:
        items = {i.filename: z.read(i.filename) for i in z.infolist()}
        order = [i.filename for i in z.infolist()]

    content = items["content.xml"].decode("utf-8")
    pics, seen = {}, {}

    def repl(m):
        href = m.group(1)
        if href.startswith("Pictures/"):
            return m.group(0)
        path = urllib.parse.unquote(href)
        if path.startswith("file://"):
            path = path[7:]
        if not os.path.isabs(path):
            path = os.path.normpath(os.path.join(base, path))
        # ⚠️ LibreOffice 寫的相對路徑基準和 odt 所在目錄不一定一致（實測差一層），
        #    所以解不到檔時再用檔名到圖檔目錄找一次。
        if not os.path.exists(path) and figdir:
            alt = os.path.join(figdir, os.path.basename(path))
            if os.path.exists(alt):
                path = alt
        if not os.path.exists(path):
            print("  找不到圖，保持原樣：", path)
            return m.group(0)
        if path not in seen:
            ext = os.path.splitext(path)[1].lower()
            name = f"Pictures/img{len(seen) + 1:03d}{ext}"
            seen[path] = name
            pics[name] = (open(path, "rb").read(), MIME.get(ext, "image/png"))
        return m.group(0).replace(href, seen[path], 1)

    content = re.sub(r'<draw:image xlink:href="([^"]+)"', repl, content)
    # 首段的頁面樣式切換（空白首頁與標題消失的元凶）
    content = re.sub(r' style:master-page-name="[^"]*"', "", content)
    content = re.sub(r' style:page-number="auto"', "", content)
    items["content.xml"] = content.encode("utf-8")

    mf = items["META-INF/manifest.xml"].decode("utf-8")
    entries = "".join(
        f'<manifest:file-entry manifest:full-path="{n}" '
        f'manifest:media-type="{m}"/>' for n, (_, m) in pics.items())
    mf = mf.replace("</manifest:manifest>", entries + "</manifest:manifest>")
    items["META-INF/manifest.xml"] = mf.encode("utf-8")

    tmp = odt + ".tmp"
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        # mimetype 必須是第一個檔而且不能壓縮，否則有些程式認不出這是 ODT
        z.writestr(zipfile.ZipInfo("mimetype"), items["mimetype"],
                   compress_type=zipfile.ZIP_STORED)
        for n in order:
            if n != "mimetype":
                z.writestr(n, items[n])
        for n, (data, _) in pics.items():
            z.writestr(n, data)
    shutil.move(tmp, odt)
    print(f"已內嵌 {len(pics)} 張圖，並清掉首段的頁面樣式切換")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
