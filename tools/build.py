"""Build the SD-card folders and zips from src/ (src/common is shared byte for byte).
  python tools/build.py  ->  dist/EdgeTX/..., dist/Ethos/..., dist/*.zip"""
import os, re, shutil, zipfile
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC, DIST = os.path.join(ROOT, 'src'), os.path.join(ROOT, 'dist')
ver = re.search(r'C.VERSION = "([^"]+)"', open(os.path.join(SRC, 'common', 'core.lua')).read()).group(1)
LAYOUT = {
    'EdgeTX': {'SCRIPTS/TOOLS/APParam.lua': 'edgetx/APParam.lua',
               'SCRIPTS/APPARAM/core.lua': 'common/core.lua', 'SCRIPTS/APPARAM/list.lua': 'common/list.lua',
               'SCRIPTS/APPARAM/ui_color.lua': 'common/ui_color.lua', 'SCRIPTS/APPARAM/ui_bw.lua': 'edgetx/ui_bw.lua',
               'SCRIPTS/APPARAM/custom.txt': 'common/custom.txt'},
    'Ethos': {'scripts/apparam/main.lua': 'ethos/main.lua', 'scripts/apparam/icon.png': 'ethos/icon.png',
              'scripts/apparam/core.lua': 'common/core.lua', 'scripts/apparam/list.lua': 'common/list.lua',
              'scripts/apparam/ui_color.lua': 'common/ui_color.lua', 'scripts/apparam/custom.txt': 'common/custom.txt'},
}
shutil.rmtree(DIST, ignore_errors=True)
for tgt, files in LAYOUT.items():
    zp = os.path.join(DIST, 'AP_Param_Editor_%s_v%s.zip' % (tgt, ver))
    os.makedirs(DIST, exist_ok=True)
    with zipfile.ZipFile(zp, 'w', zipfile.ZIP_DEFLATED) as z:
        for dst, src in files.items():
            out = os.path.join(DIST, tgt, dst)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            shutil.copyfile(os.path.join(SRC, src), out)
            z.write(out, dst)
        z.write(os.path.join(ROOT, 'INSTALL.txt'), 'INSTALL_AP_Param_Editor.txt')
    print(zp)
