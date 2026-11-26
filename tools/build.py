"""Build the SD-card folders and zips from src/ (src/common is shared byte for byte).
  python tools/build.py  ->  dist/EdgeTX/..., dist/Ethos/..., dist/*.zip"""
import os, re, shutil, zipfile
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC, DIST = os.path.join(ROOT, 'src'), os.path.join(ROOT, 'dist')
ver = re.search(r'local VERSION = "([^"]+)"', open(os.path.join(SRC, 'edgetx', 'ParamEd.lua')).read()).group(1)
ver2 = re.search(r'local VERSION = "([^"]+)"', open(os.path.join(SRC, 'ethos', 'main.lua')).read()).group(1)
assert ver == ver2, 'EdgeTX %s and Ethos %s versions differ' % (ver, ver2)
COMMON = ['pick.lua', 'core.lua', 'mspcore.lua', 'list.lua', 'ilist.lua', 'bflist.lua', 'ui_color.lua',
          'custom.txt', 'custom_inav.txt']
LAYOUT = {
    'EdgeTX': dict([('SCRIPTS/TOOLS/ParamEd.lua', 'edgetx/ParamEd.lua'), ('SCRIPTS/PARAMED/ui_bw.lua', 'edgetx/ui_bw.lua')]
                   + [('SCRIPTS/PARAMED/' + f, 'common/' + f) for f in COMMON]),
    'Ethos': dict([('scripts/parameditor/main.lua', 'ethos/main.lua'), ('scripts/parameditor/icon.png', 'ethos/icon.png')]
                  + [('scripts/parameditor/' + f, 'common/' + f) for f in COMMON]),
}
shutil.rmtree(DIST, ignore_errors=True)
for tgt, files in LAYOUT.items():
    zp = os.path.join(DIST, 'Param_Editor_%s_v%s.zip' % (tgt, ver))
    os.makedirs(DIST, exist_ok=True)
    with zipfile.ZipFile(zp, 'w', zipfile.ZIP_DEFLATED) as z:
        for dst, src in files.items():
            out = os.path.join(DIST, tgt, dst)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            shutil.copyfile(os.path.join(SRC, src), out)
            z.write(out, dst)
        z.write(os.path.join(ROOT, 'INSTALL.txt'), 'INSTALL_Param_Editor.txt')
    print(zp)
