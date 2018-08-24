
from building import *

cwd     = GetCurrentDir()
src     = Glob('*.c') + Glob('*.cpp')
path    = [cwd]

group = DefineGroup('ap3216', src, depend = ['PKG_USING_AP3216'], CPPPATH = path)

Return('group')
