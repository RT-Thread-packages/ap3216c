from building import *

cwd     = GetCurrentDir()
src     = ['ap3216c.c']
path    = [cwd]

if GetDepend('AP3216C_USING_SENSOR_DEVICE'):
    src += ['sensor_lsc_ap3216c.c']

group = DefineGroup('ap3216c', src, depend = ['PKG_USING_AP3216C'], CPPPATH = path)

Return('group')
