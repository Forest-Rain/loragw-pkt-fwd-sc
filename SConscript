from building import *

src   = []
cwd   = GetCurrentDir()
include_path = [cwd+'/inc']

# sx126x\sx127x
if GetDepend('LORAGW_PKT_FWD_USING_SINGLE_CHANNEL_MODE'):
    src = Split('''
    src/base64.c
    src/jitqueue.c
    src/parson.c
    src/loragw_pkt_fwd_sc.c
    ''')

group = DefineGroup('loragw-pkt-fwd', src, depend = [''], CPPPATH = include_path)

Return('group')