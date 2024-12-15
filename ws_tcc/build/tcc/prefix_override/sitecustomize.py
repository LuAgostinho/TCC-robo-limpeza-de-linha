import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lucas/Documents/GitHub/TCC-robo-limpeza-de-linha/ws_tcc/install/tcc'
