import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/liam-bray/roboboat_ws/src/mhsboat_ctrl/install/mhsboat_ctrl'
