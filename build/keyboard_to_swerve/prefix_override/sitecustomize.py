import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jinwoolee/Desktop/swervedrive_prj_26/swdr_ws/install/keyboard_to_swerve'
