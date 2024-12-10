import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mscrobotics2425laptop7/robotdesign - team10/install/object_detect_opencv'
