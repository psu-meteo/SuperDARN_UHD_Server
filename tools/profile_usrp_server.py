import yappi
from usrp_server import *

yappi.start()

try:
    radar = RadarHardwareManager(45000)
    radar.run()
except:
    pass
print('usrp server quit, dropping to debug console..')
yappi.stop()
pdb.set_trace()

