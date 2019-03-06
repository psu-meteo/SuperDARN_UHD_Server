

from usrp_server.usrp_server import RadarHardwareManager


def test_default_RHM():
    rhm = RadarHardwareManager(0)
    assert rhm.exit_usrp_server == False
