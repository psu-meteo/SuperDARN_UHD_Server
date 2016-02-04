# phasing matrix utilities
# for calculating phase shifts and other array stuff..

import numpy as np

C = 3e8

X_SPACING = 15.4 # TODO: set this from a config file

# returns a complex number from a phase in radians
def rad_to_rect(rad):
    return np.exp(1j * rad)

# nbeams - total number of beams
# beamnum - beam number to calculate azm
# beam sep - beam separation in degrees
# returns beam azm angle in radians
def calc_beam_azm_rad(nbeams, beamnum, beam_sep):
    # calculate beamforming shift..
    center_beam = (nbeams - 1.0) / 2.

    # calculate beam azimuth, in radians
    bmazm = np.deg2rad(90 + (beamnum - center_beam) * beam_sep)

    return bmazm

# calculates the phase increment between antennas
# to produce a mainlobe steering of bmazm at tfreq 
# tfreq in hz, bmazm in radians
def calc_phase_increment(bmazm, tfreq, x_spacing = X_SPACING):
    # translate to phase increment
 
    wavelength = C / tfreq
    pshift = (2 * np.pi * x_spacing * np.sin(bmazm)) / wavelength
    return pshift 

# calculates a complex number for a phase shift
def calc_beamforming_phase_rect(antenna, pshift):
    beamforming_phase = rad_to_rect(antenna * pshift)
    return beamforming_phase
