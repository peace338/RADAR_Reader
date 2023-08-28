#####################################################################################
#   EQUIP_INFO    radar equip inforamtion                                           #
#####################################################################################

'''
# equip height of the radar from the ground. 
# I assume the LOS of radar is parallel to the ground.
'''
EQUIP_HEIGHT = 0.6 



'''
Interested azim FOV. object out of this region will be removed.
And removed object will not be used as the input of algorithm emulator.
'''
AZIM_FOV = 130 
ELEV_FOV = 80 
MAX_HEIGHT = 3.0 

MAX_Z = MAX_HEIGHT - EQUIP_HEIGHT
MIN_Z = -EQUIP_HEIGHT