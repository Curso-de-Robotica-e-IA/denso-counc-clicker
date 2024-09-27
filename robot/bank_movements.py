BANK_MOVEMENT = {'home': ([], [], 'Robot moved to home position'),
                 'pre_tap': ([], [], 'Robot moved to pre-tap position'),
                 'tap': ([], [], 'Robot moved to tap position')}


def get_pose(key: str, indx: int):
    """
    Return list of joint positions (if indx == 0) or list of cartesian coordinates and figure (if indx == 1)
    and a message for the test robot
    """
    if key in BANK_MOVEMENT.keys():
        return BANK_MOVEMENT[key][indx], BANK_MOVEMENT[key][2]
    else:
        print('Error trying to select pose')
