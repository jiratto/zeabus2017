#!/usr/bin/env python
import rospy
from auto_exposure import AutoExposure


def main():
    EVmin = CONST.EV_BOTTOM_MIN
    EVdefault = CONST.EV_BOTTOM_DEFAULT

    subTopicL = rospy.get_param(
        "/auto_exposure_bottom/imageTopicL", None)
    clientL = rospy.get_param(
        "/auto_exposure_bottom/imageClientL", None)
    subTopicR = rospy.get_param(
        "/auto_exposure_bottom/imageTopicR", None)
    clientR = rospy.get_param(
        "/auto_exposure_bottom/imageClientR", None)

    if not subTopicL is None:
        AEL = AutoExposure(subTopicL, clientL, CONST.IMAGE_BOTTOM_WIDTH,
                           CONST.IMAGE_BOTTOM_HEIGHT, EVdefault, EVmin)
        AEL.adjust_exposure_time()

    if not subBOTTOMicR is None:
        AER = AutoExposure(subBOTTOMicR, clientR, CONST.IMAGE_BOTTOM_WIDTH,
                           CONST.IMAGE_BOTTOM_HEIGHT, EVdefault, EVmin)
        AER.adjust_exposure_time()


if __name__ == '__main__':
    rospy.init_node('Auto_Exposure_Bottom')
    main()
