#!/usr/bin/env python
import rospy
from auto_exposure import AutoExposure


def main():
    subTopicL = rospy.get_param(
        "/auto_exposure_top/imageTopicL", None)
    clientL = rospy.get_param(
        "/auto_exposure_top/imageClientL", None)
    subTopicR = rospy.get_param(
        "/auto_exposure_top/imageTopicR", None)
    clientR = rospy.get_param(
        "/auto_exposure_top/imageClientR", None)

    if not subTopicL is None:
        AEL = AutoExposure(subTopicL, clientL)
        AEL.adjust_exposure_time()

    if not subTopicR is None:
        AER = AutoExposure(subTopicR, clientR)
        AER.adjust_exposure_time()


if __name__ == '__main__':
    rospy.init_node('Auto_Exposure_Top')
    main()
