from pathlib import Path
import xml.etree.ElementTree as ET


BT_PATH = (Path(__file__).resolve().parents[1] / 'config' /
           'navigate_to_pose_ackermann.xml')


def test_follow_path_failure_cannot_retry_the_stale_path_locally():
    root = ET.parse(BT_PATH).getroot()
    assert root.find(".//RecoveryNode[@name='FollowPath']") is None
    assert root.find('.//FollowPath') is not None


def test_moving_obstacle_wait_precedes_backup_recovery():
    root = ET.parse(BT_PATH).getroot()
    round_robin = root.find(".//RoundRobin")
    tags = [child.tag for child in round_robin]
    assert tags.index('Wait') < tags.index('BackUp')
