from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET
from pai_ai_navigation.scenarios import SCENARIOS, get_scenario

ROOT = Path(__file__).resolve().parents[1]


def world_boxes():
    boxes = []
    for model in ET.parse(ROOT/'worlds/navigation.sdf').getroot().findall('./world/model'):
        if not model.attrib.get('name','').startswith('obstacle_'):
            continue
        center = list(map(float,model.findtext('pose').split()))[:3]
        size = list(map(float,model.findtext('./link/collision/geometry/box/size').split()))
        boxes.append(tuple((c-s/2,c+s/2) for c,s in zip(center,size)))
    return boxes


def segment_hits_box(start, end, box):
    low, high = 0.0, 1.0
    for a,b,(minimum,maximum) in zip(start,end,box):
        delta = b-a
        if abs(delta)<1e-12:
            if not minimum<=a<=maximum:return False
            continue
        t1,t2=(minimum-a)/delta,(maximum-a)/delta
        low,high=max(low,min(t1,t2)),min(high,max(t1,t2))
        if low>high:return False
    return True


class ScenarioGeometryTests(unittest.TestCase):
    def test_hidden_goal_marker_is_blocked_by_actual_world_geometry(self):
        gx,gy=get_scenario('behind_obstacle').goal
        # From the initial LiDAR-height viewpoint to the centre and marker rim.
        points=[(gx,gy,.02)]+[(gx+.25*math.cos(i*math.tau/64),gy+.25*math.sin(i*math.tau/64),.02) for i in range(64)]
        for point in points:
            self.assertTrue(any(segment_hits_box((0,0,.325),point,b) for b in world_boxes()),point)

    def test_scenario_goals_have_room_for_the_robot_and_safety_stop_margin(self):
        for scenario in SCENARIOS:
            gx,gy=scenario.goal
            self.assertLess(abs(gx)+.55,4.925)
            self.assertLess(abs(gy)+.55,4.925)
            for box in world_boxes():
                dx=max(box[0][0]-gx,0,gx-box[0][1]);dy=max(box[1][0]-gy,0,gy-box[1][1])
                self.assertGreater(math.hypot(dx,dy),.55)

    def test_unrecognized_scenario_does_not_fall_back_to_another_goal(self):
        with self.assertRaises(ValueError):get_scenario('not-a-scenario')


if __name__=='__main__':unittest.main()