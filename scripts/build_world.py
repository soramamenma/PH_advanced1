"""Generate a small original SDF robot and deterministic obstacle world."""

from __future__ import annotations

from pathlib import Path
import math
import xml.etree.ElementTree as ET


def inertia(mass: float, x: float, y: float, z: float) -> str:
    return f"<inertia><ixx>{mass*(y*y+z*z)/12}</ixx><iyy>{mass*(x*x+z*z)/12}</iyy><izz>{mass*(x*x+y*y)/12}</izz></inertia>"


def box(name: str, xyz: tuple[float, float, float], size: tuple[float, float, float], color: str) -> str:
    pose = " ".join(str(v) for v in xyz) + " 0 0 0"
    dimensions = " ".join(str(v) for v in size)
    return f'''<model name="{name}"><static>true</static><pose>{pose}</pose><link name="link">
    <collision name="collision"><geometry><box><size>{dimensions}</size></box></geometry></collision>
    <visual name="visual"><geometry><box><size>{dimensions}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
    </link></model>'''


def build() -> Path:
    wheels = []
    for side, y in [("left", 0.205), ("right", -0.205)]:
        radius, length, mass = 0.08, 0.04, 0.1
        transverse = mass * (3 * radius**2 + length**2) / 12
        axial = mass * radius**2 / 2
        geometry = f"<geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>"
        wheels.append(f'''<link name="{side}_wheel"><pose>0 {y} {radius} 0 0 0</pose>
        <inertial><mass>{mass}</mass><inertia><ixx>{transverse}</ixx><iyy>{axial}</iyy><izz>{transverse}</izz></inertia></inertial>
        <visual name="visual"><pose>0 0 0 {math.pi/2} 0 0</pose>{geometry}<material><ambient>0.08 0.08 0.1 1</ambient><diffuse>0.08 0.08 0.1 1</diffuse></material></visual>
        <collision name="collision"><pose>0 0 0 {math.pi/2} 0 0</pose>{geometry}<surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision></link>
        <joint name="{side}_wheel_joint" type="revolute"><parent>base_link</parent><child>{side}_wheel</child><axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>''')
    casters = []
    for name, x in [("front", 0.20), ("rear", -0.20)]:
        casters.append(f'''<link name="{name}_caster"><pose>{x} 0 0.03 0 0 0</pose>
        <inertial><mass>0.01</mass><inertia><ixx>0.0000036</ixx><iyy>0.0000036</iyy><izz>0.0000036</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.03</radius></sphere></geometry><surface><friction><ode><mu>0.001</mu><mu2>0.001</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><sphere><radius>0.03</radius></sphere></geometry></visual></link>
        <joint name="{name}_caster_joint" type="ball"><parent>base_link</parent><child>{name}_caster</child></joint>''')
    obstacles = [
        box("wall_north", (0, 5, .5), (10, .15, 1), ".25 .3 .35 1"),
        box("wall_south", (0, -5, .5), (10, .15, 1), ".25 .3 .35 1"),
        box("wall_east", (5, 0, .5), (.15, 10, 1), ".25 .3 .35 1"),
        box("wall_west", (-5, 0, .5), (.15, 10, 1), ".25 .3 .35 1"),
        box("obstacle_a", (1.6, 0, .45), (.6, 1.2, .9), ".9 .35 .15 1"),
        box("obstacle_b", (2.8, 1.7, .45), (.7, .7, .9), ".95 .65 .15 1"),
    ]
    xml = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="pai_navigation">
<physics name="physics" type="ignored"><max_step_size>0.005</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
<light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><diffuse>.9 .9 .9 1</diffuse><direction>-.5 -.3 -1</direction></light>
<model name="ground_plane"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><ambient>.8 .85 .88 1</ambient><diffuse>.8 .85 .88 1</diffuse></material></visual></link></model>
{''.join(obstacles)}
<model name="goal"><static>true</static><pose>3.8 0 .01 0 0 0</pose><link name="link"><visual name="visual"><geometry><cylinder><radius>.25</radius><length>.02</length></cylinder></geometry><material><ambient>.1 .8 .3 1</ambient><diffuse>.1 .8 .3 1</diffuse></material></visual></link></model>
<model name="navbot"><pose>0 0 .005 0 0 0</pose>
<link name="base_link"><inertial><pose>0 0 .16 0 0 0</pose><mass>2</mass>{inertia(2,.5,.35,.12)}</inertial>
<visual name="body"><pose>0 0 .16 0 0 0</pose><geometry><box><size>.5 .35 .12</size></box></geometry><material><ambient>.1 .45 .9 1</ambient><diffuse>.1 .45 .9 1</diffuse></material></visual>
<collision name="base_collision"><pose>0 0 .16 0 0 0</pose><geometry><box><size>.5 .35 .12</size></box></geometry></collision>
<sensor name="lidar" type="gpu_lidar"><pose>0 0 .32 0 0 0</pose><topic>/nav/scan</topic><gz_frame_id>navbot/lidar_link</gz_frame_id><update_rate>10</update_rate><always_on>true</always_on><visualize>true</visualize><lidar><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>{-math.pi}</min_angle><max_angle>{math.pi}</max_angle></horizontal></scan><range><min>.05</min><max>8</max><resolution>.01</resolution></range></lidar></sensor>
<sensor name="body_contacts" type="contact"><always_on>true</always_on><update_rate>20</update_rate><topic>/nav/contacts</topic><contact><collision>base_collision</collision><topic>/nav/contacts</topic></contact></sensor>
</link>
{''.join(wheels)}{''.join(casters)}
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"><left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint><wheel_separation>.41</wheel_separation><wheel_radius>.08</wheel_radius><topic>/nav/cmd_vel</topic><odom_topic>/nav/odom</odom_topic><tf_topic>/nav/tf</tf_topic><frame_id>odom</frame_id><child_frame_id>navbot/base_link</child_frame_id><odom_publish_frequency>20</odom_publish_frequency><max_linear_velocity>.2</max_linear_velocity><min_linear_velocity>0</min_linear_velocity><max_angular_velocity>.8</max_angular_velocity><min_angular_velocity>-.8</min_angular_velocity><max_linear_acceleration>.5</max_linear_acceleration><min_linear_acceleration>-.8</min_linear_acceleration></plugin>
</model></world></sdf>'''
    ET.fromstring(xml)
    target = Path(__file__).resolve().parents[1] / "worlds" / "navigation.sdf"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(xml, encoding="utf-8")
    return target


if __name__ == "__main__":
    print(build())
