# gazebo_continuous_track_ros2_gazebo11

ROS 2 / Gazebo Classic plugins and xacro macros for simulating non-deformable
continuous tracks with explicit track-shoe, friction, and grouser geometry.

This is an unofficial ROS 2 / Gazebo 11 port of the ROS 1 package originally
created by Okada et al. The original ROS 1 repository is:
https://github.com/yoshito-n-students/gazebo_continuous_track

This package installs two Gazebo model plugins:

- `libContinuousTrack.so`: generates moving track-shoe collision/visual elements
  along a defined trajectory.
- `libContinuousTrackSimple.so`: drives existing straight/revolute track segment
  joints from a sprocket joint.

The companion package `gazebo_continuous_track_example_ros2_gazebo11` contains complete launch
and model examples.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gazebo_continuous_track_ros2_gazebo11 --symlink-install
source install/setup.bash
```

The package installs Gazebo environment hooks, so sourcing `install/setup.bash`
adds the plugin library and SDF paths needed by Gazebo.

## Using `libContinuousTrack.so`

Add the plugin to a Gazebo model. The sprocket joint provides the reference
rotational speed, and each trajectory segment maps a joint from position `0` to
`end_position`.

```xml
<gazebo>
  <plugin name="track" filename="libContinuousTrack.so">
    <sprocket>
      <joint>sprocket_axle</joint>
      <pitch_diameter>0.23</pitch_diameter>
    </sprocket>

    <trajectory>
      <segment>
        <joint>track_straight_segment_joint0</joint>
        <end_position>0.7</end_position>
      </segment>
      <segment>
        <joint>track_arc_segment_joint0</joint>
        <end_position>3.141592653589793</end_position>
      </segment>
    </trajectory>

    <pattern>
      <elements_per_round>40</elements_per_round>
      <element>
        <collision name="shoe_collision">
          <geometry>
            <box>
              <size>0.02 0.5 0.02</size>
            </box>
          </geometry>
        </collision>
        <visual name="shoe_visual">
          <geometry>
            <box>
              <size>0.02 0.5 0.02</size>
            </box>
          </geometry>
        </visual>
      </element>
    </pattern>
  </plugin>
</gazebo>
```

### `libContinuousTrack.so` Parameters

| Parameter | Required | Description |
| --- | --- | --- |
| `plugin/@name` | yes | Gazebo plugin instance name. |
| `plugin/@filename` | yes | Use `libContinuousTrack.so`. |
| `sprocket/joint` | yes | Existing revolute or continuous joint used as the reference sprocket. |
| `sprocket/pitch_diameter` | yes | Sprocket pitch diameter in meters. Track speed is derived from this joint. |
| `trajectory/segment` | yes | One or more trajectory sections that form the track path. |
| `trajectory/segment/joint` | yes | Existing prismatic or revolute joint whose child link traces this section. |
| `trajectory/segment/end_position` | yes | Positive end position of that joint. Use meters for prismatic sections and radians for revolute sections. |
| `pattern/elements_per_round` | yes | Number of repeated shoe elements around the whole track. |
| `pattern/element` | yes | One or more SDF collision/visual elements that define a track shoe or grouser pattern. |

## Using `libContinuousTrackSimple.so`

Use the simple plugin when the track is already represented by links/joints and
you only want Gazebo to drive those joints from a sprocket speed.

```xml
<gazebo>
  <plugin name="track_simple" filename="libContinuousTrackSimple.so">
    <sprocket>
      <joint>sprocket_axle</joint>
      <pitch_diameter>0.24</pitch_diameter>
    </sprocket>
    <track>
      <segment>
        <joint>track_straight_segment_joint0</joint>
      </segment>
      <segment>
        <joint>track_arc_segment_joint0</joint>
        <pitch_diameter>0.24</pitch_diameter>
      </segment>
    </track>
  </plugin>
</gazebo>
```

### `libContinuousTrackSimple.so` Parameters

| Parameter | Required | Description |
| --- | --- | --- |
| `plugin/@name` | yes | Gazebo plugin instance name. |
| `plugin/@filename` | yes | Use `libContinuousTrackSimple.so`. |
| `sprocket/joint` | yes | Existing sprocket joint used as the speed source. |
| `sprocket/pitch_diameter` | yes | Sprocket pitch diameter in meters. |
| `track/segment` | yes | One or more joints to drive from the sprocket. |
| `track/segment/joint` | yes | Existing prismatic or revolute joint. |
| `track/segment/pitch_diameter` | required for revolute joints | Pitch diameter for a rotational segment. Omit for prismatic segments. |

## Xacro Macros

The package also provides xacro helpers under `urdf_xacro/`.

### `make_track`

Include:

```xml
<xacro:include filename="$(find gazebo_continuous_track_ros2_gazebo11)/urdf_xacro/macros_track_gazebo.urdf.xacro" />
```

Example:

```xml
<xacro:make_track
  name="track"
  mass="0.4"
  length="0.7"
  radius="0.115"
  width="0.5"
  parent="body"
  sprocket_joint="sprocket_axle"
  pitch_diameter="0.23"
  enable_plugin="true">
  <material>
    <ambient>0.1 0.1 0.3 1</ambient>
    <diffuse>0.2 0.2 0.2 1</diffuse>
  </material>
  <pattern>
    <elements_per_round>40</elements_per_round>
    <element>
      <xacro:make_box_element size="0.02 0.5 0.02" />
    </element>
  </pattern>
</xacro:make_track>
```

Parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `name` | required | Prefix for generated links, joints, and plugin instance. |
| `x y z` | `0` | Track center offset in the parent model frame. |
| `roll pitch yaw` | `0` | Track orientation. |
| `mass` | required | Total generated track mass. |
| `length` | required | Straight section length in meters. |
| `radius` | required | Pulley radius in meters. The macro adds the built-in belt thickness to the visual/collision track radius. |
| `width` | required | Track width in meters. |
| `parent` | required | Parent link for generated trajectory joints. |
| `sprocket_joint` | required | Existing sprocket joint name. |
| `pitch_diameter` | required | Sprocket pitch diameter used by the plugin. |
| `mu` | `0.5` | Contact friction coefficient. |
| `min_depth` | `0.01` | ODE contact min depth. |
| `implicit_spring_damper` | `1` | ODE joint spring/damper stabilization flag. |
| `enable_plugin` | `true` | Set `false` to generate only the geometry and joints without adding `libContinuousTrack.so`. |
| `material` block | required | Material inserted into generated visuals. |
| `pattern` block | required when plugin is enabled | Track-shoe pattern inserted into `libContinuousTrack.so`. |

The current macro generates static grouser collision/visual boxes on the straight
and arc sections. The default continuous track macro uses 18 straight grousers
per straight segment and 20 arc grousers per arc segment.

### `make_lugged_wheel`

Include:

```xml
<xacro:include filename="$(find gazebo_continuous_track_ros2_gazebo11)/urdf_xacro/macros_lugged_wheel_gazebo.urdf.xacro" />
```

Important parameters are the same as `make_track`, except that it creates a
single circular lugged wheel with `radius`, `width`, `parent`,
`sprocket_joint`, and `pitch_diameter`. The macro generates 8 lug collision and
visual boxes by default.

### `make_track_simple` and `make_track_simple_wheels`

Include:

```xml
<xacro:include filename="$(find gazebo_continuous_track_ros2_gazebo11)/urdf_xacro/macros_track_simple_gazebo.urdf.xacro" />
```

Use these helpers with `libContinuousTrackSimple.so` for simpler comparison
models. They accept the same geometry parameters as `make_track`; additionally
`make_track_simple_wheels` requires `count`, the number of wheel segments.

## Notes

- `libContinuousTrack.so` needs existing sprocket and trajectory joints. The
  plugin does not create the sprocket joint for you.
- Prismatic trajectory `end_position` values are distances in meters.
- Revolute trajectory `end_position` values are angles in radians.
- `pattern/elements_per_round` controls shoe spacing around the full trajectory.
- Grousers generated by the xacro macros have both visual and collision geometry.

## License

This ROS 2 / Gazebo 11 port keeps the original MIT License and copyright notice.
See `LICENSE` for the full license text. Keep that notice with redistributed
source or binary copies.

## Citation

Y. Okada, S. Kojima, K. Ohno and S. Tadokoro,
"Real-time Simulation of Non-Deformable Continuous Tracks with Explicit
Consideration of Friction and Grouser Geometry,"
2020 IEEE International Conference on Robotics and Automation (ICRA), 2020.
