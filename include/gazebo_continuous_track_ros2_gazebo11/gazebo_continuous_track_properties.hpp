#ifndef GAZEBO_CONTINUOUS_TRACK_PROPERTIES
#define GAZEBO_CONTINUOUS_TRACK_PROPERTIES

#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo {

class ContinuousTrackProperties {

public:
  ContinuousTrackProperties(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    name = _sdf->HasAttribute("name") ? _sdf->Get<std::string>("name") : "continuous_track";
    sprocket = LoadSprocket(_model, _sdf->GetElement("sprocket"));
    trajectory = LoadTrajectory(_model, _sdf->GetElement("trajectory"));
    pattern = LoadPattern(_model, _sdf->GetElement("pattern"));
  }

  virtual ~ContinuousTrackProperties() {}

  // *****************
  // public properties
  // *****************

  std::string name;
  struct Sprocket {
    physics::JointPtr joint;
    double pitch_diameter;
  };
  Sprocket sprocket;

  struct Trajectory {
    struct Segment {
      physics::JointPtr joint;
      double end_position;
    };
    std::vector< Segment > segments;
  };
  Trajectory trajectory;

  struct Pattern {
    std::size_t elements_per_round;
    struct Element {
      std::vector< sdf::ElementPtr > collision_sdfs;
      std::vector< sdf::ElementPtr > visual_sdfs;
    };
    std::vector< Element > elements;
  };
  Pattern pattern;

private:
  // ******************
  // loading properties
  // ******************

  static Sprocket LoadSprocket(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked by ToPluginSDF(). no need to check if required elements exist.

    Sprocket sprocket;

    // [joint]
    sprocket.joint = _model->GetJoint(_sdf->GetElement("joint")->Get< std::string >());
    GZ_ASSERT(sprocket.joint,
              "Cannot find a joint with the value of [sprocket]::[joint] element in sdf");
    GZ_ASSERT(sprocket.joint->HasType(physics::Joint::HINGE_JOINT),
              "[sprocket]::[joint] must be a rotatinal joint");

    // [pitch_diameter]
    sprocket.pitch_diameter = _sdf->GetElement("pitch_diameter")->Get< double >();
    GZ_ASSERT(sprocket.pitch_diameter > 0.,
              "[sprocket]::[pitch_diameter] must be a positive real number");

    return sprocket;
  }

  static Trajectory LoadTrajectory(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked by ToPluginSDF(). no need to check if required elements exist.

    Trajectory trajectory;

    // [segment] (multiple, +)
    for (sdf::ElementPtr segment_elem = _sdf->GetElement("segment"); segment_elem;
         segment_elem = segment_elem->GetNextElement("segment")) {
      Trajectory::Segment segment;

      // []::[joint]
      segment.joint = _model->GetJoint(segment_elem->GetElement("joint")->Get< std::string >());
      GZ_ASSERT(segment.joint, "Cannot find a joint with the value of "
                               "[trajectory]::[segment]::[joint] element in sdf");
      GZ_ASSERT(segment.joint->HasType(physics::Joint::HINGE_JOINT) ||
                    segment.joint->HasType(physics::Joint::SLIDER_JOINT),
                "[trajectory]::[segment]::[joint] must be a rotatinal or translational joint");

      // []::[end_position]
      segment.end_position = segment_elem->GetElement("end_position")->Get< double >();
      GZ_ASSERT(segment.end_position > 0.,
                "[trajectory]::[segment]::[end_position] must be a positive real number");

      trajectory.segments.push_back(segment);
    }

    return trajectory;
  }

  static Pattern LoadPattern(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked by ToPluginSDF(). no need to check if required elements exist.

    Pattern pattern;

    // [elements_per_round]
    pattern.elements_per_round = _sdf->GetElement("elements_per_round")->Get< std::size_t >();
    GZ_ASSERT(pattern.elements_per_round > 0,
              "[pattern]::[elements_per_round] must be positive intger");

    // [element] (multiple, +)
    for (sdf::ElementPtr element_elem = _sdf->GetElement("element"); element_elem;
         element_elem = element_elem->GetNextElement("element")) {
      Pattern::Element element;

      // []::[collision]
      // Gazebo 11 can crash when dynamically loaded ODE links receive SDF elements
      // created from a fresh schema. Keep the first collision from the original
      // sdformat tree, which preserves the element metadata Gazebo expects.
      if (element_elem->HasElement("collision")) {
        element.collision_sdfs.push_back(element_elem->GetElement("collision")->Clone());
      }

      // []::[visual] (multiple, *)
      if (element_elem->HasElement("visual")) {
        const sdf::ElementPtr base_visual(element_elem->GetElement("visual")->Clone());
        const std::vector< std::string > visual_blocks(ChildBlocks(element_elem, "visual"));
        if (visual_blocks.size() > 1) {
          element.visual_sdfs.push_back(VisualFromBase(base_visual, visual_blocks[1]));
        } else {
          element.visual_sdfs.push_back(base_visual);
        }
      }

      pattern.elements.push_back(element);
    }

    return pattern;
  }

  static std::vector< std::string > ChildBlocks(const sdf::ElementPtr &_element,
                                                const std::string &_tag_name) {
    std::vector< std::string > blocks;
    const std::string xml(_element->ToString(""));
    const std::string open_tag("<" + _tag_name);
    const std::string close_tag("</" + _tag_name + ">");

    std::size_t pos = 0;
    while (true) {
      const std::size_t start = xml.find(open_tag, pos);
      if (start == std::string::npos) {
        break;
      }
      const std::size_t end = xml.find(close_tag, start);
      GZ_ASSERT(end != std::string::npos,
                "Cannot extract ContinuousTrack pattern child block");
      const std::size_t block_end = end + close_tag.size();
      blocks.push_back(xml.substr(start, block_end - start));
      pos = block_end;
    }

    return blocks;
  }

  static sdf::ElementPtr VisualFromBase(const sdf::ElementPtr &_base_visual,
                                        const std::string &_visual_xml) {
    const sdf::ElementPtr visual(_base_visual->Clone());
    if (visual->HasElement("pose")) {
      visual->GetElement("pose")->Set(ParsePose(_visual_xml));
    }
    const ignition::math::Vector3d size(ParseSize(_visual_xml));
    sdf::ElementPtr geometry(visual->GetElement("geometry"));
    if (geometry->HasElement("box")) {
      geometry->GetElement("box")->GetElement("size")->Set(size);
    }
    return visual;
  }

  static ignition::math::Pose3d ParsePose(const std::string &_xml) {
    std::istringstream stream(InnerText(_xml, "pose"));
    double x = 0., y = 0., z = 0., roll = 0., pitch = 0., yaw = 0.;
    stream >> x >> y >> z >> roll >> pitch >> yaw;
    return ignition::math::Pose3d(x, y, z, roll, pitch, yaw);
  }

  static ignition::math::Vector3d ParseSize(const std::string &_xml) {
    std::istringstream stream(InnerText(_xml, "size"));
    double x = 0., y = 0., z = 0.;
    stream >> x >> y >> z;
    return ignition::math::Vector3d(x, y, z);
  }

  static std::string InnerText(const std::string &_xml, const std::string &_tag_name) {
    const std::string open_tag("<" + _tag_name + ">");
    const std::string close_tag("</" + _tag_name + ">");
    const std::size_t start = _xml.find(open_tag);
    const std::size_t end = _xml.find(close_tag, start);
    GZ_ASSERT(start != std::string::npos && end != std::string::npos,
              "Cannot extract ContinuousTrack pattern child value");
    return _xml.substr(start + open_tag.size(), end - start - open_tag.size());
  }

  // **************
  // formatting sdf
  // **************

  // get a sdf element which has been initialized by the plugin format file.
  // the initialied sdf may look empty but have a format information.
  static sdf::ElementPtr LoadPluginFormat() {
    const sdf::ElementPtr fmt(new sdf::Element());
    GZ_ASSERT(sdf::initFile(ament_index_cpp::get_package_share_directory("gazebo_continuous_track_ros2_gazebo11") +
                                "/sdf/continuous_track_plugin.sdf",
                            fmt),
              "Cannot initialize sdf by continuous_track_plugin.sdf");
    return fmt;
  }

  // merge the plugin format sdf and the given sdf.
  // assert if the given sdf does not match the format
  // (ex. no required element, value type mismatch, ...).
  static sdf::ElementPtr ToPluginSDF(const sdf::ElementPtr &_sdf) {
    static const sdf::ElementPtr fmt(LoadPluginFormat());
    const sdf::ElementPtr dst(fmt->Clone());
    GZ_ASSERT(
        sdf::readString("<sdf version='" SDF_VERSION "'>" + _sdf->ToString("") + "</sdf>", dst),
        "The given sdf does not match ContinuousTrack plugin format");
    return dst;
  }
};
} // namespace gazebo

#endif
