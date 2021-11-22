#ifndef IAUV_CONTROL_MODELPARSER_H
#define IAUV_CONTROL_MODELPARSER_H

#include <iauv_control/eigen_typedefs.h>
#include <Eigen/Core>
#include <urdf_parser/urdf_parser.h>
#include <tinyxml.h>

namespace iauv_control
{

class IAUV
{
  using ThrusterJoints = std::map<std::string, urdf::JointSharedPtr>;

public:

  // return raw xml or urdf
  static std::string getModelXML(const std::string &ns, std::string caller = "rov");
  static urdf::ModelInterfaceSharedPtr getModel(const std::string &ns, std::string caller = "rov");

  IAUV() {}

  // actual parsing of hydrodynamic model
  std::vector<std::string> parseModel(const std::string &ns);

  void compensate(Vector6d &wrench, const Eigen::Matrix3d &R, const Vector6d &vel) const;
  void solveWrench(const Vector6d &wrench, std::vector<double> &thrusts) const;

  uint n_thr;
  double buoyancy;
  MatrixX6d tam_pinv;
  Eigen::VectorXd max_thrusts;
  Vector6d max_wrench,quad_drag, lin_drag;
  Eigen::Matrix<double, 6, 6> Ma, Mi;
  Eigen::Vector3d cog, cob;

private:

  // decompose parser in these functions
  void parseThrusterMap(TiXmlElement* root, const ThrusterJoints &thrusters);
  void parseStatics(urdf::InertialConstSharedPtr inertial);
  void parseHydrodynamics(TiXmlElement* root, const std::string &base_link);

  // read an element inside a tag
  template <typename T>
  static bool readFromSequence(TiXmlElement* root,
                       std::vector<std::string> tag_sequence,
                       T & val)
  {
    if(tag_sequence.size() == 0)
    {
      if(root)
      {
        std::stringstream ss(root->GetText());
        ss >> val;
        return true;
      }
      return false;
    }
    return readFromSequence(root->FirstChildElement(tag_sequence.front().c_str()),
    {tag_sequence.begin()+1, tag_sequence.end()},
             val);
  }
  template <typename T>
  inline static bool readFromTag(TiXmlElement* root,
                       std::string tag,
                       T & val)
  {
    return readFromSequence(root, {tag}, val);
  }

  template <typename T>
  inline static T readFromTag(TiXmlElement* root, std::string tag)
  {
    T value;
    readFromTag(root, tag, value);
    return value;
  }

  template <int rows, int cols>
  void readMatrix(TiXmlElement* root,
                  std::vector<std::string> tag_sequence,
                  Eigen::Matrix<double, rows, cols> & M)
  {
    std::string values;
    readFromSequence(root, tag_sequence, values);
    std::istringstream iss(values);
    for(uint row = 0; row < rows; ++row)
    {
      for(uint col = 0; col < cols; ++col)
      {
        iss >> M(row, col);
      }
    }
  }

};
}

#endif // IAUV_CONTROL_MODELPARSER_H
