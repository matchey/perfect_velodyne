/*

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "perfect_velodyne/convert.h"

namespace perfect_velodyne
{
  class CloudNodelet: public nodelet::Nodelet
  {
  public:

    CloudNodelet() {}
    ~CloudNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<ConvertWithNormal> conv_;
  };

  /** @brief Nodelet initialization. */
  void CloudNodelet::onInit()
  {
    conv_.reset(new ConvertWithNormal(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace perfect_velodyne


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(perfect_velodyne::CloudNodelet, nodelet::Nodelet)
