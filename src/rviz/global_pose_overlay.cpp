// Screen-pixel pose readout; independent of the 3D camera and map TF.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreResourceGroupManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <Overlay/OgreTextAreaOverlayElement.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>

namespace co_3dto2d_mapping
{
class GlobalPoseOverlay : public rviz_common::Display
{
  Q_OBJECT

public:
  GlobalPoseOverlay() = default;

  ~GlobalPoseOverlay() override
  {
    subscriptions_ = {};
    auto * manager = Ogre::OverlayManager::getSingletonPtr();
    if (!manager) {return;}
    if (overlay_) {
      overlay_->hide();
      if (panel_) {overlay_->remove2D(panel_);}
    }
    for (auto * line : lines_) {
      if (line) {
        if (panel_) {panel_->removeChild(line->getName());}
        manager->destroyOverlayElement(line);
      }
    }
    if (panel_) {manager->destroyOverlayElement(panel_);}
    if (overlay_) {manager->destroy(overlay_);}
    material_.reset();
    if (Ogre::MaterialManager::getSingletonPtr() && !material_name_.empty()) {
      Ogre::MaterialManager::getSingleton().remove(material_name_);
    }
  }

  void update(float, float) override
  {
    if (!overlay_ || !isEnabled()) {return;}
    std::array<Pose, 2> poses;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      poses = poses_;
    }
    const auto now = std::chrono::steady_clock::now();
    lines_[0]->setCaption("GLOBAL POSE | map     x, y [m]   yaw [deg]");
    for (size_t rid = 0; rid < poses.size(); ++rid) {
      const auto & p = poses[rid];
      const bool stale = p.valid && std::chrono::duration<double>(now-p.received).count() > 2.0;
      std::ostringstream text;
      text << "r" << rid;
      if (!p.valid) {
        text << "   waiting for global pose";
      } else {
        text << std::showpos << std::fixed << std::setprecision(2)
             << "   x " << p.x << "   y " << p.y
             << std::setprecision(1) << "   yaw " << p.yaw_deg;
        if (stale) {text << "   [STALE]";}
      }
      lines_[rid+1]->setCaption(text.str());
      lines_[rid+1]->setColour(!p.valid || stale ? Ogre::ColourValue(.65f, .65f, .65f) :
        rid == 0 ? Ogre::ColourValue(1.f, .51f, .16f) : Ogre::ColourValue(0.f, .86f, .86f));
    }
  }

  void reset() override
  {
    rviz_common::Display::reset();
    std::lock_guard<std::mutex> lock(mutex_);
    poses_ = {};
  }

protected:
  void onInitialize() override
  {
    static std::atomic<unsigned> next_id{0};
    const std::string name = "co_mapping_pose_hud_" + std::to_string(next_id++);
    auto & manager = Ogre::OverlayManager::getSingleton();
    overlay_ = manager.create(name);
    overlay_->setZOrder(600);
    panel_ = static_cast<Ogre::PanelOverlayElement *>(manager.createOverlayElement("Panel", name+"_panel"));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    panel_->setPosition(12.f, 12.f);
    panel_->setDimensions(600.f, 100.f);
    material_name_ = name+"_material";
    material_ = Ogre::MaterialManager::getSingleton().create(material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    auto * pass = material_->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->setDepthCheckEnabled(false);
    pass->setDepthWriteEnabled(false);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    auto * background = pass->createTextureUnitState();
    background->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(.04f, .06f, .09f));
    background->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, .84f);
    panel_->setMaterialName(material_name_);
    overlay_->add2D(panel_);
    for (size_t row = 0; row < lines_.size(); ++row) {
      auto * text = static_cast<Ogre::TextAreaOverlayElement *>(manager.createOverlayElement(
          "TextArea", name+"_text_"+std::to_string(row)));
      lines_[row] = text;
      text->setMetricsMode(Ogre::GMM_PIXELS);
      text->setPosition(12.f, 8.f+28.f*static_cast<float>(row));
      text->setDimensions(580.f, 26.f);
      text->setFontName("Liberation Sans", Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
      text->setCharHeight(row == 0 ? 18.f : 22.f);
      text->setSpaceWidth(7.f);
      text->setColour(Ogre::ColourValue::White);
      panel_->addChild(text);
    }
    const auto abstraction = context_->getRosNodeAbstraction().lock();
    if (!abstraction) {
      setStatus(rviz_common::properties::StatusProperty::Error, "ROS", "RViz ROS node unavailable");
      return;
    }
    auto node = abstraction->get_raw_node();
    for (size_t rid = 0; rid < subscriptions_.size(); ++rid) {
      subscriptions_[rid] = node->create_subscription<nav_msgs::msg::Odometry>(
        "/multi_agent/r"+std::to_string(rid)+"/aligned_odometry", rclcpp::SensorDataQoS().keep_last(1),
        [this, rid](nav_msgs::msg::Odometry::ConstSharedPtr msg) {receive(*msg, rid);});
    }
    setStatus(rviz_common::properties::StatusProperty::Ok, "Pose", "Screen-fixed map pose readout");
  }

  void onEnable() override {if (overlay_) {overlay_->show();}}
  void onDisable() override {if (overlay_) {overlay_->hide();}}

private:
  struct Pose
  {
    bool valid{false};
    double x{0.0}, y{0.0}, yaw_deg{0.0};
    int64_t stamp{0};
    std::chrono::steady_clock::time_point received{};
  };

  void receive(const nav_msgs::msg::Odometry & msg, size_t rid)
  {
    if (msg.header.frame_id != "map") {return;}
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) || norm < 1e-12) {return;}
    const double x=q.x/norm, y=q.y/norm, z=q.z/norm, w=q.w/norm;
    const double yaw = std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z))*180.0/std::acos(-1.0);
    const int64_t stamp = static_cast<int64_t>(msg.header.stamp.sec)*1000000000LL+msg.header.stamp.nanosec;
    std::lock_guard<std::mutex> lock(mutex_);
    if (poses_[rid].valid && stamp < poses_[rid].stamp) {poses_ = {};}
    poses_[rid] = Pose{true, p.x, p.y, yaw, stamp, std::chrono::steady_clock::now()};
  }

  std::mutex mutex_;
  std::array<Pose, 2> poses_{};
  std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, 2> subscriptions_{};
  Ogre::Overlay * overlay_{nullptr};
  Ogre::PanelOverlayElement * panel_{nullptr};
  std::array<Ogre::TextAreaOverlayElement *, 3> lines_{};
  Ogre::MaterialPtr material_;
  std::string material_name_;
};
}  // namespace co_3dto2d_mapping

PLUGINLIB_EXPORT_CLASS(co_3dto2d_mapping::GlobalPoseOverlay, rviz_common::Display)
#include "global_pose_overlay.moc"
