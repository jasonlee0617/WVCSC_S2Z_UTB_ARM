#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gazebo
{
constexpr double kPi = 3.14159265358979323846;

class SprayVisualPlugin final : public GUIPlugin
{
public:
  SprayVisualPlugin()
  {
    // This plugin only draws render-scene primitives.  Gazebo also treats a
    // GUIPlugin as a QWidget; leaving that widget visible creates an empty
    // opaque panel in the upper-left corner of gzclient.
    this->setAttribute(Qt::WA_DontShowOnScreen, true);
    this->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    this->setFixedSize(0, 0);
    this->hide();
  }

  void Load(sdf::ElementPtr sdf) override
  {
    // Gazebo may show GUIPlugin widgets again while installing the plugin.
    // Keep the widget hidden while retaining its PreRender callback.
    this->hide();
    this->node_ = gazebo_ros::Node::Get(sdf);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    this->wide_sub_ = this->node_->create_subscription<std_msgs::msg::Bool>(
        "/relay/sim/channel_1_active", qos,
        [this](const std_msgs::msg::Bool::SharedPtr message)
        { this->wide_active_.store(message->data); });
    this->nozzle_sub_ = this->node_->create_subscription<std_msgs::msg::Bool>(
        "/relay/sim/channel_2_active", qos,
        [this](const std_msgs::msg::Bool::SharedPtr message)
        { this->nozzle_active_.store(message->data); });
    this->render_connection_ = event::Events::ConnectPreRender(
        std::bind(&SprayVisualPlugin::OnRender, this));
  }

  ~SprayVisualPlugin() override
  {
    this->render_connection_.reset();
  }

private:
  void OnRender()
  {
    // gazebo_ros::Node::Get() is already owned by Gazebo's ROS executor.
    // Adding it to a second executor from the render thread aborts gzclient.
    // The subscriptions update atomics in Gazebo's executor; rendering only
    // reads those states here.
    if (!this->scene_)
      this->scene_ = rendering::get_scene();
    if (!this->scene_)
      return;
    this->EnsureVisuals();
    this->phase_ += 0.035;
    this->DrawWideSpray();
    this->DrawNozzleSpray();
  }

  rendering::VisualPtr FindVisual(const std::initializer_list<const char *> suffixes)
  {
    for (const auto *suffix : suffixes)
    {
      auto visual = this->scene_->GetVisual(
          std::string("wvcsc_utb_alicia") + suffix);
      if (visual)
        return visual;
    }
    return nullptr;
  }

  rendering::VisualPtr MakeGuiVisual(
      const std::string &name, const rendering::VisualPtr &parent)
  {
    auto visual = std::make_shared<rendering::Visual>(name, parent);
    visual->Load();
    visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    this->scene_->AddVisual(visual);
    return visual;
  }

  void EnsureVisuals()
  {
    if (!this->wide_visual_)
    {
      auto base = this->FindVisual({"::base_link", ""});
      if (base)
      {
        this->wide_visual_ = this->MakeGuiVisual("wvcsc_wide_spray_gui", base);
        this->wide_rays_ = this->wide_visual_->CreateDynamicLine(
            rendering::RENDERING_LINE_LIST);
        for (int index = 0; index < 3; ++index)
          this->wide_rings_[index] = this->wide_visual_->CreateDynamicLine(
              rendering::RENDERING_LINE_STRIP);
      }
    }
    if (!this->nozzle_visual_)
    {
      auto nozzle = this->FindVisual({"::spray_nozzle_link", "::tool0"});
      if (nozzle)
      {
        this->nozzle_visual_ = this->MakeGuiVisual("wvcsc_nozzle_spray_gui", nozzle);
        this->nozzle_rays_ = this->nozzle_visual_->CreateDynamicLine(
            rendering::RENDERING_LINE_LIST);
      }
    }
  }

  static void Clear(rendering::DynamicLines *line)
  {
    if (line)
    {
      line->Clear();
      line->Update();
    }
  }

  void DrawWideSpray()
  {
    if (!this->wide_rays_)
      return;
    if (!this->wide_active_.load())
    {
      Clear(this->wide_rays_);
      for (auto *ring : this->wide_rings_)
        Clear(ring);
      return;
    }
    const ignition::math::Color cyan(0.0, 0.88, 1.0, 0.78);
    constexpr double kTop = 1.175;
    this->wide_rays_->Clear();
    for (int index = 0; index < 36; ++index)
    {
      const double angle = 2.0 * kPi * index / 36.0;
      this->wide_rays_->AddPoint(0.10, 0.0, kTop, cyan);
      this->wide_rays_->AddPoint(
          0.10 + 0.95 * std::cos(angle),
          0.95 * std::sin(angle), kTop, cyan);
    }
    this->wide_rays_->Update();
    for (int ring_index = 0; ring_index < 3; ++ring_index)
    {
      auto *ring = this->wide_rings_[ring_index];
      ring->Clear();
      const double radius = 0.85 + 0.15 * std::fmod(
          this->phase_ + static_cast<double>(ring_index) / 3.0, 1.0);
      for (int point = 0; point <= 48; ++point)
      {
        const double angle = 2.0 * kPi * point / 48.0;
        ring->AddPoint(
            0.10 + radius * std::cos(angle), radius * std::sin(angle), kTop,
            cyan);
      }
      ring->Update();
    }
  }

  void DrawNozzleSpray()
  {
    if (!this->nozzle_rays_)
      return;
    if (!this->nozzle_active_.load())
    {
      Clear(this->nozzle_rays_);
      return;
    }
    const ignition::math::Color spray(0.35, 0.92, 1.0, 0.86);
    this->nozzle_rays_->Clear();
    for (int index = 0; index < 7; ++index)
    {
      const double angle = 2.0 * kPi * index / 7.0 + this->phase_;
      this->nozzle_rays_->AddPoint(0.0, 0.0, 0.0, spray);
      this->nozzle_rays_->AddPoint(
          0.030 * std::cos(angle), 0.030 * std::sin(angle), 0.35, spray);
    }
    this->nozzle_rays_->Update();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wide_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nozzle_sub_;
  rendering::ScenePtr scene_;
  rendering::VisualPtr wide_visual_;
  rendering::VisualPtr nozzle_visual_;
  rendering::DynamicLines *wide_rays_{nullptr};
  rendering::DynamicLines *wide_rings_[3]{nullptr, nullptr, nullptr};
  rendering::DynamicLines *nozzle_rays_{nullptr};
  event::ConnectionPtr render_connection_;
  std::atomic_bool wide_active_{false};
  std::atomic_bool nozzle_active_{false};
  double phase_{0.0};
};

GZ_REGISTER_GUI_PLUGIN(SprayVisualPlugin)
}  // namespace gazebo
