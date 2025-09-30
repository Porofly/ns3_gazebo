#include <gz/sim/Server.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <iostream>

namespace ns3_gazebo {

class WorldPluginTutorial : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemUpdate {
  public:
  WorldPluginTutorial() {
    printf("Hello World!\n");
  }

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override {
    std::cout << "Configure: Gazebo Harmonic World Plugin\n";
  }

  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override {
    // Update logic here - example of getting model pose
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool {
          if (_name->Data() == "vehicle") {
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
            if (poseComp) {
              gz::math::Pose3d pose = poseComp->Data();
              float x = pose.Pos().X();
              std::cout << "OnUpdate Point.x: " << x << "\n";
            }
          }
          return true;
        });
  }
};

} // namespace ns3_gazebo

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(ns3_gazebo::WorldPluginTutorial,
              gz::sim::System,
              ns3_gazebo::WorldPluginTutorial::ISystemConfigure,
              ns3_gazebo::WorldPluginTutorial::ISystemUpdate)