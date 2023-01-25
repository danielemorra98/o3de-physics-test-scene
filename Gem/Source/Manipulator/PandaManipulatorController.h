#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/AzFrameworkModule.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace PandaRobot
{
    //! Component responsible for storing counters of apples gathered by Kraken.
    class ManipulatorController
        : public AZ::Component
        // , public ManipulatorRequestBus ::Handler
        , public AZ::TickBus::Handler
    {
    public:

        AZ_COMPONENT(ManipulatorController, "{1C292F1D-F050-42EB-81C1-F6F83C4929F4}", AZ::Component); // , ManipulatorRequestBus::Handler);

        // AZ::Component interface implementation.
        ManipulatorController() = default;
        ~ManipulatorController() = default;
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        // bool initialized{ false };
        AzToolsFramework::EntityIdList GetHingeJointsChildren(AZ::EntityId parentEntityId);
        bool HasHingeJoint(AZ::EntityId entityId);
        AZStd::string GetJointName(const AZ::Entity* hingeEntity);  // Maybe we need this function
        void PublishMessage();
        void InitializeJointStateMessage();
        void FillMessage();
        double GetJointPosition(const AZ::Entity* hingeEntity); 

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> m_jointstatePublisher;
        AzToolsFramework::EntityIdList m_hingejointsListId;
        sensor_msgs::msg::JointState m_jointstate_msg;
        bool m_initialized{false};

    };
} // namespace AppleKraken
