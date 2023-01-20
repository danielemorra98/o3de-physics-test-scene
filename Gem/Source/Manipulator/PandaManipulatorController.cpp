#include "PandaManipulatorController.h"
#include <AzCore/Serialization/EditContext.h>
#include "ROS2/ROS2Bus.h"
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/EditorHingeJointComponent.h>

// #include ROS_things


namespace PandaRobot
{

    void ManipulatorController::Activate()
    {
        // ManipulatorRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        // initialized = false;
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        // const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), "joint_states");
        m_jointstatePublisher = ros2Node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);        // TODO: add QoS instead of "1"
        m_hingejointsListId = GetHingeJointsChildren(GetEntityId());
        InitializeJointStateMessage();
    }

    void ManipulatorController::Deactivate()
    {
        // ManipulatorRequestBus::Handler::BusDisconnect(GetEntityId());
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ManipulatorController::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorController, AZ::Component>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorController>("ManipulatorController", "[Controller for Panda robotic arm]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_entityX, "m_entityX", "m_entityX")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_entityY, "m_entityY", "m_entityY")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_entityZ, "m_entity_z1", "m_entityZ")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_vectorX, "vx", "vx")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_vectorY, "vy", "vy")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_vectorZ, "vz", "vz")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_effector, "Effector", "Effector")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_restEntity, "Rest entity", "Rest Entity")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::max_errorXZ, "max_errorXZ", "max error XZ to retract nose")
                    // ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::max_errorY, "max_errorY", "max error Y to retract nose");
            }
        }
    }

    AzToolsFramework::EntityIdList ManipulatorController::GetHingeJointsChildren(AZ::EntityId parentEntityId)
    {
        AzToolsFramework::EntityIdList hingeJointChildren;
        AzToolsFramework::EntityIdList allChildren = AzToolsFramework::GetEntityChildOrder(parentEntityId);
        for (auto childId : allChildren)
        {
            AZ_TracePrintf("GetHingeJointsChildren", "Considering child %s\n", childId.ToString().c_str());
            if (HasHingeJoint(childId))
            {
                AZ_TracePrintf("GetHingeJointsChildren", "Child %s has a collider\n", childId.ToString().c_str());
                hingeJointChildren.push_back(childId);
            }
        }
        return hingeJointChildren;
    }

    void ManipulatorController::InitializeJointStateMessage()
    {
        double nullPosition{0};
        const char* nullString = "";
        for ([[maybe_unused]] auto HingeJointId : m_hingejointsListId)
        {
            m_jointstate_msg.name.push_back(nullString);
            m_jointstate_msg.position.push_back(nullPosition);
        }
    }

    bool ManipulatorController::HasHingeJoint(AZ::EntityId entityId)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "Unknown entity %s", entityId.ToString().c_str());
        return entity->FindComponent<PhysX::EditorHingeJointComponent>() != nullptr;
    }

    void ManipulatorController::PublishMessage()
    {
        std_msgs::msg::Header ros_header;
        ros_header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_jointstate_msg.header = ros_header;
        m_jointstatePublisher->publish(m_jointstate_msg);
    }

    void ManipulatorController::FillMessage()
    {
        int i = 0;
        for (auto HingeJointId : m_hingejointsListId)
        {
            m_jointstate_msg.position[i] = GetJointPosition(AzToolsFramework::GetEntityById(HingeJointId));
            i++;
        }
    }

    double ManipulatorController::GetJointPosition(const AZ::Entity* hingeEntity)
    {
        double position{0};
        auto* hingeComponent = hingeEntity->FindComponent<PhysX::EditorHingeJointComponent>();
        auto componentId = hingeComponent->GetId();
        const AZ::EntityComponentIdPair id(hingeEntity->GetId(),componentId);
        PhysX::JointRequestBus::EventResult(position, id, &PhysX::JointRequests::GetPosition);
        return position;
    }

    void ManipulatorController::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        PublishMessage();
    }
}