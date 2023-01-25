#include "PandaManipulatorController.h"
#include <AzCore/Serialization/EditContext.h>
#include "ROS2/ROS2Bus.h"
#include <PhysX/Joint/PhysXJointRequestsBus.h>
// #include <Source/EditorHingeJointComponent.h>
#include <Source/HingeJointComponent.h>
#include <AzCore/Component/TransformBus.h>

// #include ROS_things


namespace PandaRobot
{

    void ManipulatorController::Init()
    {
        // m_hingejointsListId = GetHingeJointsChildren(GetEntityId());
        // AZ_TracePrintf("ROS2ManipulatorController", "Name entity: %s\n", GetNamedEntityId().ToString().c_str());
    }

    void ManipulatorController::Activate()
    {
        // ManipulatorRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        // initialized = false;
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        // const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), "joint_states");
        m_jointstatePublisher = ros2Node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);        // TODO: add QoS instead of "1"
        // m_hingejointsListId = GetHingeJointsChildren(GetEntityId());
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

    AZStd::vector<AZ::EntityId> ManipulatorController::GetHingeJointsChildren(AZ::EntityId parentEntityId)
    {
        AZStd::vector<AZ::EntityId> hingeJointChildren;
        AZStd::vector<AZ::EntityId> allChildren;
        AZ::TransformBus::EventResult(allChildren, parentEntityId, &AZ::TransformBus::Events::GetAllDescendants);
        AZ_TracePrintf("ROS2ManipulatorController", "Name entity: %s\n", GetNamedEntityId().ToString().c_str());
        AZ_TracePrintf("ROS2ManipulatorController", "Number of child: %d\n", allChildren.size());
        for (auto childId : allChildren)
        {
            AZ_TracePrintf("ROS2ManipulatorController", "Considering child %s\n", childId.ToString().c_str());
            if (HasHingeJoint(childId))
            {
                AZ_TracePrintf("GetHingeJointsChildren", "Child %s has a hingeJoint\n", childId.ToString().c_str());
                hingeJointChildren.push_back(childId);
            }
            // hingeJointChildren.insert(hingeJointChildren.end(), GetHingeJointsChildren(childId).begin(), GetHingeJointsChildren(childId).end());
        }
        return hingeJointChildren;
    }

    void ManipulatorController::InitializeJointStateMessage()
    {
        double nullPosition{0};
        int counter = 1;
        for (auto HingeJointId : m_hingejointsListId)
        {
            AZStd::string panda_joint_name = GetJointName(AzToolsFramework::GetEntityById(HingeJointId));
            m_jointstate_msg.name.push_back(panda_joint_name.c_str());
            AZ_TracePrintf("ROS2ManipulatorController", "Joint Name: %s\n", panda_joint_name.c_str());
            m_jointstate_msg.position.push_back(nullPosition);
            counter++;
        }
    }

    bool ManipulatorController::HasHingeJoint(AZ::EntityId entityId)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "Unknown entity %s", entityId.ToString().c_str());
        return entity->FindComponent<PhysX::HingeJointComponent>() != nullptr;
    }

    void ManipulatorController::PublishMessage()
    {
        std_msgs::msg::Header ros_header;
        ros_header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_jointstate_msg.header = ros_header;
        FillMessage();
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

    AZStd::string ManipulatorController::GetJointName(const AZ::Entity* hingeEntity)
    {
        auto* hingeComponent = hingeEntity->FindComponent<PhysX::HingeJointComponent>();
        return hingeComponent->GetJointName();
    }

    double ManipulatorController::GetJointPosition(const AZ::Entity* hingeEntity)
    {
        double position{0};
        auto* hingeComponent = hingeEntity->FindComponent<PhysX::HingeJointComponent>();
        auto componentId = hingeComponent->GetId();
        const AZ::EntityComponentIdPair id(hingeEntity->GetId(),componentId);
        PhysX::JointRequestBus::EventResult(position, id, &PhysX::JointRequests::GetPosition);
        return position;
    }

    void ManipulatorController::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if(!m_initialized)
        {
            m_hingejointsListId = GetHingeJointsChildren(GetEntityId());
            InitializeJointStateMessage();
            m_initialized = true;
        }

        PublishMessage();
    }
}