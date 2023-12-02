using System;
using ugrdv_kobuki_msgs.msg;
using UnityEngine;

namespace ROS2
{

/// <summary>
/// This class simulates the functionality of the Kobuki. It subscribes to the /ugrdv_kobuki/drive_command
/// topic and sets its linear and angular velocity accordingly.
/// </summary>
public class KobukiController : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<ugrdv_kobuki_msgs.msg.DriveCommand> driveCommandSub;

    private float linearVel = 0.0f;
    private float angularVel = 0.0f;
    
    private Rigidbody rb;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            driveCommandSub = ros2Node.CreateSubscription<ugrdv_kobuki_msgs.msg.DriveCommand>(
              "/ugrdv_kobuki/drive_command", driveCommand => HandleDriveCommand(driveCommand));
        }
        rb.velocity = transform.forward * linearVel;
        transform.Rotate(new Vector3(0,Mathf.Rad2Deg*angularVel, 0) * Time.deltaTime);
    }

    void HandleDriveCommand(DriveCommand driveCommand)
    {
        linearVel = driveCommand.Linearvel;
        angularVel = driveCommand.Angularvel;
        Debug.Log("Linear vel: " + linearVel + ". Angular vel: " + angularVel);
    }
}

}  // namespace ROS2
