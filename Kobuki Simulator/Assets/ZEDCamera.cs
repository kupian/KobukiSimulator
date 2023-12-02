using System.Linq;
using example_interfaces.msg;
using UnityEngine;
using System.Collections.Generic;

namespace ROS2
{
/// <summary>
/// This class simulates the functionality of the ZED camera and YOLO model in a simplified way,
/// publishing a list of visible cones to the topic /cone_positions, in the form
/// (x,y,distance,colour)
/// </summary>
public class ZEDCamera : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.String> cone_pos_pub;
    [SerializeField] Camera cam;
    private Plane[] planes;

    private List<Vector4> conePositions;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        planes = GeometryUtility.CalculateFrustumPlanes(cam);
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ConePositionPublisher");
                cone_pos_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("cone_positions");
            }
            conePositions = GetConePositions();
            string conePositionsString = string.Join("", conePositions);
            std_msgs.msg.String msg = new std_msgs.msg.String();
            msg.Data = conePositionsString;
            cone_pos_pub.Publish(msg);
        }
    }

    List<Vector4> GetConePositions()
    {
        // For now using a 4D vector representing x,y,distance,colour
        List<Vector4> conePositions = new List<Vector4>();
        foreach (GameObject cone in GameObject.FindGameObjectsWithTag("blue_cone"))
        {
            Vector3 screenPos = cam.WorldToScreenPoint(cone.transform.position);
            if (KobukiSeesCone(screenPos))
            {
                float distanceToCone = Vector3.Distance(transform.position, cone.transform.position);
                Vector4 coneVector = new Vector4(screenPos.x/Screen.width, screenPos.y/Screen.height, distanceToCone, 0);
                conePositions.Add(coneVector);
            }
            

        }
        foreach (GameObject cone in GameObject.FindGameObjectsWithTag("yellow_cone"))
        {
            Vector3 screenPos = cam.WorldToScreenPoint(cone.transform.position);
            if (KobukiSeesCone(screenPos))
            {
                float distanceToCone = Vector3.Distance(transform.position, cone.transform.position);
                Vector4 coneVector = new Vector4(screenPos.x/Screen.width, screenPos.y/Screen.height, distanceToCone, 1);
                conePositions.Add(coneVector);
            }
            

        }
        return conePositions;
    }

    bool KobukiSeesCone(Vector3 screenPos)
    {
        // Is the cone inside the camera's fov?
        // This probably needs improving as it requires at least half the cone to be visible
        // Works good enough for now though
        if(screenPos.x >= 0 && screenPos.x <= Screen.width && screenPos.y >= 0 && screenPos.y <= Screen.height && screenPos.z > 0) {
            return true;
        }
        else {
            return false;
        }
        // At some point add raycast checks in case the cone is obscured
    }
}

}
