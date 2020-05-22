using System;
using System.Collections;
using System.Collections.Concurrent;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
//using Unity.Mathematics;
using UnityEngine.Analytics;
using Object = UnityEngine.Object;
using Random = UnityEngine.Random;

/// <summary>
/// These are the inputs that are recorded, and may be constrained (or left as free inputs)
/// </summary>

[Serializable]
public enum TrackerIdentifier
{
    HMD,
    LTouch,
    RTouch
} 

[Serializable]
public enum ConstraintParadigm
{
    HMD,
    LTouch,
    RTouch,
    HMD_LTouch,
    HMD_RTouch,
    LTouch_RTouch,
    HMD_LTouch_RTouch
}

[Serializable]
public struct StaticPoseConstraintTask
{
    public readonly string id; //unique task id
    public readonly string Creator;
    public List<bool> attemptResults;
    public ConstraintParadigm constraintParadigm;
    public Dictionary<TrackerIdentifier, SerializablePose> constraints;
    
    public StaticPoseConstraintTask(ConstraintParadigm toConstrain, Pose inputPose, string creator, string taskId)
    {
        Creator = creator;
        attemptResults = new List<bool>();
        constraintParadigm = toConstrain;
        id = taskId;
        constraints = new Dictionary<TrackerIdentifier, SerializablePose>();
        if (toConstrain == ConstraintParadigm.RTouch)
        {
            constraints.Add(TrackerIdentifier.RTouch, inputPose);
        }
        else
        {
            throw new NotImplementedException();
        }
    }

    
    /// <summary>
    /// constructor for when you input a JSON. call via StaticPoseConstraintTask.Import(...)
    /// </summary>
    /// <param name="jsonObject"></param>
    /// <returns></returns>
    public static StaticPoseConstraintTask Import(JObject jsonObject)
    {
        var myPosition = jsonObject["point_task"]["position"];
        var myRotation = jsonObject["point_task"]["rotation"];
        var myPose = new Pose(GetVector3(myPosition), GetQuaternion(myRotation));
        var myID = jsonObject["id"].Value<string>();
        var creatorString = jsonObject["source_algorithm"].Value<string>();
        var taskStr = new StaticPoseConstraintTask(ConstraintParadigm.RTouch, myPose, creatorString, myID);
        return taskStr;
    }

    private static Quaternion GetQuaternion(JToken myRotation)
    {
        return BTools.ArrayXYZWToQuaternion(BTools.GetValuePerDimension(new List<string>(){"x","y","z","w"}, myRotation));
    }

    private static Vector3 GetVector3(JToken myPosition)
    {
        return BTools.ArrayToVector3(BTools.GetValuePerDimension(new List<string>(){"x","y","z"}, myPosition));
    }

    internal string ToJSON()
    {
        return BTools.ToJsonWithoutLoopback(this);
    }
}

[Serializable]
public struct SerializablePose
{
    public SerializableVector3 position;
    public SerializableQuaternion rotation;

    public SerializablePose(Vector3 positionInput, Quaternion rotationInput)
    {
        position = positionInput;
        rotation = rotationInput;
    }
         
    /// <summary>
    /// Automatic conversion from SerializablePose to Pose
    /// </summary>
    /// <param name="rValue"></param>
    /// <returns></returns>
    public static implicit operator Pose(SerializablePose rValue)
    {
        return new Pose(rValue.position,rValue.rotation);
    }
     
    /// <summary>
    /// Automatic conversion from Pose to SerializablePose
    /// </summary>
    /// <param name="rValue"></param>
    /// <returns></returns>
    public static implicit operator SerializablePose(Pose rValue)
    {
        return new SerializablePose(rValue.position,rValue.rotation);
    }
}


[Serializable]
public struct PoseObservation
{
    public Dictionary<TrackerIdentifier, TrackedObjectKinematics> poseKinematics;
    public string timestamp;
    /// <summary>
    /// a pose observation is the hands and head at a moment in time.
    /// </summary>


    /// <param name="timestampInput"></param>
    /// <param name="input_id"></param>
    internal PoseObservation(string timestampInput, XRSource source)
    {
        poseKinematics = new Dictionary<TrackerIdentifier, TrackedObjectKinematics>();
        switch (source)
        {
            case XRSource.None:
                poseKinematics.Add(TrackerIdentifier.HMD,  new TrackedObjectKinematics(TrackerIdentifier.HMD, Vector3.one,Vector3.one,Vector3.one, Quaternion.identity,Vector3.one, Vector3.one));
                poseKinematics.Add(TrackerIdentifier.LTouch, new TrackedObjectKinematics(TrackerIdentifier.LTouch, Vector3.one,Vector3.one,Vector3.one, Quaternion.identity,Vector3.one, Vector3.one));
                poseKinematics.Add(TrackerIdentifier.RTouch, new TrackedObjectKinematics(TrackerIdentifier.RTouch, Vector3.one,Vector3.one,Vector3.one, Quaternion.identity,Vector3.one, Vector3.one));
                break;
            case XRSource.OVR:
                poseKinematics.Add(TrackerIdentifier.HMD, BTools.GetHeadsetKinematics());
                poseKinematics.Add(TrackerIdentifier.LTouch, BTools.GetOVRControllerKinematics(TrackerIdentifier.LTouch, OVRInput.Controller.LTouch));
                poseKinematics.Add(TrackerIdentifier.RTouch, BTools.GetOVRControllerKinematics(TrackerIdentifier.RTouch, OVRInput.Controller.RTouch));
                break;
            case XRSource.SteamVR:
                Debug.Log("ruh rhoes");
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(source), source, null);
        }
        timestamp = timestampInput;
        
    }
    internal string ToJSON()
    {
        return BTools.ToJsonWithoutLoopback(this);
    }
}

internal enum XRSource
{
    None,
    OVR,
    SteamVR
}

public struct PoseObservationList
{
    public List<PoseObservation> poseObservations;
    public string sessionId;

    public PoseObservationList(string session_input)
    {
        poseObservations = new List<PoseObservation>();
        sessionId = session_input;
    }
    public string ToJson()
    {
        return BTools.ToJsonWithoutLoopback(this);
    }
}
[System.Serializable]
public struct SessionPoseObservations
{
    public ConcurrentQueue<PoseObservation> poseObservations;
    public string id;
    public SessionPoseObservations(string sessionId)
    {
        id = sessionId;
        poseObservations = new ConcurrentQueue<PoseObservation>();
    }
    internal string ToJSON()
    {
        return BTools.ToJsonWithoutLoopback(this);
    }
}

[System.Serializable]
public struct TrackedObjectKinematics
{
    //TODO create a kinematics that, when senses they're not tracking, attaches these values to the rigidbodies
    // on the controller models so it can be manipulated in the Editor without VR connected.
    public string trackedObject;
    public SerializableVector3 pos; //meters
    public SerializableVector3 vel; //meters per second
    public SerializableVector3 acc; //meters per second squared
    
    public SerializableQuaternion rot; //orientation
    public SerializableVector3 rotdot; //radians per second, per dimension
    public SerializableVector3 rotdotdot; // radians per second per second, per dimension

    public TrackedObjectKinematics(TrackerIdentifier trackedObjectName, Vector3 _pos, Vector3 _vel, Vector3 _acc, Quaternion _rot,
        Vector3 _rotdot, Vector3 _rotdotdot)
    {
        trackedObject = trackedObjectName.ToString();
     
        pos = _pos;
        vel = _vel;
        acc = _acc;
        
        rot = _rot;
        rotdot = _rotdot;
        rotdotdot = _rotdotdot;
    }
}



public static class BTools
{
    
    internal static double[] Vector3ToDouble(Vector3 input)
    {
        return new[] {(double) input.x, (double) input.y, (double) input.z};
    }
    /// <summary>
    /// A GUID is a 128-bit integer (16 bytes) that can be used across all computers and networks wherever a
    /// unique identifier is required. Such an identifier has a very low probability of being duplicated.
    /// https://stackoverflow.com/questions/11313205/generate-a-unique-id
    /// </summary>
    /// <returns>string guid like F9168C5E-CEB2-4faa-B6BF-329BF39FA1E4</returns>
    public static string GenerateID()
    {
        return Guid.NewGuid().ToString();
    }
    /// <summary>
    /// the order is XYZW for 0123
    /// </summary>
    /// <param name="input"></param>
    /// <returns></returns>
    internal static Quaternion ArrayXYZWToQuaternion(float[] input)
    {
        return new Quaternion(input[0],input[1],input[2],input[3]);
    }

    public static Vector3 ArrayToVector3(float[] input)
    {
        return new Vector3(input[0],input[1],input[2]);
    }

    internal static float[] GetValuePerDimension(List<string> dimensions, JToken myP)
    {
        return dimensions.Select(x => myP[x].Value<float>()).ToArray();
    }

    public static string GetComputerId()
    {
        return SystemInfo.deviceUniqueIdentifier;
    }
    /// <summary>
    /// unix time in seconds with digits as float.
    /// </summary>
    /// <returns></returns>
    public static double UnixSeconds()
    {
        return DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
    }

    public static string SessionId()
    {
        return AnalyticsSessionInfo.sessionId.ToString();
    }

    public static string UnixString()
    {
        return UnixSeconds().ToString();
    }
    public static string ToJsonWithoutLoopback(object y)
    {
        var stringOut = JsonConvert.SerializeObject(y,
            new JsonSerializerSettings()
            {
                ReferenceLoopHandling = ReferenceLoopHandling.Ignore
            });
        return stringOut;
    }
    public static IEnumerator Haptics(float frequency, float amplitude, float duration, bool rightHand, bool leftHand)
    {
        if(rightHand) OVRInput.SetControllerVibration(frequency, amplitude, OVRInput.Controller.RTouch);
        if(leftHand) OVRInput.SetControllerVibration(frequency, amplitude, OVRInput.Controller.LTouch);

        yield return new WaitForSeconds(duration);

        if (rightHand) OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.RTouch);
        if (leftHand) OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.LTouch);
    }
    /// <summary>
    /// Gets the controller physics up to accelerations for each the pos and rot.
    /// </summary>
    /// <param name="trackedObject">an enum element</param>
    /// <param name="controller">an OVR controller to extract from</param>
    /// <returns></returns>
    public static TrackedObjectKinematics GetOVRControllerKinematics(TrackerIdentifier trackedObject, OVRInput.Controller controller)
    {
        var pos = OVRInput.GetLocalControllerPosition(controller);
        var vel = OVRInput.GetLocalControllerVelocity(controller);
        var acc = OVRInput.GetLocalControllerAcceleration(controller);
        var rot = OVRInput.GetLocalControllerRotation(controller);
        var rotdot = OVRInput.GetLocalControllerAngularVelocity(controller);
        var rotdotdot = OVRInput.GetLocalControllerAngularAcceleration(controller);
        return new TrackedObjectKinematics(trackedObject, pos, vel, acc, rot, rotdot, rotdotdot);
    }
    /// <summary>
    /// Forms a structure that describes the physics of the headset. pos/rot and up to accelerations of each
    /// </summary>
    /// <returns></returns>
    public static TrackedObjectKinematics GetHeadsetKinematics()
    {
        var myPose = GetOVRHeadsetPose();

        return new TrackedObjectKinematics(TrackerIdentifier.HMD,
            myPose.position,
            OVRManager.display.velocity,
            OVRManager.display.acceleration,
            myPose.orientation,
            OVRManager.display.angularVelocity,
            OVRManager.display.angularAcceleration);
    }
    private static OVRPose GetOVRHeadsetPose()
    {
        var relativeOrigin = GetOVRRelativeOrigin();
        OVRPose myPose = OVRPlugin.GetTrackingTransformRelativePose((OVRPlugin.TrackingOrigin) relativeOrigin).ToOVRPose();
        return myPose;
    }

    private static OVRManager.TrackingOrigin GetOVRRelativeOrigin()
    {
        var previousTrackingOrigin = OVRManager.instance.trackingOriginType;
        OVRManager.TrackingOrigin relativeOrigin = (previousTrackingOrigin != OVRManager.TrackingOrigin.Stage)
            ? OVRManager.TrackingOrigin.Stage
            : OVRManager.TrackingOrigin.EyeLevel;
        return relativeOrigin;
    }

    public static double RandomBoolInDouble()
    {
        return Math.Round(Random.value, 0);
    }

    public static double[] RandomDouble3()
    {
        return Vector3ToDouble(Random.insideUnitSphere);
    }

    /// <summary>
    /// centers it at zero
    /// </summary>
    /// <param name="sd"></param>
    /// <returns></returns>
    public static Vector3 RandomCenteredGaussianVector3(double sd)
    {
        return new Vector3(
            (float)RandomGaussian(0,sd),
            (float)RandomGaussian(0,sd),
            (float)RandomGaussian(0,sd));
    }

    public static double[] RandomGaussianArray(int n, double sd)
    {
        return Enumerable.Range(0, n).Select(x => RandomGaussian(0, sd)).ToArray();
    }
    
    /// <summary>
    /// derived from https://stackoverflow.com/questions/218060/random-gaussian-variables
    /// </summary>
    /// <param name="mean"></param>
    /// <param name="stdDev"></param>
    /// <returns></returns>
    public static double RandomGaussian(double mean, double sd)
    {
        double u1 = 1.0-Random.value; //uniform(0,1] random doubles
        double u2 = 1.0-Random.value;
        double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                               Math.Sin(2.0 * Math.PI * u2); //random normal(0,1)
        double randNormal =
            mean + sd * randStdNormal; //random normal(mean,stdDev^2)
        return randNormal;
    }

    public static Vector3 DoubleToVector3(double[] v)
    {
        var vf = v.Select(x => (float) x).ToArray();
        return new Vector3(vf[0], vf[1], vf[2]);
    }

    public static int RoundDownToNearestEvenInt(float testAndValidationLen)
    {
        int x = (int)Math.Round(testAndValidationLen * 0.5);
        if (x%2!=0)
        {
            x -= 1;
        }

        return (x);
    }

    public static double[] RandomFromUnitCube(float multiplier, float bias)
    {
        return RandomDouble3().Select(x=> x*multiplier + bias).ToArray();
    }


    public static long UnixTimeLong()
    {
        return DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }
}
 
/// <summary>
/// derived from: https://answers.unity.com/questions/956047/serialize-quaternion-or-vector3.html accessed 11/24/2019
/// </summary>
[System.Serializable]
public struct SerializableVector3
{
    public float x;
    public float y;
    public float z;

    public SerializableVector3(float rX, float rY, float rZ)
    {
        x = rX;
        y = rY;
        z = rZ;
    }
    public override string ToString()
    {
        return String.Format("[{0}, {1}, {2}]", x, y, z);
    }
    public static implicit operator Vector3(SerializableVector3 rValue)
    {
        return new Vector3(rValue.x, rValue.y, rValue.z);
    }
    public static implicit operator SerializableVector3(Vector3 rValue)
    {
        return new SerializableVector3(rValue.x, rValue.y, rValue.z);
    }
}

 
/// <summary>
/// derived from: https://answers.unity.com/questions/956047/serialize-quaternion-or-vector3.html accessed 11/24/2019
/// </summary>
[System.Serializable]
public struct SerializableQuaternion
{
    public float x;
    public float y;
    public float z;
    public float w;
    public SerializableQuaternion(float rX, float rY, float rZ, float rW)
    {
        x = rX;
        y = rY;
        z = rZ;
        w = rW;
    }
     
    public override string ToString()
    {
        return $"[{x}, {y}, {z}, {w}]";
    }

    public static implicit operator Quaternion(SerializableQuaternion rValue)
    {
        return new Quaternion(rValue.x, rValue.y, rValue.z, rValue.w);
    }
     
    public static implicit operator SerializableQuaternion(Quaternion rValue)
    {
        return new SerializableQuaternion(rValue.x, rValue.y, rValue.z, rValue.w);
    }
}