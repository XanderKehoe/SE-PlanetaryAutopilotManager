using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        static float totalTime = 0;

        static double totalRunTime = 0;
        static float totalRunTimes = 0;

        public MyIni ini = new MyIni();

        public Program()
        {
            PlanetaryAutopilotManager.Init(GridTerminalSystem, this, ref ini);
        }

        public void Save()
        {
            ini.Clear(); // clear out old data to prepare for new data

            PlanetaryAutopilotManager.Save(ref ini);

            Storage = ini.ToString();
        }

        public void Main(string argument, UpdateType updateSource)
        {
            PlanetaryAutopilotManager.Update(this, updateSource);

            totalRunTime += Runtime.LastRunTimeMs;
            totalRunTimes++;

            Echo("Runtime (ms): " + Runtime.LastRunTimeMs);
            Echo("Avg. Runtime (ms)" + Math.Round(totalRunTime / totalRunTimes, 4));
            Echo("IC: " + Runtime.CurrentInstructionCount + " / " + Runtime.MaxInstructionCount);


            if (argument != null || argument == "")
            {
                string[] argSplit = argument.Split(new string[] { "," }, StringSplitOptions.None);

                string command = argSplit[0].ToUpper();

                if (command == "GOTO")
                {
                    if (argSplit.Length != 2)
                    {
                        Echo("Invalid amount of parameters for command 'GOTO' (GOTO, *MyWaypointInfo*)");
                        return;
                    }

                    MyWaypointInfo waypointInfo;
                    if (MyWaypointInfo.TryParse(argSplit[1], out waypointInfo))
                    {
                        PlanetaryAutopilotManager.SetDestination(waypointInfo.Coords, this);
                        Echo("Destination succesfully set!");

                        PlanetaryAutopilotManager.FlightCompleted += OnFlightCompletion;
                    }
                    else
                    {
                        Echo("Failed to parse waypoint info (copy to clipboard from GPS menu)");
                    }
                }
                else if (command == "STOP")
                {
                    PlanetaryAutopilotManager.Stop(this);
                    Echo("Stopping...");
                }
                else if (command == "TARGET_ELEVATION")
                {
                    if (argSplit.Length == 2)
                    {
                        Echo("Invalid amount of parameters for command 'TARGET_ELEVATION' (TARGET_ELEVATION, *number (m)*)");
                        return;
                    }

                    float targetElevation;
                    if (float.TryParse(argSplit[1], out targetElevation))
                    {
                        PlanetaryAutopilotManager.SetTargetElevation(targetElevation);
                    }
                    else
                    {
                        Echo("Failed to parse target elevation info (enter a number)");
                    }
                }
                else if (command == "ELEVATION_TYPE")
                {
                    if (argSplit.Length == 2)
                    {
                        Echo("Invalid amount of parameters for command 'TARGET_ELEVATION' (TARGET_ELEVATION, SURFACE or SEALEVEL)");
                        return;
                    }

                    if (argSplit[1].ToUpper() == "SEALEVEL")
                    {
                        PlanetaryAutopilotManager.SetElevationType(MyPlanetElevation.Sealevel);
                        Echo("Succesfully changed elevation type to 'SEALEVEL'");
                    }
                    else if (argSplit[1].ToUpper() == "SURFACE")
                    {
                        PlanetaryAutopilotManager.SetElevationType(MyPlanetElevation.Surface);
                        Echo("Succesfully changed elevation type to 'SURFACE'");
                    }
                    else
                    {
                        Echo("Invalid elevation type, please enter 'SURFACE' or 'SEALEVEL'");
                    }
                }
                else if (command == "MAX_SPEED")
                {
                    if (argSplit.Length == 2)
                    {
                        Echo("Invalid amount of parameters for command 'MAX_SPEED' (MAX_SPEED, *number (m/s)*)");
                        return;
                    }

                    float maxSpeed;
                    if (float.TryParse(argSplit[1], out maxSpeed))
                    {
                        PlanetaryAutopilotManager.SetMaxSpeed(maxSpeed);
                    }
                    else
                    {
                        Echo("Failed to parse max speed info (enter a number)");
                    }
                }
                else if (command != "")
                {
                    Echo("Command not recognized");
                }
            }

            totalTime += (float)Runtime.LastRunTimeMs / 1000;
        }

        public void OnFlightCompletion()
        {
            Echo("Flight completed! Thanks for using the Planetary Autopilot Script.");
        }

        public static class HelperFunctions
        {
            /*public const int MAX_DEBUG_CONSOLE_LENGTH = 10000;

            public static void CreateDbgMsg(string str, MyGridProgram gp)
            {
                string currentData = gp.Me.CustomData;

                string newData;

                if (currentData.Length + str.Length > MAX_DEBUG_CONSOLE_LENGTH)
                {
                    string subCurrentData = currentData.Substring(0, currentData.Length - str.Length); //substring of current data with old data removed

                    newData = "["+Math.Round(totalTime, 2)+"]" + str + "\n" + subCurrentData;
                }
                else
                {
                    newData = str + "\n" + currentData;
                }

                gp.Me.CustomData = newData;
            }*/

            // Credit to Whiplash141 for this function
            public static double VectorAngleBetween(Vector3D a, Vector3D b) //returns radians 
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                    return 0;
                else
                    return Math.Acos(MathHelper.Clamp(a.Dot(b) / Math.Sqrt(a.LengthSquared() * b.LengthSquared()), -1, 1));
            }

            //  Credit to Whiplash141 for this function
            public static double DistanceFromVector(Vector3D origin, Vector3D direction, Vector3D currentPos)
            {
                direction.Normalize();
                Vector3D lhs = currentPos - origin;

                double dotP = lhs.Dot(direction);
                Vector3D closestPoint = origin + direction * dotP;
                return Vector3D.Distance(currentPos, closestPoint);
            }

            // Credit to Whiplash141 for this function
            public static Vector3D SafeNormalize(Vector3D a)
            {
                if (Vector3D.IsZero(a))
                    return Vector3D.Zero;

                if (Vector3D.IsUnit(ref a))
                    return a;

                return Vector3D.Normalize(a);
            }

            // Credit to Whiplash141 for this function
            // This function is slightly modified to prioritize desiredUpVector over desiredForwardVector
            public static void GetRotationAnglesSimultaneous(Vector3D desiredForwardVector, Vector3D desiredUpVector, MatrixD worldMatrix, out double pitch, out double yaw, out double roll)
            {
                desiredForwardVector = SafeNormalize(desiredForwardVector);

                MatrixD transposedWm;
                MatrixD.Transpose(ref worldMatrix, out transposedWm);
                Vector3D.Rotate(ref desiredForwardVector, ref transposedWm, out desiredForwardVector);
                Vector3D.Rotate(ref desiredUpVector, ref transposedWm, out desiredUpVector);

                Vector3D leftVector = Vector3D.Cross(desiredUpVector, desiredForwardVector);
                Vector3D axis;
                double angle;
                if (Vector3D.IsZero(desiredUpVector) || Vector3D.IsZero(leftVector))
                {
                    axis = new Vector3D(desiredForwardVector.Y, -desiredForwardVector.X, 0);
                    angle = Math.Acos(MathHelper.Clamp(-desiredForwardVector.Z, -1.0, 1.0));
                }
                else
                {
                    leftVector = SafeNormalize(leftVector);
                    desiredUpVector = SafeNormalize(desiredUpVector);

                    Vector3D forwardVector = SafeNormalize(Vector3D.Cross(leftVector, desiredUpVector));

                    // Create matrix
                    MatrixD targetMatrix = MatrixD.Zero;
                    targetMatrix.Forward = forwardVector;
                    targetMatrix.Left = leftVector;
                    targetMatrix.Up = desiredUpVector;

                    axis = new Vector3D(targetMatrix.M23 - targetMatrix.M32,
                                        targetMatrix.M31 - targetMatrix.M13,
                                        targetMatrix.M12 - targetMatrix.M21);

                    double trace = targetMatrix.M11 + targetMatrix.M22 + targetMatrix.M33;
                    angle = Math.Acos(MathHelper.Clamp((trace - 1) * 0.5, -1, 1));
                }

                if (Vector3D.IsZero(axis))
                {
                    angle = desiredForwardVector.Z < 0 ? 0 : Math.PI;
                    yaw = angle;
                    pitch = 0;
                    roll = 0;
                    return;
                }

                axis = SafeNormalize(axis);
                // Because gyros rotate about -X -Y -Z, we need to negate our angles
                yaw = -axis.Y * angle;
                pitch = -axis.X * angle;
                roll = -axis.Z * angle;
            }

            // Credit to Whiplash141 for this function
            public static void ApplyGyroOverride(double pitch_speed, double yaw_speed, double roll_speed, List<IMyGyro> gyro_list, IMyTerminalBlock reference)
            {
                var rotationVec = new Vector3D(pitch_speed, yaw_speed, roll_speed); //because keen does some weird stuff with signs
                var shipMatrix = reference.WorldMatrix;
                var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);
                foreach (var thisGyro in gyro_list)
                {
                    var gyroMatrix = thisGyro.WorldMatrix;
                    var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));
                    thisGyro.Pitch = (float)transformedRotationVec.X;
                    thisGyro.Yaw = (float)transformedRotationVec.Y;
                    thisGyro.Roll = (float)transformedRotationVec.Z;
                    thisGyro.GyroOverride = true;
                }
            }
        }

        public static class PlanetaryAutopilotManager
        {
            public static float maxSpeed = 100f;

            private static float targetElevation = 500f; // the elevation the ship will try to maintain
            private static float targetElevationTolerance = 25f; // if ship's elevation is between targetElevation+-targetElevationTolerance, it will not adjust its elevation.

            public static float scanningDistanceForwardModifier = 1.5f; // Modifier to increase scanning range, should be greater than cruisingDistanceModifier 
            public static float scanningDistanceDownwardModifier = 2f; // Modifier to increase scanning range
            public static float cruisingDistanceModifier = 1.25f; // Modifier to increase distance the ship can continue forward (while going up) even if it sees an obstacle, don't recommend setting below 1.1f.

            // You can change these in code before initialization if needed (although I can't see why that would be necessarry)
            public static string nameOfRemoteControl = "Remote Control";
            public static string nameOfHorizontalCameraGroup = "PA Forward Cams";
            public static string nameOfVerticalCameraGroup = "PA Down Cams";

            public static MyPlanetElevation elevationType { get; private set; } = MyPlanetElevation.Surface; // how we should measure the ship's current height

            // Do not change below this line until end of this class

            public static double GRAVITY { get; private set; }

            public static Vector3D destination { get; private set; }
            public static bool destinationIsSet { get; private set; } = false;

            public static List<IMyCameraBlock> forwardCams { get; private set; } = new List<IMyCameraBlock>();
            public static List<IMyCameraBlock> downCams { get; private set; } = new List<IMyCameraBlock>();
            public static IMyRemoteControl rc { get; private set; }

            public static List<IMyGyro> gyros { get; private set; }

            private static List<IMyThrust> forwardThrusters = new List<IMyThrust>();
            private static List<IMyThrust> backThrusters = new List<IMyThrust>();
            private static List<IMyThrust> upThrusters = new List<IMyThrust>();

            private static bool initialized = false;
            private static string errorStr = "";
            private static bool hasError = false;

            private static int nextForwardCamForScanIndex = 0;
            private static int nextDownCamForScanIndex = 0;

            private static bool forwardHitDetected = false;
            private static bool downHitDetected = false;

            // we should scan both directly in front and our estimated future position simulateously
            // this variable controls the alternation between the two.
            private static bool forwardCamStraightRaycastAlternate = false;
            private static float forwardCamStraightRaycastAngle = -2.5f;

            enum SpeedMode
            {
                Accelerating,
                Cruising,
                Decelerating,
            }

            // You can subscribe to this event to be notified when the ship has reached it's destination
            public delegate void FlightCompleteHandler();
            public static event FlightCompleteHandler FlightCompleted;

            public static void Init(IMyGridTerminalSystem gts, MyGridProgram gp, ref MyIni ini)
            {
                SetUpdateFrequency(true, gp);

                // initialize blocks and block groups.
                IMyBlockGroup forwardCamsGroup = gts.GetBlockGroupWithName(nameOfHorizontalCameraGroup);
                if (forwardCamsGroup != null)
                    forwardCamsGroup.GetBlocksOfType(forwardCams);
                else
                    SetError("No block group by name: " + nameOfHorizontalCameraGroup + " found...");

                IMyBlockGroup downCamsGroup = gts.GetBlockGroupWithName(nameOfVerticalCameraGroup);
                if (downCamsGroup != null)
                    downCamsGroup.GetBlocksOfType(downCams);
                else
                    SetError("No block group by name: " + nameOfVerticalCameraGroup + " found...");

                rc = gts.GetBlockWithName(nameOfRemoteControl) as IMyRemoteControl;

                gyros = new List<IMyGyro>();
                gts.GetBlocksOfType(gyros);
                foreach (IMyGyro gyro in gyros)
                    gyro.GyroOverride = false;

                List<IMyThrust> allThrusters = new List<IMyThrust>();
                gts.GetBlocksOfType(allThrusters);

                // RC has to be checked for errors first since thruster groups are assigned based on its orientation
                if (rc == null)
                {
                    SetError("Remote Control was not found");
                }
                else
                {
                    foreach (IMyThrust thisThruster in allThrusters)
                    {
                        if (thisThruster.Orientation.Forward == Base6Directions.GetFlippedDirection(rc.Orientation.Forward))
                            forwardThrusters.Add(thisThruster);
                        else if (thisThruster.Orientation.Forward == rc.Orientation.Forward)
                            backThrusters.Add(thisThruster);
                        else if (thisThruster.Orientation.Forward == Base6Directions.GetFlippedDirection(rc.Orientation.Up))
                            upThrusters.Add(thisThruster);

                        thisThruster.ThrustOverridePercentage = 0f;

                        GRAVITY = rc.GetNaturalGravity().Length();
                    }
                }

                // check for errors
                if (forwardCams.Count == 0)
                {
                    SetError("No forward cameras were found");
                }
                else
                {
                    foreach (IMyCameraBlock cam in forwardCams)
                    {
                        if (cam.Orientation.Forward != rc.Orientation.Forward)
                        {
                            SetError(" - " + cam.Name + " in " + nameOfHorizontalCameraGroup + " is not facing in the same direction as the Remote Control");
                        }
                    }
                }

                if (downCams.Count == 0)
                {
                    SetError("No downward cameras were found");
                }
                else
                {
                    foreach (IMyCameraBlock cam in downCams)
                    {
                        if (cam.Orientation.Forward != Base6Directions.GetFlippedDirection(rc.Orientation.Up))
                        {
                            SetError(" - " + cam.Name + " in " + nameOfVerticalCameraGroup + " is not facing in the downward direction of the Remote Control");
                        }
                    }
                }

                if (gyros.Count == 0)
                {
                    SetError("No Gyros were found");
                }

                // if this is 0, but rc is null, skip because rc being null is why these are empty.
                if (forwardThrusters.Count == 0 && rc != null)
                {
                    SetError("No forward thrusters were found");
                }

                if (backThrusters.Count == 0 && rc != null)
                {
                    SetError("No back thrusters were found");
                }

                if (upThrusters.Count == 0 && rc != null)
                {
                    SetError("No up thrusters were found");
                }

                if (cruisingDistanceModifier < 1.1f)
                {
                    SetError("cruisingDistanceModifier was less than 1.1f");
                }

                if (scanningDistanceDownwardModifier < 1.1f)
                {
                    SetError("scanningDistanceDownwardModifier was less than 1.1f");
                }

                if (scanningDistanceForwardModifier <= cruisingDistanceModifier)
                {
                    SetError("scanningDistanceForwardModifier was less than or equal to cruisingDistanceModifier");
                }

                if (targetElevation < 0)
                {
                    SetError("targetElevation was less than 0");
                }

                if (targetElevationTolerance < 0)
                {
                    SetError("targetElevationTolerance was less than 0");
                }

                initialized = true;

                if (hasError)
                {
                    ResetThrusters();
                }
                else
                {
                    //HelperFunctions.CreateDbgMsg("Attempting loading...", gp);
                    Load(gp.Storage, ref ini);
                }
            }

            public static void Save(ref MyIni ini)
            {
                //HelperFunctions.CreateDbgMsg("Saving data...", gp);
                ini.Clear();

                ini.Set("save", "destination", destination.ToString());
                ini.Set("save", "destinationIsSet", destinationIsSet.ToString());
                ini.Set("save", "maxSpeed", maxSpeed.ToString());
                ini.Set("save", "targetElevation", targetElevation.ToString());
            }

            public static void Load(string Storage, ref MyIni ini)
            {
                bool storageParseSuccess = ini.TryParse(Storage);
                if (storageParseSuccess)
                {
                    //HelperFunctions.CreateDbgMsg("Succesfully parsed storage!", gp);

                    if (ini.ContainsKey("save", "destination"))
                    {
                        Vector3D tempDestination;
                        if (Vector3D.TryParse(ini.Get("save", "destination").ToString(), out tempDestination))
                        {
                            destination = tempDestination;
                        }
                    }

                    if (ini.ContainsKey("save", "destinationIsSet"))
                    {
                        bool tempDestinationIsSet;
                        if (bool.TryParse(ini.Get("save", "destinationIsSet").ToString(), out tempDestinationIsSet))
                        {
                            destinationIsSet = tempDestinationIsSet;

                            //HelperFunctions.CreateDbgMsg("Successfully loaded destinationIsSet with a value of: " + destinationIsSet, gp);
                        }
                    }
                    /*else
                        HelperFunctions.CreateDbgMsg("did not find a saved value for 'destinationIsSet'", gp);*/

                    if (ini.ContainsKey("save", "maxSpeed"))
                    {
                        float tempMaxSpeed;
                        if (float.TryParse(ini.Get("save", "maxSpeed").ToString(), out tempMaxSpeed))
                        {
                            maxSpeed = tempMaxSpeed;
                        }
                    }

                    if (ini.ContainsKey("save", "targetElevation"))
                    {
                        float tempTargetElevation;
                        if (float.TryParse(ini.Get("save", "targetElevation").ToString(), out tempTargetElevation))
                        {
                            targetElevation = tempTargetElevation;
                        }
                    }
                }
                /*else
                {
                    HelperFunctions.CreateDbgMsg("Did NOT succesfully parse storage", gp);
                }*/
            }

            public static void Update(MyGridProgram gp, UpdateType updateSource)
            {
                if (initialized)
                {
                    gp.Echo("Initialized...");
                    if (!hasError)
                    {
                        if (destinationIsSet)
                        {
                            gp.Echo("Destination Set - facing required direction...");

                            bool facingCorrect = SetGyroOverride();

                            if (facingCorrect)
                            {
                                gp.Echo("Facing correct direction, travelling to destination");
                                GoToDestination(gp, updateSource);
                            }
                            else
                            {
                                gp.Echo("Orientating to face target destination...");
                                ResetThrusters();
                            }
                        }
                    }
                    else
                    {
                        gp.Echo("Error(s) in setup detected, fix and then recomplile \n" + errorStr);
                    }
                }
                else
                {
                    gp.Echo("PlanetaryAutopilotManager - Update called but class is not initialized...");
                }
            }

            // To be called once ship is properly aligned towards destination
            private static void GoToDestination(MyGridProgram gp, UpdateType updateSource)
            {
                double verticalVelocity = GetVerticalVelocity();

                double stoppingHorizontalDistance = GetHorizontalStoppingDistance() + 50;
                double stoppingVerticalDistance = GetVerticalStoppingDistance(verticalVelocity);

                Vector3D currentPos = rc.GetPosition();
                Vector3D planetPos;

                if (rc.TryGetPlanetPosition(out planetPos))
                {
                    // if we are close enough to the destination vector & check angles to ensure were not on the opposite side of the planet
                    if (HelperFunctions.DistanceFromVector(planetPos, destination - planetPos, currentPos) < stoppingHorizontalDistance
                        && MathHelper.ToDegrees(HelperFunctions.VectorAngleBetween(currentPos - planetPos, destination - planetPos)) < 75)
                    {
                        Stop(gp);

                        FlightCompleted?.Invoke();

                        return;
                    }
                }

                Vector3D shipVelocity = rc.GetShipVelocities().LinearVelocity;

                double forwardRaycastLength = stoppingHorizontalDistance * scanningDistanceDownwardModifier;
                double downRaycastLength = stoppingVerticalDistance * scanningDistanceDownwardModifier;

                Vector3D forwardScanPos = rc.GetPosition() + Vector3D.Normalize(shipVelocity) * forwardRaycastLength;
                Vector3D downScanPos = rc.GetPosition() + Vector3D.Normalize(shipVelocity) * downRaycastLength;

                IMyCameraBlock forwardCam = GetCamForScan(forwardScanPos, forwardCams, ref nextForwardCamForScanIndex);
                IMyCameraBlock downCam = GetCamForScan(downScanPos, downCams, ref nextDownCamForScanIndex);

                if (forwardCam != null || downCam != null)
                {
                    if (downCam != null && !Vector3D.IsZero(shipVelocity) && verticalVelocity < 0)
                    {

                        MyDetectedEntityInfo downRaycastInfo;

                        if (MathHelper.ToDegrees(HelperFunctions.VectorAngleBetween(downCam.WorldMatrix.Forward, forwardScanPos - downCam.GetPosition())) < downCam.RaycastConeLimit)
                        {
                            downRaycastInfo = downCam.Raycast(downScanPos);
                        }
                        else
                            downRaycastInfo = downCam.Raycast(downRaycastLength, downCam.RaycastConeLimit);


                        Vector3D? downHitPosition = downRaycastInfo.HitPosition;
                        if (downHitPosition.HasValue)
                        {
                            //HelperFunctions.CreateDbgMsg("Downward raycast hit a target. shd: " + stoppingHorizontalDistance, gp);
                            //gp.Echo("Downward raycast hit a target");

                            Vector3D hitPos = downHitPosition.Value;

                            double angle = HelperFunctions.VectorAngleBetween(Vector3.Subtract(hitPos, downCam.Position), downCam.WorldMatrix.Forward);

                            if (Vector3D.Distance(downCam.GetPosition(), hitPos)
                                * Math.Cos(angle) > stoppingVerticalDistance + 100)
                            {
                                // obstacle approaching but can still continue to travel forward while going up
                                SetThrusterGroup(upThrusters, null, SpeedMode.Accelerating, false);
                            }

                            downHitDetected = true;
                        }
                        else
                            downHitDetected = false;
                    }

                    if (forwardCam != null && !Vector3D.IsZero(shipVelocity))
                    {
                        MyDetectedEntityInfo forwardRaycastInfo;

                        if (!forwardCamStraightRaycastAlternate || verticalVelocity > 0)
                        {
                            forwardRaycastInfo = forwardCam.Raycast(forwardRaycastLength, forwardCamStraightRaycastAngle);

                            forwardCamStraightRaycastAlternate = true;
                        }
                        else
                        {
                            forwardCamStraightRaycastAlternate = false;

                            if (MathHelper.ToDegrees(HelperFunctions.VectorAngleBetween(forwardCam.WorldMatrix.Forward, forwardScanPos - forwardCam.GetPosition())) < forwardCam.RaycastConeLimit)
                            {
                                //HelperFunctions.CreateDbgMsg("Forward raycast within bounds ", gp);
                                forwardRaycastInfo = forwardCam.Raycast(forwardScanPos);
                            }
                            else
                            {
                                //HelperFunctions.CreateDbgMsg("Forward raycast outside bounds going down ", gp);
                                forwardRaycastInfo = forwardCam.Raycast(forwardRaycastLength, -forwardCam.RaycastConeLimit);
                            }
                        }

                        Vector3D? forwardHitPosition = forwardRaycastInfo.HitPosition;
                        if (forwardHitPosition.HasValue)
                        {
                            //HelperFunctions.CreateDbgMsg("Forward raycast hit a target. ", gp);
                            //gp.Echo("Forward raycast hit a target. ");

                            Vector3D hitPos = forwardHitPosition.Value;

                            double angle = HelperFunctions.VectorAngleBetween(Vector3.Subtract(hitPos, forwardCam.Position), forwardCam.WorldMatrix.Forward);

                            if (Vector3D.Distance(forwardCam.GetPosition(), forwardHitPosition.Value)
                                * Math.Cos(angle) > stoppingHorizontalDistance * cruisingDistanceModifier + 100)
                            {
                                // obstacle approaching but can still continue to travel forward while going up
                                SetThrusterGroup(upThrusters, null, SpeedMode.Accelerating, false);
                            }
                            else
                            {
                                // obstacle approaching and little time to brake, turn off forward thrusters, turn on back thrusters
                                SetThrusterGroup(forwardThrusters, backThrusters, SpeedMode.Decelerating);
                                SetThrusterGroup(upThrusters, null, SpeedMode.Accelerating, false);
                            }

                            forwardHitDetected = true;
                        }
                        else
                            forwardHitDetected = false;
                    }
                }

                if (forwardHitDetected || downHitDetected)
                    return;

                // raycast did not find any obstacles, continue cruising along
                double currentElevation;
                bool success = rc.TryGetPlanetElevation(elevationType, out currentElevation);

                if (success)
                {
                    SetThrustForElevation(currentElevation, targetElevation);
                }
                else
                {
                    SetError("Did not find a planet.");

                    //HelperFunctions.CreateDbgMsg("Did not find a planet, entering error mode...", gp);

                    ResetThrusters();
                }

                ControlCruisingSpeed();
            }

            // credit to whiplast141 for this function
            // rotates gyros towards the set destination
            // returns true if facing the correct direction for travel
            // false if not facing the correct direction (still orientating)
            private static bool SetGyroOverride()
            {
                if (initialized)
                {
                    Vector3 dirToDestination = Vector3D.Normalize(destination - rc.GetPosition());

                    double pitch, yaw, roll;

                    HelperFunctions.GetRotationAnglesSimultaneous(dirToDestination, rc.GetNaturalGravity() * -1, rc.WorldMatrix, out pitch, out yaw, out roll);

                    HelperFunctions.ApplyGyroOverride(pitch, yaw, roll, gyros, rc);

                    float accuracyThreshhold = 0.1f;
                    return pitch < accuracyThreshhold && yaw < accuracyThreshhold && roll < accuracyThreshhold;
                }

                return false;
            }

            // Controls horizontal speed to match the set maxSpeed 
            private static void ControlCruisingSpeed()
            {
                if (initialized)
                {
                    float tolerance = 1f; // how far can we be off from the max speed and still be considered ok.

                    if (rc.GetShipSpeed() > maxSpeed - tolerance && rc.GetShipSpeed() < maxSpeed + tolerance)
                    {
                        SetThrusterGroup(forwardThrusters, backThrusters, SpeedMode.Cruising);
                    }
                    else if (rc.GetShipSpeed() < maxSpeed)
                    {
                        SetThrusterGroup(forwardThrusters, backThrusters, SpeedMode.Accelerating);
                    }
                    else
                    {
                        SetThrusterGroup(forwardThrusters, backThrusters, SpeedMode.Decelerating);
                    }
                }
            }

            private static void EnableRaycast(bool enable)
            {
                foreach (IMyCameraBlock cam in forwardCams)
                    cam.EnableRaycast = enable;

                foreach (IMyCameraBlock cam in downCams)
                    cam.EnableRaycast = enable;
            }

            // turns on all thrusters and disables their override
            private static void ResetThrusters()
            {
                if (initialized)
                {
                    foreach (IMyThrust thisThruster in forwardThrusters)
                    {
                        thisThruster.ApplyAction("OnOff_On");
                        thisThruster.ThrustOverridePercentage = 0f;
                    }

                    foreach (IMyThrust thisThruster in backThrusters)
                    {
                        thisThruster.ApplyAction("OnOff_On");
                        thisThruster.ThrustOverridePercentage = 0f;
                    }

                    foreach (IMyThrust thisThruster in upThrusters)
                    {
                        thisThruster.ApplyAction("OnOff_On");
                        thisThruster.ThrustOverridePercentage = 0f;
                    }
                }
            }

            // Turns power and thruster override on/off for input thrusterGroup and applies opposite action to opposingThrusterGroup.
            // opposingThrusterGroup can be null (example upward facing thrusters with no counteracting down thrusters with gravity)
            private static void SetThrusterGroup(List<IMyThrust> thrusterGroup, List<IMyThrust> opposingThrusterGrouper, SpeedMode speedMode, bool horizontal = true)
            {
                IMyThrust firstThruster = thrusterGroup.First();
                IMyThrust firstOpposingThruster = null;
                if (opposingThrusterGrouper != null)
                    firstOpposingThruster = opposingThrusterGrouper.First();

                switch (speedMode)
                {
                    case SpeedMode.Cruising:
                        {
                            foreach (IMyThrust thruster in thrusterGroup)
                            {
                                if (horizontal)
                                    thruster.Enabled = false;
                                else //(vertical)
                                    thruster.Enabled = true;
                                thruster.ThrustOverridePercentage = 0f;
                            }

                            if (opposingThrusterGrouper != null)
                            {
                                foreach (IMyThrust thruster in opposingThrusterGrouper)
                                {
                                    thruster.Enabled = false;
                                    thruster.ThrustOverridePercentage = 0f;
                                }
                            }

                            break;
                        }

                    case SpeedMode.Accelerating:
                        {
                            if (!firstThruster.Enabled || firstThruster.ThrustOverridePercentage < 1f)
                            {
                                foreach (IMyThrust thruster in thrusterGroup)
                                {
                                    thruster.Enabled = true;
                                    thruster.ThrustOverridePercentage = 1f;
                                }

                            }

                            if (opposingThrusterGrouper != null)
                            {
                                if (firstOpposingThruster.Enabled || firstOpposingThruster.ThrustOverridePercentage > 0f)
                                    foreach (IMyThrust thruster in opposingThrusterGrouper)
                                    {
                                        thruster.Enabled = false;
                                        thruster.ThrustOverridePercentage = 0f;
                                    }
                            }

                            break;
                        }

                    case SpeedMode.Decelerating:
                        {
                            if (firstThruster.Enabled || firstThruster.ThrustOverridePercentage > 0f)
                            {
                                foreach (IMyThrust thruster in thrusterGroup)
                                {
                                    thruster.Enabled = false;
                                    thruster.ThrustOverridePercentage = 0f;
                                }
                            }

                            if (opposingThrusterGrouper != null)
                            {
                                if (!firstOpposingThruster.Enabled || firstOpposingThruster.ThrustOverridePercentage > 0f)
                                    foreach (IMyThrust thruster in opposingThrusterGrouper)
                                    {
                                        thruster.Enabled = true;
                                        thruster.ThrustOverridePercentage = 1f;
                                    }
                            }

                            break;
                        }
                }

            }

            // returns the distance needed to come to a full stop given the backwards facing thrusters & current ship mass
            public static double GetHorizontalStoppingDistance()
            {
                if (initialized)
                {
                    float totalBackwardsForce = 0;
                    foreach (IMyThrust thruster in backThrusters)
                        totalBackwardsForce = thruster.MaxEffectiveThrust;

                    float shipMass = rc.CalculateShipMass().PhysicalMass;

                    float backwardsAcceleration = totalBackwardsForce / shipMass;

                    double stoppingDistance = Math.Pow(GetHorizontalVelocity(), 2) / (2 * backwardsAcceleration);

                    return stoppingDistance;
                }
                else
                    return float.NaN;
            }

            // returns the distance needed to come to a full stop along the vertical axis
            public static double GetVerticalStoppingDistance(double verticalVelocity)
            {
                if (initialized)
                {
                    double stoppingDistance;
                    if (verticalVelocity > 0)
                    {
                        stoppingDistance = Math.Pow(verticalVelocity, 2) / (2 * GRAVITY);
                    }
                    else
                    {
                        float totalUpwardThrust = 0;
                        foreach (IMyThrust thruster in upThrusters)
                            totalUpwardThrust += thruster.MaxEffectiveThrust;

                        float shipMass = rc.CalculateShipMass().PhysicalMass;

                        stoppingDistance = Math.Pow(verticalVelocity, 2) / (2 * (totalUpwardThrust / shipMass));
                    }

                    return stoppingDistance;
                }
                else
                    return float.NaN;
            }

            // called when raycast detect no obstacles, controls thrusters to make ship stay at desired targetElevation.
            private static void SetThrustForElevation(double currentElevation, double targetElevation)
            {
                double distanceToTargetElevation = Math.Abs(currentElevation - targetElevation);

                // if already within tolerance, just maintain current height...
                if (distanceToTargetElevation < targetElevationTolerance)
                {
                    if (!upThrusters.First().Enabled || upThrusters.First().ThrustOverridePercentage > 0)
                        foreach (IMyThrust thruster in upThrusters)
                        {
                            thruster.Enabled = true;
                            thruster.ThrustOverridePercentage = 0;
                        }

                    return;
                }

                double verticalVelocity = GetVerticalVelocity();

                if (currentElevation > targetElevation && verticalVelocity > 0)
                {
                    // above target elevation and already going up, turn off upward thrusters if needed
                    if (upThrusters.First().Enabled)
                        foreach (IMyThrust thruster in upThrusters)
                            thruster.Enabled = false;

                    return;
                }


                double shipMass = rc.CalculateShipMass().PhysicalMass;

                if (currentElevation > targetElevation && verticalVelocity < 0) //falling
                {
                    // F = (mv^2 / 2d) + (mg)
                    double totalNeededForce = ((shipMass * Math.Pow(verticalVelocity, 2))
                        / (distanceToTargetElevation * 2)) + (shipMass * GRAVITY);

                    float thrustPerThruster = (float)totalNeededForce / upThrusters.Count;

                    if (thrustPerThruster <= 0)
                    {
                        if (upThrusters.First().Enabled)
                            foreach (IMyThrust thruster in upThrusters)
                                thruster.Enabled = false;
                    }
                    else
                    {
                        foreach (IMyThrust thruster in upThrusters)
                        {
                            thruster.Enabled = true;
                            thruster.ThrustOverride = thrustPerThruster;
                        }
                    }
                }
                else
                {
                    if (verticalVelocity <= 0 || GetVerticalStoppingDistance(verticalVelocity) < distanceToTargetElevation)
                    {
                        if (!upThrusters.First().Enabled || upThrusters.First().ThrustOverridePercentage < 1f)
                            foreach (IMyThrust thruster in upThrusters)
                            {
                                thruster.Enabled = true;
                                thruster.ThrustOverridePercentage = 1f;
                            }
                    }
                    else
                    {
                        if (upThrusters.First().Enabled)
                            foreach (IMyThrust thruster in upThrusters)
                            {
                                thruster.Enabled = false;
                                thruster.ThrustOverridePercentage = 0f;
                            }
                    }
                }
            }

            // returns the vertical velocity relative to the planets center
            // going away from planet (+), going towards (-)
            public static double GetVerticalVelocity()
            {
                Vector3D gravityVec = rc.GetNaturalGravity();
                Vector3D shipVelocity = rc.GetShipVelocities().LinearVelocity;

                double angle = HelperFunctions.VectorAngleBetween(gravityVec, shipVelocity);

                double verticalVelocity = (shipVelocity.Length() * Math.Cos(angle)) * -1;

                return verticalVelocity;
            }

            // returns the velocity relative to the Remote Control's forward direction
            public static double GetHorizontalVelocity()
            {
                Vector3D forwardDir = rc.WorldMatrix.Forward;
                Vector3D shipVelocity = rc.GetShipVelocities().LinearVelocity;

                double angle = HelperFunctions.VectorAngleBetween(forwardDir, shipVelocity);

                double horizontalVelocity = shipVelocity.Length() * Math.Cos(angle);

                return horizontalVelocity;
            }

            // Returns a camera that can scan the given target location
            // Returns null if no cam can scan the target or last cam scanned too recently.
            private static IMyCameraBlock GetCamForScan(Vector3D target, List<IMyCameraBlock> camGroup, ref int nextCamForScanIndex)
            {
                IMyCameraBlock ret = null;

                if (camGroup.Count == 1)
                {
                    if (camGroup[0].CanScan(Vector3D.Distance(camGroup[0].GetPosition(), target) * 1.1f))
                        return camGroup[0];
                }

                int previousIndex = nextCamForScanIndex - 1;
                if (previousIndex == -1)
                    previousIndex = camGroup.Count - 1;

                if (camGroup[previousIndex].AvailableScanRange
                    >= Vector3.Distance(camGroup[previousIndex].GetPosition(), target) / camGroup.Count
                    && camGroup[nextCamForScanIndex].CanScan(Vector3D.Distance(camGroup[0].GetPosition(), target) * 1.1f))
                {
                    ret = camGroup[nextCamForScanIndex];

                    nextCamForScanIndex++;
                    if (nextCamForScanIndex > camGroup.Count - 1)
                        nextCamForScanIndex = 0;
                }

                return ret;
            }

            // enables or disables the Runtime.UpdateFrequency
            private static void SetUpdateFrequency(bool enable, MyGridProgram gp)
            {
                /*
                if (enable)
                    gp.Runtime.UpdateFrequency |= UpdateFrequency.Update10 | UpdateFrequency.Update100;
                else
                    gp.Runtime.UpdateFrequency = gp.Runtime.UpdateFrequency & ~(UpdateFrequency.Update10 | UpdateFrequency.Update100);
                */

                if (enable)
                    gp.Runtime.UpdateFrequency |= UpdateFrequency.Update10;
                else
                    gp.Runtime.UpdateFrequency = gp.Runtime.UpdateFrequency & ~(UpdateFrequency.Update10);
            }

            // sets an error that the user must fix before program will allow continuation of functionality.
            private static void SetError(string str)
            {
                errorStr += "- " + str + "\n";
                hasError = true;
            }

            // sets the destination and starts the ship to travel to the set destination
            public static void SetDestination(Vector3D newDestination, MyGridProgram gp)
            {
                destination = newDestination;
                destinationIsSet = true;

                EnableRaycast(true);

                SetUpdateFrequency(true, gp);
            }

            // Stops the ship, resets all thrusters and gyros to user control, and clears its set destination.
            public static void Stop(MyGridProgram gp)
            {
                ResetThrusters();

                EnableRaycast(false);

                SetUpdateFrequency(false, gp);

                foreach (IMyGyro gyro in gyros)
                    gyro.GyroOverride = false;

                destinationIsSet = false;
            }

            // Sets the max speed of the ship, can be set while in flight.
            public static void SetMaxSpeed(float newMaxSpeed)
            {
                maxSpeed = newMaxSpeed;
            }

            // Sets the target elevation of the ship, can be set while in flight.
            public static void SetTargetElevation(float newTargetElevation)
            {
                targetElevation = newTargetElevation;
            }

            // Sets the elevation type of the ship, can be set while in flight.
            // elevation type determines how target elevation is measured:
            // - MyPlanetElevation.Sealevel measures from the distance from the planet's sealevel
            // - MyPlanetElevation.Surface measures from the distance from the surface
            public static void SetElevationType(MyPlanetElevation newElevationType)
            {
                elevationType = newElevationType;
            }
        }

    }
}
