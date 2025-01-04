package frc.lib.drivers.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class QuestNav {
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
    private static NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    private static NetworkTable nt4Table = nt4Instance.getTable("questnav");
    private static IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private static IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    private static IntegerSubscriber questFrameCount =
            nt4Table.getIntegerTopic("frameCount").subscribe(0);

    private static DoubleSubscriber questTimestamp =
            nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);

    private static FloatArraySubscriber questPosition =
            nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    private static FloatArraySubscriber questQuaternion =
            nt4Table.getFloatArrayTopic("quaternion")
                    .subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});

    private static FloatArraySubscriber questEulerAngles =
            nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    private static DoubleSubscriber questBattery =
            nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

    // publishers
    private static DoubleArrayPublisher questOdometry =
            nt4Table.getDoubleArrayTopic("odometry").publish();

    private static StructPublisher<Pose2d> questOdometryStruct =
            NetworkTableInstance.getDefault()
                    .getStructTopic("QuestOdometry", Pose2d.struct)
                    .publish();
    
    private static DoubleArrayPublisher questOdometryOffset =
            nt4Table.getDoubleArrayTopic("odometryOffset").publish();

    // Local heading helper variables
    private static double yawOffset = 0.0f;

    private static Translation2d questTranslationOffset = new Translation2d();

    public static void periodic() {
        publishOdometry();
        cleanUpMessages();

        Logger.recordOutput("QuestNav/Pose", getQuestNavPose());
        Logger.recordOutput("QuestNav/FrameCount", questFrameCount.get());
        Logger.recordOutput("QuestNav/Timestamp", questTimestamp.get());
        Logger.recordOutput("QuestNav/RawPositionFloats", questPosition.get());
        Logger.recordOutput("QuestNav/Quaternion", getQuaternion());
        Logger.recordOutput("QuestNav/Battery", questBattery.get());
        Logger.recordOutput("QuestNav/Offset", questBattery.get());
    }

    public static void resetQuestPose(Pose2d pose) {
        resetHeading(pose.getRotation().getDegrees());
        resetTranslation(pose.getTranslation());
    }

    public static void resetTranslation(Translation2d translation) {
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
        questTranslationOffset = translation;
    }

    public static void resetHeading(double angleDegrees) {
        float[] eulerAngles = questEulerAngles.get();
        yawOffset = eulerAngles[1] + angleDegrees;
    }

    // Clean up QuestNav subroutine messages after processing on the headset
    public static void cleanUpMessages() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    // Return the robot heading in degrees, between -180 and 180 degrees
    public static double getQuestNavHeading() {
        return Rotation2d.fromDegrees(getQuestNavYaw()).getDegrees();
    }

    // Get the yaw Euler angle of the headset
    public static double getQuestNavYaw() {
        float[] eulerAngles = questEulerAngles.get();
        double ret = eulerAngles[1] - yawOffset;
        ret %= 360;
        if (ret < 0) {
            ret += 360;
        }
        return 360 - ret;
    }

    public static Quaternion getQuaternion() {
        float[] quaternion = questQuaternion.get();
        return new Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    }

    public static Translation2d getQuestNavRawPosition() {
        float[] questNavPosition = questPosition.get();
        return new Translation2d(questNavPosition[2], -questNavPosition[0]);
    }

    public static Translation2d getQuestNavPosition() {
        return getQuestNavRawPosition().plus(questTranslationOffset);
    }

    public static Pose2d getQuestNavPose() {
        return new Pose2d(getQuestNavPosition(), Rotation2d.fromDegrees(getQuestNavYaw()));
    }

    public static void publishOdometry() {
        Pose2d pose = getQuestNavPose();
        double[] odometry = {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
        questOdometry.set(odometry);
        questOdometryStruct.set(pose);

        double[] odometryOffset = {questTranslationOffset.getX(), questTranslationOffset.getY(), yawOffset};
        questOdometryOffset.set(odometryOffset);
    }
}