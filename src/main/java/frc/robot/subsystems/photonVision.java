package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants;

public class photonVision {
    private final PhotonCamera m_cameraOne;
    private final PhotonPoseEstimator m_cameraOneEstimator;

    public photonVision(String cameraName){
        m_cameraOne = new PhotonCamera(cameraName);
        m_cameraOneEstimator = new PhotonPoseEstimator(Constants.vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.vision.localizationCameraOneToRobot);
        m_cameraOneEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : m_cameraOne.getAllUnreadResults()) {
            visionEst = m_cameraOneEstimator.update(change);
            //updateEstimationStdDevs(visionEst, m_cameraOneEstimator, change.getTargets());
        }
        return visionEst;
    }

    @SuppressWarnings("unused")
    public double getMovement(){
        var results = m_cameraOne.getAllUnreadResults();
        double targetYaw = 0;
        boolean targetVisible = false;

        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }
        return -1.0 * targetYaw * 0.01 * Constants.Swerve.maxAngularVelocity;
    }
}
