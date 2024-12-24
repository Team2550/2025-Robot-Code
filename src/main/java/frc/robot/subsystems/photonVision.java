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

    private final PhotonCamera m_cameraTwo;
    private final PhotonPoseEstimator m_cameraTwoEstimator;

    public photonVision(String cameraOneName, String cameraTwoName){
        m_cameraOne = new PhotonCamera(cameraOneName);
        m_cameraOneEstimator = new PhotonPoseEstimator(Constants.vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.vision.localizationCameraOneToRobot);
        m_cameraOneEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_cameraTwo = new PhotonCamera(cameraTwoName);
        m_cameraTwoEstimator = new PhotonPoseEstimator(Constants.vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.vision.localizationCameraOneToRobot);
        m_cameraTwoEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(String cameraName) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if(m_cameraOne.getName() == cameraName){
            for (var change : m_cameraOne.getAllUnreadResults()) {
                visionEst = m_cameraOneEstimator.update(change);
                //updateEstimationStdDevs(visionEst, m_cameraOneEstimator, change.getTargets());
            }
        }else if(m_cameraTwo.getName() == cameraName){
            for (var change : m_cameraTwo.getAllUnreadResults()) {
                visionEst = m_cameraTwoEstimator.update(change);
                //updateEstimationStdDevs(visionEst, m_cameraOneEstimator, change.getTargets());
            }
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
