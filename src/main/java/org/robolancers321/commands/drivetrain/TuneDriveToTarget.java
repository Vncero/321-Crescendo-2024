package org.robolancers321.commands.drivetrain;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneDriveToTarget extends Command {
    /*
     * Constants
     */

    private static final double kTranslationP = 0.0;
    private static final double kTranslationI = 0.0;
    private static final double kTranslationD = 0.0;

    private static final double kRotationP = 0.0;
    private static final double kRotationI = 0.0;
    private static final double kRotationD = 0.0;

    /*
     * Implementation
     */

    Drivetrain drivetrain;
    PhotonCamera camera;

    DoubleSupplier desiredDxSupplier;
    DoubleSupplier desiredDzSupplier;
    DoubleSupplier desiredDThetaSupplier;

    PIDController xController;
    PIDController zController;
    PIDController thetaController;

    public TuneDriveToTarget(DoubleSupplier desiredDxSupplier, DoubleSupplier desiredDzSupplier, DoubleSupplier desiredDThetaSupplier){
        this.drivetrain = Drivetrain.getInstance();
        this.camera = new PhotonCamera("photonvision");

        this.desiredDxSupplier = desiredDxSupplier;
        this.desiredDzSupplier = desiredDzSupplier;
        this.desiredDThetaSupplier = desiredDThetaSupplier;

        this.xController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
        this.zController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
        this.thetaController = new PIDController(kRotationP, kRotationI, kRotationD);

        SmartDashboard.putNumber("translation controller kp", SmartDashboard.getNumber("translation controller kp", kTranslationP));
        SmartDashboard.putNumber("translation controller ki", SmartDashboard.getNumber("translation controller ki", kTranslationI));
        SmartDashboard.putNumber("translation controller kd", SmartDashboard.getNumber("translation controller kd", kTranslationD));

        SmartDashboard.putNumber("rotation controller kp", SmartDashboard.getNumber("rotation controller kp", kRotationP));
        SmartDashboard.putNumber("rotation controller ki", SmartDashboard.getNumber("rotation controller ki", kRotationI));
        SmartDashboard.putNumber("rotation controller kd", SmartDashboard.getNumber("rotation controller kd", kRotationD));

        this.addRequirements(this.drivetrain);
    }

    public TuneDriveToTarget(double desiredDx, double desiredDz, double desiredTheta){
        this(() -> desiredDx, () -> desiredDz, () -> desiredTheta);
    }

    @Override
    public void execute(){
        double tunedTranslationP = SmartDashboard.getNumber("translation controller kp", kTranslationP);
        double tunedTranslationI = SmartDashboard.getNumber("translation controller ki", kTranslationI);
        double tunedTranslationD = SmartDashboard.getNumber("translation controller kd", kTranslationD);

        this.xController.setPID(tunedTranslationP, tunedTranslationI, tunedTranslationD);
        this.zController.setPID(tunedTranslationP, tunedTranslationI, tunedTranslationD);

        double tunedRotationP = SmartDashboard.getNumber("rotation controller kp", kRotationP);
        double tunedRotationI = SmartDashboard.getNumber("rotation controller ki", kRotationI);
        double tunedRotationD = SmartDashboard.getNumber("rotation controller kd", kRotationD);

        this.thetaController.setPID(tunedRotationP, tunedRotationI, tunedRotationD);

        double desiredDx = this.desiredDxSupplier.getAsDouble();
        double desiredDz = this.desiredDzSupplier.getAsDouble();
        double desiredDTheta = this.desiredDThetaSupplier.getAsDouble();

        this.xController.setSetpoint(desiredDx);
        this.zController.setSetpoint(desiredDz);
        this.thetaController.setSetpoint(desiredDTheta);

        double actualDx;
        double actualDz;
        double actualDTheta;

        PhotonPipelineResult visionResults = this.camera.getLatestResult();

        SmartDashboard.putBoolean("has vision target", visionResults.hasTargets());

        if(visionResults.hasTargets()){
            PhotonTrackedTarget bestTarget = visionResults.getBestTarget();

            Transform3d relativeCameraPosition = bestTarget.getBestCameraToTarget();

            actualDx = relativeCameraPosition.getX();
            actualDz = relativeCameraPosition.getZ();
            actualDTheta = relativeCameraPosition.getRotation().getY();
        } else {
            actualDx = desiredDx;
            actualDz = desiredDz;
            actualDTheta = desiredDTheta;
        }

        SmartDashboard.putNumber("actual target dx", actualDx);
        SmartDashboard.putNumber("actual target dz", actualDz);
        SmartDashboard.putNumber("actual target dtheta", actualDTheta);


        double xControllerOutput = this.xController.calculate(actualDx);
        double zControllerOutput = this.zController.calculate(actualDz);
        double thetaControllerOutput = this.thetaController.calculate(actualDTheta);

        SmartDashboard.putNumber("x controller output", xControllerOutput);
        SmartDashboard.putNumber("z controller output", zControllerOutput);
        SmartDashboard.putNumber("theta controller output", thetaControllerOutput);

        // TODO: check vision outputs before actually calling drive
        // this.drivetrain.drive(zControllerOutput, xControllerOutput, thetaControllerOutput, false);
    }
}
