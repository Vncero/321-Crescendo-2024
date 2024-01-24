package org.robolancers321.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;

public class TeleopDrive extends FeedForwardDrive {
    public TeleopDrive(XboxController controller, boolean fieldCentric){
        super(controller::getLeftY, controller::getLeftX, controller::getRightX, () -> fieldCentric);
    }
    
    public TeleopDrive(XboxController controller){
        this(controller, true);
    }
}
