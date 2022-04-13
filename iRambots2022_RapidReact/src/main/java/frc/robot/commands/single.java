package frc.robot.commands;

import frc.robot.subsystems.driveTrain;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class single extends SequentialCommandGroup {
    
public single() {
    addCommands(
         new shooterManual().withTimeout(3),
         new ParallelCommandGroup(
             driveTrain.tankDrive(7.0, -7.0);

         )
    );

}
}
