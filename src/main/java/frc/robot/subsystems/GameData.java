// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class GameData implements Sendable {

    public static final String[] teleOpShiftName = { "Transition", "Shift 1", "Shift 2",
            "Shift 3", "Shift 4", "Endgame"};
    public static final double[] shiftEndTime = { 130.0, 105.0, 80.0, 55.0, 30.0, 0.0 }; // Transition, Shift 1 ... 4,
                                                                                         // Endgame
    public static final boolean[] firstShiftScoringAllowed = { true, true, false, true, false, true };
    public static final boolean[] secondShiftScoringAllowed = { true, false, true, false, true, true };

    private boolean[] scoringAllowed = firstShiftScoringAllowed;
    private Alliance alliance;

    public void setGameDataSring(String data) {
        alliance = DriverStation.getAlliance().get();

        if (data.length() > 0) {
            if (data.charAt(0) == 'R' && alliance == Alliance.Red) {
                scoringAllowed = secondShiftScoringAllowed;
            } else if (data.charAt(0) == 'B' && alliance == Alliance.Blue) {
                scoringAllowed = secondShiftScoringAllowed;
            } else {
                scoringAllowed = firstShiftScoringAllowed;
            }
        } 
    }

    public int getCurrentShift() {
        for (int i = 0; i < shiftEndTime.length; i++) {
            if (DriverStation.getMatchTime() > shiftEndTime[i]) {
                return i;
            }
        }
        return 0; // Default to the first shift if no other shift is active
    }

    public String getCurrentShiftName() {
        if(DriverStation.isAutonomous()){
            return "Auton";
        }
        return teleOpShiftName[getCurrentShift()];
    }

    public boolean scoring() {
        int currentShift = getCurrentShift();
        return scoringAllowed[currentShift];
    }

    public double getTimeLeftInShift() {
        int currentShift = getCurrentShift();
        if (DriverStation.isDisabled()){
            return 0;
        }
        return DriverStation.getMatchTime() - shiftEndTime[currentShift];
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Current Shift", this::getCurrentShiftName, null);
        builder.addBooleanProperty("Scoring Allowed", this::scoring, null);
        builder.addDoubleProperty("Time Left In Shift", this::getTimeLeftInShift, null);
    }
}
