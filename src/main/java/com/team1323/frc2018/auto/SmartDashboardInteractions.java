package com.team1323.frc2018.auto;

import com.team1323.frc2018.auto.modes.LLAssistMode;
import com.team1323.frc2018.auto.modes.LRAssistMode;
import com.team1323.frc2018.auto.modes.LeftFrontSwitchMode;
import com.team1323.frc2018.auto.modes.LeftScaleMode;
import com.team1323.frc2018.auto.modes.LeftSwitchLeftScaleMode;
import com.team1323.frc2018.auto.modes.LeftSwitchRightScaleMode;
import com.team1323.frc2018.auto.modes.RLAssistMode;
import com.team1323.frc2018.auto.modes.RRAssistMode;
import com.team1323.frc2018.auto.modes.RightFrontSwitchMode;
import com.team1323.frc2018.auto.modes.RightScaleMode;
import com.team1323.frc2018.auto.modes.RightSwitchLeftScaleMode;
import com.team1323.frc2018.auto.modes.RightSwitchRightScaleMode;
import com.team1323.frc2018.auto.modes.StandStillMode;
import com.team1323.frc2018.auto.modes.TestMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
	private static final String AUTO_OPTIONS = "auto_options";
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.SWITCH_ONLY;
    
    private SendableChooser<AutoOption> modeChooser;
    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
    	modeChooser.addDefault(DEFAULT_MODE.name, DEFAULT_MODE);
    	//modeChooser.addObject("Switch Only", AutoOption.SWITCH_ONLY);
		modeChooser.addObject("Scale Only", AutoOption.SCALE_ONLY);
		modeChooser.addObject("2 Switch + 1 Scale", AutoOption.ASSIST);
    	
    	SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(String gameData){
        AutoOption selectedOption =  (AutoOption)  modeChooser.getSelected();
                
        return createAutoMode(selectedOption, gameData);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }
    
    enum AutoOption{
    	SWITCH_AND_SCALE("Switch and Scale (Deprecated)"),
    	SWITCH_ONLY("Switch Only"),
		SCALE_ONLY("Scale Only"),
		ASSIST("Assist");
    	
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }
    
    private AutoModeBase createAutoMode(AutoOption option, String gameData){
    	switch(option){
    		case SWITCH_AND_SCALE:
    			switch(gameData){
    			case "RR":
    				return new RightSwitchRightScaleMode();
    			case "RL":
    				return new RightSwitchLeftScaleMode();
    			case "LR":
    				return new LeftSwitchRightScaleMode();
    			case "LL":
    				return new LeftSwitchLeftScaleMode();
    			case "TT":
    				return new TestMode();
    			default:
    				System.out.println("ERROR: unexpected auto mode: " + option);
                    return new StandStillMode();
    			}
    		case SWITCH_ONLY:
    			switch(gameData){
    			case "LL":
    				return new LeftFrontSwitchMode(false);
    			case "LR":
    				return new LeftFrontSwitchMode(true);
    			case "RR":
    				return new RightFrontSwitchMode(false);
    			case "RL":
    				return new RightFrontSwitchMode(true);
    			default:
    				System.out.println("ERROR: unexpected auto mode: " + option);
                    return new StandStillMode();
    			}
    		case SCALE_ONLY:
    			switch(gameData){
    			case "LL":
    				//fall-through intended
    			case "RL":
    				return new LeftScaleMode();
    			case "RR":
    				//fall-through intended
    			case "LR":
					return new RightScaleMode();
				case "TT":
    				return new TestMode();
    			default:
    				System.out.println("ERROR: unexpected auto mode: " + option);
                    return new StandStillMode();
				}
			case ASSIST:
				switch(gameData){
				case "LL":
					return new LLAssistMode();
				case "LR":
					return new LRAssistMode();
				case "RR":
					return new RRAssistMode();
				case "RL":
					return new RLAssistMode();
				default:
					System.out.println("ERROR: unexpected auto mode: " + option);
					return new StandStillMode();
				}
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}
