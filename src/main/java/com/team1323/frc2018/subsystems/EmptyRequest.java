package com.team1323.frc2018.subsystems;

public class EmptyRequest extends Request{
	
	@Override
	public void act(){
		//empty, as the name suggests
	}
	
	@Override
	public boolean isFinished(){
		return true;
	}
	
}
