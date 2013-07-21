#include "InvPendulum.h"

InvPendulum::InvPendulum()
{
    //we should estimate these from the character info...
    legLength = 1;
    
    coronalStepWidth = 0.1;
}

/**
	returns a panic level which is 0 if val is between minG and maxG, 1 if it's
	smaller than minB or larger than maxB, and linearly interpolated 
*/
double getValueInFuzzyRange(double val, double minB, double minG, double maxG, double maxB)
{
	if (val <= minB || val >= maxB)
		return 1;
	if (val >= minG && val <= maxG)
		return 0;
	if (val > minB && val < minG)
		return (minG - val) / (minG - minB);
	if (val > maxG && val < maxB)
		return (val - maxG) / (maxB - maxG);
	//the input was probably wrong, so return panic...
	return 1;
}

/**
	this method determines the degree to which the character should be panicking
*/
double InvPendulum::getPanicLevel(const RobotInfo& rinfo, double velDSagittal, double velDCoronal)
{
	//the estimate of the panic is given, roughly speaking by the difference between the desired and actual velocities
	double panicEstimate = 0;
	panicEstimate += getValueInFuzzyRange(rinfo.getV().x(), velDSagittal-0.4, velDSagittal-0.3, velDSagittal+0.3, velDSagittal+0.4);
	panicEstimate += getValueInFuzzyRange(rinfo.getV().y(), velDCoronal-0.3, velDCoronal-0.2, velDCoronal+0.2, velDCoronal+0.3);
//	boundToRange(&panicEstimate, 0, 1);
	return panicEstimate/2;
}

/**
	determines the desired swing foot location
*/
void InvPendulum::calcDesiredSwingFootLocation(const RobotInfo& rinfo, double velDSagittal, double velDCoronal, Eigen::Vector3d& desiredPos, Eigen::Vector3d& desiredVel)
{
	Eigen::Vector3d step0 = computeSwingFootLocationEstimate(rinfo, rinfo.comPos(), rinfo.phi(), velDSagittal, velDCoronal);
	
	double dt = 0.001;
	Eigen::Vector3d step1 = computeSwingFootLocationEstimate(rinfo, rinfo.comPos() + rinfo.comVel() * dt, rinfo.phi()+dt, velDSagittal, velDCoronal);
	
	desiredPos = step0;
	desiredVel = (step1 - step0) / dt;
}

/**
	returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
	on the assumption that the character will come to a stop by taking a step at that location. The step location
	is expressed in the character's frame coordinates.
*/
Eigen::Vector3d InvPendulum::computeIPStepLocation(const RobotInfo& rinfo)
{
	Eigen::Vector3d step;
	const double g = 9.8;
	const double h = fabs(rinfo.comPos().z() - rinfo.stanceFootPos().z());
	const double vx = rinfo.getV().x();
	const double vy = rinfo.getV().y();
	
	step.x() = vx * sqrt(h/g + vx*vx / (4*g*g)) * 1.1;
	step.y() = vy * sqrt(h/g + vy*vy / (4*g*g)) * 1.3;
	step.z() = 0;
	
	return step;
}

/**
	determine the estimate desired location of the swing foot, given the etimated position of the COM, and the phase
*/
Eigen::Vector3d InvPendulum::computeSwingFootLocationEstimate(const RobotInfo& rinfo, const Eigen::Vector3d& comPos, double phase, double velDSagittal, double velDCoronal)
{
	Eigen::Vector3d step = computeIPStepLocation(rinfo);

	//applying the IP prediction would make the character stop, so take a smaller step if you want it to walk faster, or larger
	//if you want it to go backwards
	step.x() -= velDSagittal / 20;
	//and adjust the stepping in the coronal plane in order to account for desired step width...
	step.y() = adjustCoronalStepLocation(rinfo, step.y());

	boundToRange(&step.x(), -0.4 * legLength, 0.4 * legLength);
	boundToRange(&step.y(), -0.4 * legLength, 0.4 * legLength);

	Eigen::Vector3d initialStep = swingFootStartPos - comPos;
	initialStep = rinfo.characterFrame().conjugate()._transformVector(initialStep);
	//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
	//now interpolate between this position and initial foot position - but provide two estimates in order to provide some gradient information
	double t = (1-phase);
	t = t * t;
	boundToRange(&t, 0, 1);

	Eigen::Vector3d result = (1-t) * step + t * initialStep;

	result.z() = 0;

	return result;
}

double InvPendulum::getCoronalPanicLevel(const RobotInfo& rinfo)
{
    double stepWidth = coronalStepWidth / 2;
	stepWidth = (rinfo.stance() == LEFT_STANCE)?(-stepWidth):(stepWidth);
	
	//now for the step in the coronal direction - figure out if the character is still doing well - panic = 0 is good, panic = 1 is bad...
    double panicLevel;
	if (rinfo.stance() == LEFT_STANCE){
		panicLevel = getValueInFuzzyRange(rinfo.getD().y(), 1.15 * stepWidth, 0.5 * stepWidth, 0.25 * stepWidth, -0.25 * stepWidth);
		panicLevel += getValueInFuzzyRange(rinfo.getV().y(), 2*stepWidth, stepWidth, -stepWidth, -stepWidth*1.5);
	}
	else{
		panicLevel = getValueInFuzzyRange(rinfo.getD().y(), -0.25 * stepWidth, 0.25 * stepWidth, 0.5 * stepWidth, 1.15 * stepWidth);
		panicLevel += getValueInFuzzyRange(rinfo.getV().y(), -stepWidth*1.5, -stepWidth, stepWidth, 2*stepWidth);
	}
	boundToRange(&panicLevel, 0, 1);
	
	return panicLevel;
}

/**
	modify the coronal location of the step so that the desired step width results.
*/
double InvPendulum::adjustCoronalStepLocation(const RobotInfo& rinfo, double IPPrediction)
{
	//nothing to do if it's the default value...
	if (coronalStepWidth < 0.01)
		return IPPrediction;

	double stepWidth = coronalStepWidth / 2;
	stepWidth = (rinfo.stance() == LEFT_STANCE)?(-stepWidth):(stepWidth);

	const double panicLevel = getCoronalPanicLevel(rinfo);
	
	Trajectory1d offsetMultiplier;
	offsetMultiplier.addKnot(0.05, 0); offsetMultiplier.addKnot(0.075, 1/2.0);
	double offset = stepWidth * offsetMultiplier.evaluate_linear(fabs(stepWidth));
//	if (IPPrediction * stepWidth < 0) offset = 0;
	//if it's doing well, use the desired step width...
	IPPrediction = panicLevel * (IPPrediction + offset) + (1-panicLevel) * stepWidth;

//	if (panicLevel >= 1)
//		tprintf("panic level: %lf; d.x = %lf\n", panicLevel, lowLCon->d.x);

	return IPPrediction;
}

double InvPendulum::calcComOffsetCoronal(const RobotInfo& rinfo)
{
    double stepWidth = coronalStepWidth / 2;
    stepWidth = (rinfo.stance() == LEFT_STANCE)?(-stepWidth):(stepWidth);
    
    return (1.-getCoronalPanicLevel(rinfo)) * stepWidth;
}


