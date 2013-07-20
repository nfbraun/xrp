#include "InvPendulum.h"

InvPendulum::InvPendulum()
{
    //we should estimate these from the character info...
    legLength = 1;
    
    shouldPreventLegIntersections = true;
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
	determines weather a leg crossing is bound to happen or not, given the predicted final desired position	of the swing foot.
	The suggested via point is expressed in the character frame, relative to the COM position...The via point is only suggested
	if an intersection is detected.
*/
bool InvPendulum::detectPossibleLegCrossing(const RobotInfo& rinfo, const Vector3d& swingFootPos, Vector3d* viaPoint)
{
	//first, compute the world coords of the swing foot pos, since this is in the char. frame 
	Point3d desSwingFootPos = Quaternion(rinfo.characterFrame()).rotate(swingFootPos) + rinfo.comPos();
	//now, this is the segment that starts at the current swing foot pos and ends at the final
	//swing foot position

	Segment swingFootTraj(rinfo.swingFootPos(), desSwingFootPos); swingFootTraj.a.z() = 0; swingFootTraj.b.z() = 0;
	
	//and now compute the segment originating at the stance foot that we don't want the swing foot trajectory to pass...
	Vector3d segDir = rinfo.fstate().trToWorld(rinfo.stanceFootIndex()).onVector(Eigen::Vector3d(100., 0., 0.));
	
	segDir.z() = 0;
	Segment stanceFootSafety(Vector3d(rinfo.stanceFootPos()), Vector3d(rinfo.stanceFootPos()) + segDir);
	stanceFootSafety.a.z() = 0; stanceFootSafety.b.z() = 0;

	//now check to see if the two segments intersect...
	Segment intersect;
	stanceFootSafety.getShortestSegmentTo(swingFootTraj, &intersect);

/*
	predSwingFootPosDebug = desSwingFootPos;predSwingFootPosDebug.y = 0;
	swingSegmentDebug = swingFootTraj;
	crossSegmentDebug = stanceFootSafety;
	viaPointSuggestedDebug.setValues(0,-100,0);
*/

	//now, if this is too small, then it means the swing leg will cross the stance leg...
	double safeDist = 0.02;
	if ((intersect.b - intersect.a).norm() < safeDist){
		if (viaPoint != NULL){
			*viaPoint = Vector3d(rinfo.stanceFootPos()) + segDir.unit() * -0.05;
			(*viaPoint) -= Vector3d(rinfo.comPos());
			*viaPoint = Quaternion(rinfo.characterFrame()).inverseRotate(*viaPoint);
			viaPoint->z() = 0;
/*
			viaPointSuggestedDebug = lowLCon->characterFrame.rotate(*viaPoint) + lowLCon->comPosition;
			viaPointSuggestedDebug.y() = 0;
*/
		}
		return true;
	}
	
	return false;
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

	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	Eigen::Vector3d initialStep = swingFootStartPos - comPos;
	initialStep = rinfo.characterFrame().conjugate()._transformVector(initialStep);
	//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
	//now interpolate between this position and initial foot position - but provide two estimates in order to provide some gradient information
	double t = (1-phase);
	t = t * t;
	boundToRange(&t, 0, 1);

	Vector3d suggestedViaPoint;
	alternateFootTraj.clear();
	bool needToStepAroundStanceAnkle = false;
	if (phase < 0.8 && shouldPreventLegIntersections && getPanicLevel(rinfo, velDSagittal, velDCoronal) < 0.5)
		needToStepAroundStanceAnkle = detectPossibleLegCrossing(rinfo, step, &suggestedViaPoint);
	if (needToStepAroundStanceAnkle){
		//use the via point...
		Vector3d currentSwingStepPos = Vector3d(rinfo.swingFootPos() - comPos);
		currentSwingStepPos = Quaternion(rinfo.characterFrame()).inverseRotate(initialStep);currentSwingStepPos.y() = 0;		
		//compute the phase for the via point based on: d1/d2 = 1-x / x-phase, where d1 is the length of the vector from
		//the via point to the final location, and d2 is the length of the vector from the swing foot pos to the via point...
		double d1 = (Vector3d(step) - suggestedViaPoint).norm();
		double d2 = (suggestedViaPoint - currentSwingStepPos).norm();
		if (d2 < 0.0001) d2 = d1 + 0.001;
		double c =  d1/d2;
		double viaPointPhase = (1+phase*c)/(1+c);
		//now create the trajectory...
		alternateFootTraj.addKnot(0, initialStep);
		alternateFootTraj.addKnot(viaPointPhase, suggestedViaPoint);
		alternateFootTraj.addKnot(1, step);
		//and see what the interpolated position is...
		result = alternateFootTraj.evaluate_catmull_rom(1-t).toEigen();
//		tprintf("t: %lf\n", 1-t);
	}else{
		result += (1-t)* step;
		result += t * initialStep;
	}

	result.z() = 0;

/*
	suggestedFootPosDebug = result;
*/
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


