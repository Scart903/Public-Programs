print(" ").
print("-----------------------------").
print(" CargoSSTO.ks               ").
print("  ").
print("-----------------------------").
print(" ").

//Setup
//-------------
//Variable setup
set turnAdjustSpeed to 5.
set MySteer to heading(0,0).

//Calculating variables

//Part Setup

//Functions
FUNCTION PrecisionCheck {
	PARAMETER checkedStat, targetStat, precisionDegree.

	if (checkedStat > (targetStat - (precisionDegree / 2))) AND (checkedStat < (targetStat + (precisionDegree / 2))) {
		RETURN True.
	} else {
		RETURN False.
	}
}

FUNCTION pitch_and_roll {
	PARAMETER myPitch, myRoll.
	LOCAL returnDir IS HEADING(heading_of_vector(SHIP:SRFPROGRADE:FOREVECTOR),myPitch).
	RETURN ANGLEAXIS(myRoll,returnDir:FOREVECTOR) * returnDir.
}

FUNCTION heading_of_vector { // heading_of_vector returns the heading of the vector (number range   0 to 360)
	PARAMETER vecT.

	LOCAL east IS VCRS(SHIP:UP:VECTOR, SHIP:NORTH:VECTOR).

	LOCAL trig_x IS VDOT(SHIP:NORTH:VECTOR, vecT).
	LOCAL trig_y IS VDOT(east, vecT).

	LOCAL result IS ARCTAN2(trig_y, trig_x).

	IF result < 0 {RETURN 360 + result.} ELSE {RETURN result.}
}

FUNCTION PlaneTurn {
	PARAMETER turnHeading.
	set desiredPitch to 0.
	set turnDone to False.

	until turnDone {
		set rollTurn to ((heading_of_vector(SHIP:SRFPROGRADE:FOREVECTOR) - turnHeading) * turnAdjustSpeed).
		if rollTurn < -45 {
			set rollTurn to -45.
		}
		if rollTurn > 45 {
			set rollTurn to 45.
		}
		set desiredRoll to rollTurn.

		if PrecisionCheck(heading_of_vector(SHIP:SRFPROGRADE:FOREVECTOR),turnHeading,1) {
			set turnDone to True.
			set desiredRoll to 0.
		}
	}
}

FUNCTION PitchUp {
    print("Pitching Up").
    PARAMETER currentPitch, targetPitch, pitchTime.
    set pitchIteration to ((targetPitch - currentPitch) / 10).
    set timeIteration to (pitchTime / 10).
    set i to 0.
    set targettingPitch to pitchIteration.
    until (i = 10) {
        set MySteer to heading(ascentHeading,targettingPitch).
        set targettingPitch to (targettingPitch + pitchIteration).
        set i to (i + 1).
        wait timeIteration.
    }
}

function GravTurn {
    parameter P1, P2, P3, P4.
    set StartAlt to P1.
    set EndAlt to P2.
    set StartHead to P3.
    set EndHead to P4.

    set AltRange to (EndAlt - StartAlt).
    set HeadRange to (StartHead - EndHead).
    set ConvFact to (HeadRange/AltRange).

    print(" ").
    print("Gravity Turn Information").
    print("Altitude range: " + (EndAlt - StartAlt) + " metres").
    print(StartAlt + " metres to " + EndAlt + " metres").
    print("Pitch range: " + (StartHead - EndHead) + " degrees").
    print(StartHead + " degrees to " + EndHead + " degrees").


    until Apoapsis >= EndAlt {
        set ReqHead to (StartHead - ((Apoapsis - StartAlt) * ConvFact)).
        set MySteer to Heading(ascentHeading,ReqHead).
        }
}

function ExecNode {
    set kuniverse:timewarp:warp to 0.
    print(" ").
    print("Prepping for Node").
    wait 10.

    set Node to nextnode.
    set MySteer to Node:deltaV.
    set Accel to ship:maxthrust/ship:mass.
    set BurnTime to Node:deltaV:mag/Accel.

    lock steering to MySteer.
    lock throttle to MyThrottle.

    print(" ").
    print("Node in " + round(Node:ETA) + " seconds").
    print("DeltaV: " + round(Node:deltaV:mag * 10) / 10).
    print("Rough Burn Time: " + round(BurnTime * 10) / 10 + " seconds").
    
    wait until Node:ETA <= ((BurnTime/2)+15).
    set MySteer to Node:deltaV.
    wait until vang(MySteer, ship:facing:vector) < 0.25.

    wait until Node:ETA <= (BurnTime/2).

    set NodeDone to False.
    set dv0 to Node:deltaV.

    print(" ").
    print("Beginning Node Burn").
    
    until NodeDone {
        set Accel to ship:maxthrust/ship:mass.
        set MyThrottle to min(Node:deltaV:mag/Accel,1).
        set MySteer to Node:deltaV.
    
        if vdot(dv0,Node:deltaV) < 0 {
            print("End Burn, Remaining dV: " + round(Node:deltav:mag,1) + " m/s, vdot:" + round(vdot(dv0,Node:deltaV),1)).
            lock throttle to 0.
            set AG10 to False.
            break.
        }

        if Node:deltaV:mag < 0.1 {
            print ("Finalizing burn, remain dv " + round(Node:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, Node:deltav),1)).
            wait until vdot(dv0, Node:deltav) < 0.5.

            lock throttle to 0.
            set AG10 to False.
            print ("End burn, remain dv " + round(Node:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, Node:deltav),1)).
            set NodeDone to True.
        }
    }
    unlock steering.
    unlock throttle.
    set MyThrottle to 0.
    wait 1.

    remove Node.
    set ship:control:pilotmainthrottle to 0.
}

function Circularise {
    print(" ").
    print("Calculating Orbit").

    set TargetOrbit to createorbit(0, 0, (600000 + Apoapsis), 0, 0, 0, 0, Kerbin).

    set CalcDone to False.
    set RoughDV to 0.
    set CircNode to node((time:seconds + ETA:apoapsis), 0, 0, RoughDV).
    add CircNode.
    set CircOrbit to CircNode:orbit.

    until CalcDone {
        set PeriDif to TargetOrbit:periapsis - CircOrbit:periapsis.
        if PeriDif > 10 {
            remove CircNode.
            set OldRoughDV to RoughDV.
            set RoughDV to (OldRoughDV + (PeriDif / 5000)).
            set CircNode to node((time:seconds + ETA:apoapsis), 0, 0, RoughDV).
            add CircNode.
            set CircOrbit to CircNode:orbit.
            wait 0.1.
        } 
        else {
            print("Orbit Calculated").
            set CalcDone to True.
        }
    }
}

//Triggers
when Abort then {
    print("Aborting program").
    set SAS to True.
}

//Misc
SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 180.

//Pre-Flight Variables

//-----------------
// Start of Launch
//-----------------
print("Pre-flight prep complete").
print("Switching to automated control").

//Flight Prep
set desiredPitch to 15.
set desiredRoll to 0.

set ascentHeading to 90.

//Takeoff
lock Throttle to MyThrottle.
set MyThrottle to 1.
set SAS to False.
lock steering to pitch_and_roll(desiredPitch,desiredRoll).

wait 1.
print("Turning to heading").
PlaneTurn(ascentHeading).
print("Turn Complete").

unlock steering.
lock steering to MySteer.
PitchUp(desiredPitch,45,20).
set AG9 to True.

wait until apoapsis > 40000.

GravTurn(40000,70000,45,0).

wait until apoapsis > 75000.
set MyThrottle to 0.

wait until altitude > 69000.
set kuniverse:timewarp:warp to 0.

wait until altitude > 70000.
Circularise().
ExecNode().
set SAS to True.
set AG8 to True.

wait 1.
print (" ").
print ("Apoapsis: " + round(Apoapsis)).
print ("Periapsis: " + round(Periapsis)).
print ("Difference: " + round(Apoapsis - Periapsis)).