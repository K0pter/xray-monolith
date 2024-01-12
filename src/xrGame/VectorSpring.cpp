#include "stdafx.h"
#include "VectorSpring.h"
#include "../xrPhysics/MathUtils.h"
#include "../xrCore/vector.h"


VectorSpring::VectorSpring()
{
	Speed = Fvector().set(1.,1.,1.);
	Damping = Fvector().set(3.,3.,3.);
	Target = Fvector().set(0,0,0);
	Limits_Min = Fvector().set(-1,-1,-1);
	Limits_Max = Fvector().set(1, 1, 1);

	NormalizeResult = false;
}

VectorSpring* VectorSpring::Create(const float speed, const float damping)
{
	VectorSpring* NewSpring = new VectorSpring();
	NewSpring->Speed = Fvector().set(speed, speed, speed);
	NewSpring->Damping = Fvector().set(damping, damping, damping);

	return NewSpring;
}
VectorSpring* VectorSpring::Create(const Fvector speed, const Fvector damping)
{
	VectorSpring* NewSpring = new VectorSpring();
	NewSpring->Speed = speed;
	NewSpring->Damping = damping;

	return NewSpring;
}

void VectorSpring::Update(float DeltaTime)
{
	const int currentFPS = (1.f / DeltaTime);

	//Increase sub-stepping the lower the fps goes. This somewhat ensures frame rate "safe" calculations & increased general stability
	int Iterations = MinIterations;
	if (currentFPS < 30) { Iterations++; }
	if (currentFPS < 15) { Iterations++; }
	if (currentFPS < 10) { Iterations++; }
	if (currentFPS < 3) { return; }
	clamp(Iterations, MinIterations, MaxIterations);

	//Params (New objects to avoid changing base param values)
	Fvector TargetValue = Fvector().set(Target);
	Fvector SpeedParam = Fvector().set(Speed).mul(SPRING_BASEFACTOR);
	Fvector DampingParam = Fvector().set(Damping);

	for (int i = 0; i < Iterations; i++)
	{
		const float SubStepDelta = (DeltaTime / (float)Iterations);

		//Delta between current result and target result
		const Fvector TargetDelta = TargetValue.sub(Current_Result);

		//Active force to bring us to target
		const Fvector SpringForce = SpeedParam.mul(SubStepDelta).mul(TargetDelta);

		//Apply delta to velocity
		Current_Velocity.add(SpringForce);

		const Fvector DampingFactor = DampingParam.mul(SubStepDelta).add(1.f);

		//Apply damping to velocity afterwards
		Current_Velocity.div(DampingFactor);

		//Set up resulting velocity to add onto current
		const Fvector ModulatedVelocity = Fvector().set(Current_Velocity).mul(SubStepDelta);

		//Apply diff as velocity factor
		if (NormalizeResult)
		{
			//This is good to use for achieving a "directional spring", that's "pointing" in a direction
			Current_Result.add(ModulatedVelocity).normalize();
		}
		else
		{
			//"Normal" behavior
			Current_Result.add(ModulatedVelocity);
		}

		//Msg("Delta (%f,%f,%f) :: Force(%f,%f,%f) @ (sd:%f)", TargetDelta.x, TargetDelta.y, TargetDelta.z, SpringForce.x, SpringForce.y, SpringForce.z, SubStepDelta);

		//Clamp results to limits
		ClampResult();
	}

	IsActive = (Current_Velocity.magnitude() > 0.0001f);
}

void VectorSpring::Reset()
{
	Current_Result = Target;
	Current_Velocity = Fvector();

	IsActive = false;
}

void VectorSpring::ClampResult()
{
	//Clamp results to limits set by parameter
	clamp(Current_Result.x, Limits_Min.x, Limits_Max.x);
	clamp(Current_Result.y, Limits_Min.y, Limits_Max.y);
	clamp(Current_Result.z, Limits_Min.z, Limits_Max.z);
}

void VectorSpring::AddImpulse(Fvector Impulse, const float InstantRatio)
{
	Current_Velocity.add(Impulse);

	if (InstantRatio > 0)
	{
		const Fvector modImpulse = Impulse.mul(InstantRatio);

		Current_Result.add(modImpulse);
		ClampResult();
	}
}