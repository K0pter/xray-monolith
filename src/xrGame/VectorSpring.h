#pragma once
#include "xrCore/_matrix.h";

//#include "../xrGame/game_cl_single.h"

#define SPRING_BASEFACTOR 100.f

class VectorSpring
{

private:
	//Custom springs use sub-stepping to avoid getting out of control
	int MinIterations = 1;
	int MaxIterations = 4;

public:
	VectorSpring();
	//virtual ~VectorSpring();

protected:
	Fvector Current_Result = Fvector();
	Fvector Current_Velocity = Fvector();

public:
	static VectorSpring* Create(const float speed = .5f, const float damping = 3.f);
	static VectorSpring* Create(const Fvector speed = Fvector().set(.5f,.5f,.5f), const Fvector damping = Fvector().set(3,3,3));

	void Update(float DeltaTime);
	void AddImpulse(const Fvector Impulse, const float InstantRatio = 0.f);
	void ClampResult();
	void Reset();
	Fvector GetResult() { return Current_Result; }
	Fvector GetVelocity() { return Current_Velocity; }

	bool IsActive = false;

	Fvector Speed;
	Fvector Damping;
	Fvector Target;
	Fvector Limits_Min;
	Fvector Limits_Max;

	//This is good if you want this spring to act more as a "direction" 
	bool NormalizeResult;
};


