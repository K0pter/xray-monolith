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



	// Composes the two supplied matrices (Fist combine rotations, then multiply B's position by that result)
	static Fmatrix& compose(const Fmatrix& A, const Fmatrix& B)
	{
		Fmatrix composedMatrix;

		//First combine rotations
		composedMatrix.m[0][0] = A.m[0][0] * B.m[0][0] + A.m[1][0] * B.m[0][1] + A.m[2][0] * B.m[0][2];
		composedMatrix.m[0][1] = A.m[0][1] * B.m[0][0] + A.m[1][1] * B.m[0][1] + A.m[2][1] * B.m[0][2];
		composedMatrix.m[0][2] = A.m[0][2] * B.m[0][0] + A.m[1][2] * B.m[0][1] + A.m[2][2] * B.m[0][2];
		composedMatrix.m[0][3] = 0;

		composedMatrix.m[1][0] = A.m[0][0] * B.m[1][0] + A.m[1][0] * B.m[1][1] + A.m[2][0] * B.m[1][2];
		composedMatrix.m[1][1] = A.m[0][1] * B.m[1][0] + A.m[1][1] * B.m[1][1] + A.m[2][1] * B.m[1][2];
		composedMatrix.m[1][2] = A.m[0][2] * B.m[1][0] + A.m[1][2] * B.m[1][1] + A.m[2][2] * B.m[1][2];
		composedMatrix.m[1][3] = 0;

		composedMatrix.m[2][0] = A.m[0][0] * B.m[2][0] + A.m[1][0] * B.m[2][1] + A.m[2][0] * B.m[2][2];
		composedMatrix.m[2][1] = A.m[0][1] * B.m[2][0] + A.m[1][1] * B.m[2][1] + A.m[2][1] * B.m[2][2];
		composedMatrix.m[2][2] = A.m[0][2] * B.m[2][0] + A.m[1][2] * B.m[2][1] + A.m[2][2] * B.m[2][2];
		composedMatrix.m[2][3] = 0;

		//Now we multiply the "child" matrix position by the combined rotation
		composedMatrix.m[3][0] =  (A.m[3][0] * composedMatrix.m[0][0]) + (A.m[3][0] * composedMatrix.m[1][0]) + (A.m[3][0] * composedMatrix.m[2][0]);
		composedMatrix.m[3][1] =  (A.m[3][1] * composedMatrix.m[1][0]) + (A.m[3][1] * composedMatrix.m[1][1]) + (A.m[3][1] * composedMatrix.m[1][2]);
		composedMatrix.m[3][2] =  (A.m[3][2] * composedMatrix.m[2][0]) + (A.m[3][2] * composedMatrix.m[2][1]) + (A.m[3][2] * composedMatrix.m[2][2]);
		
		//composedMatrix.m[3][0] = A.m[3][0] * (composedMatrix.m[0][0] + composedMatrix.m[1][0] + composedMatrix.m[2][0]);
		//composedMatrix.m[3][1] = A.m[3][1] * (composedMatrix.m[1][0] + composedMatrix.m[1][1] + composedMatrix.m[1][2]);
		//composedMatrix.m[3][2] = A.m[3][2] * (composedMatrix.m[2][0] + composedMatrix.m[2][1] + composedMatrix.m[2][2]);
		
		
		
		composedMatrix.m[3][3] = 1;



		//Offset by parent's base position
		composedMatrix.m[3][0] += B.m[3][0];
		composedMatrix.m[3][1] += B.m[3][1];
		composedMatrix.m[3][2] += B.m[3][2];

		return composedMatrix;
	}
};


