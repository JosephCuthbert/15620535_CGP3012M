#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>
 
//Create global variables that can be used between classes. 
static bool BallDelete = false;
static bool capsuleScore = false;
static bool scoreTrigger = false;
static bool cornerBounce = false;

namespace PhysicsEngine
{
	using namespace std;



	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};
 

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enums
		{
			ACTOR0 = (1 << 0), //pinball
			ACTOR1 = (1 << 1), //Capsule
			ACTOR2 = (1 << 2), //Padels
			ACTOR3 = (1 << 3), //Extra
			
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box* top;
		Box2* bottom;

	
	public:
		Trampoline(const PxTransform& pose = PxTransform(PxIdentity), const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			
			PxReal thickness = .1f;
			
			bottom = new Box2(PxTransform(PxVec3(pose.p.x, pose.p.y + thickness, pose.p.z), pose.q), PxVec3(dimensions.x, thickness, dimensions.z));
			bottom->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

			top = new Box(PxTransform(PxVec3(pose.p.x, pose.p.y + (dimensions.y + thickness), pose.p.z), pose.q), PxVec3(dimensions.x, thickness, dimensions.z));
			top->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));
			

			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}
		
		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}

		void AddForce(PxReal force)
		{
			((PxRigidDynamic*)top->Get())->addForce(PxVec3(0,  force / 2, -force));
		}

	};
	
	class PinBallPA{

		//Creates the outside areas of the playable area.
		Box2* baseBox, *rightWall, *leftWall, *topWall, *bottomWall,*plungerWall,* rightCorner,*leftCorner,*triggerBox;

		//The main internal parts of the playable area.
		Box2* pitGuardLeft, * pitGuardRight,* pitGuardWallLeft,* pitGuardWallRight;

		//Creates 3 capsules
		Capsule* cap1,*cap2,*cap3;

	public:
		 
		//Using the pose from the scene customInit, sets the rotation of the
		//objects so that the pinball board is tilted towards the player
		PinBallPA(const PxTransform& pose = PxTransform(PxIdentity)) {
			
			


			//Creates the outer walls of the play area
			baseBox = new Box2 (PxTransform(PxVec3(.0f, 10.0f, 0.f), pose.q), PxVec3(7.f, 10.f, 0.5f));

			rightWall = new Box2 (PxTransform(PxVec3(7.f, 10.7f, 0.5f), pose.q), PxVec3(0.2f, 10.f, .5f));
			rightWall->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));
		
			leftWall = new Box2(PxTransform(PxVec3(-7.f, 10.7f, 0.5f), pose.q), PxVec3(0.2f, 10.f, .5f));
			leftWall->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

			topWall = new Box2 (PxTransform(PxVec3(0.f, 15.5f, -7.8f), pose.q), PxVec3(7.f, .2f, .5f));
			topWall->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

			bottomWall = new Box2(PxTransform(PxVec3(0.f, 5.8f, 9.1f), pose.q), PxVec3(6.8f, .2f, .5f));
			bottomWall->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

			//Creates a divider between the playable area and the ball spawn and plunger area.
			plungerWall = new Box2 (PxTransform(PxVec3(6.2f,8.f, 4.6f), pose.q), PxVec3(0.2f, 5.f, .2f));
			plungerWall->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

			//Two panels that redirect the pinball towards the playable area.
			rightCorner = new Box2 (PxTransform(PxVec3(6.2f, 15.0f, -7.3f), pose.q * PxQuat(PxPi / 3.2, PxVec3(0.f, 0.f, 1.f))), PxVec3(0.2f, 1.1f, .5f));
			
			leftCorner = new Box2 (PxTransform(PxVec3(-6.f, 15.0f, -7.3f), pose.q * PxQuat(PxPi / -3.2, PxVec3(0.f, 0.f, 1.f))), PxVec3(0.2f, 1.1f, .5f));
		
			//The trigger box for when the ball falls to the bottom of the playable area.
			triggerBox = new Box2(PxTransform(PxVec3(-0.3f, 5.8f,8.75f), pose.q), PxVec3(6.45f, .002f, .1f));
			triggerBox->SetTrigger(true);

			//triggerBox->SetupFiltering(FilterGroup::BOX, FilterGroup::BALL);
			triggerBox->Name("deathTrig");

		 

			//The three capsules that are placed in the playable area.
			//Capsule 1
			cap1 = new Capsule(PxTransform(PxVec3(0.f,	12.5f, -5.0f), pose.q * PxQuat(PxHalfPi, PxVec3(0, 1, 0))));
			cap1->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
			cap1->Color(PxVec3(0.f / 255.f, 0.f / 255.f, 255.f / 255.f));
			cap1->Name("capsule1");
			
			//Capsule 2
			cap2 = new Capsule(PxTransform(PxVec3(3.0f, 9.0f, 2.f), pose.q * PxQuat(PxHalfPi, PxVec3(0, 1, 0))));
			cap2->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
			cap2->Color(PxVec3(0.f / 255.f, 0.f / 255.f, 255.f / 255.f));		
			cap2->Name("capsule2");
			
			//Capsule 3
			cap3 = new Capsule(PxTransform(PxVec3(-3.5f, 9.0f,  2.f), pose.q * PxQuat(PxHalfPi, PxVec3(0, 1, 0))));
			cap3->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
			cap3->Color(PxVec3(0.f / 255.f, 0.f / 255.f, 255.f / 255.f));
			cap3->Name("capsule3");

			//
			//Guards that stops the ball from falling straight to the trigger zone
			//

			//Creates a box that acts as the tilted guard that will connect to the flippers
			pitGuardLeft = new Box2(PxTransform(PxVec3(-4.2f, 8.0f, 5.5f), pose.q * PxQuat(PxPi / -4.5, PxVec3(0.f, 0.f, 1.f))), PxVec3(2.5f, 0.2f, .5f));
			pitGuardLeft->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

			//Creates a small channel that leads down to the trigger zone but also acts as a guard on the left
			pitGuardWallLeft = new Box2(PxTransform(PxVec3(-6.05f,9.5f,2.45f), pose.q), PxVec3(0.2f, 2.f, .5f));
			pitGuardWallLeft->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));
			
			//Creates a box that acts as the tilted guard that will connect to the flippers
			pitGuardRight = new Box2(PxTransform(PxVec3(3.4f,8.0f, 5.5f), pose.q * PxQuat(PxPi / 4.5, PxVec3(0.f, 0.f, 1.f))), PxVec3(2.5f, 0.2f, .5));
			pitGuardRight->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

			//Creates a small channel that leads down to the trigger zone but also acts as a guard on the right
			pitGuardWallRight = new Box2(PxTransform(PxVec3(5.25f, 9.5f, 2.4f), pose.q), PxVec3(0.2f, 2.f, .5f));
			pitGuardWallRight->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));

		}
		void AddToScene(Scene* scene)
		{

			//Box constructors
			//
			scene->Add(baseBox);
			scene->Add(rightWall);
			scene->Add(leftWall);
			scene->Add(topWall);
			scene->Add(bottomWall);
			scene->Add(plungerWall);
			scene->Add(rightCorner);
			scene->Add(leftCorner);
			scene->Add(triggerBox);
			
			//Capsule constructors
			//
			scene->Add(cap1);
			scene->Add(cap2);
			scene->Add(cap3);

			//Pit guard constructors
			//
			scene->Add(pitGuardLeft);
			scene->Add(pitGuardRight);
			scene->Add(pitGuardWallRight);
			scene->Add(pitGuardWallLeft);

		}


	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		bool trigger = false;

		//an example variable that will be checked in the main simulation loop
		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						string triggerActorName = std::string(pairs[i].triggerActor->getName());
						string otherActorName = std::string(pairs[i].otherActor->getName());

						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;

						//Used for when the ball goes out of the play area, or falls through between the padels
						if (triggerActorName == "plane" || "deathTrig")
						{
							//Allows to keep track on if the system is working or not
							cerr << "plane" << endl;
							BallDelete = true;
						}
				
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
						
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;
			string contactName = pairHeader.actors[1]->getName();
			//check all pairs
		
			for (PxU32 i = 0; i < nbPairs; i++)
			{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;

					if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND) 
					{
						//Triggered when the pinball makes contact with the capsule.
						if (FilterGroup::ACTOR1 | FilterGroup::ACTOR2) {
							capsuleScore = true;
						}			

					
					}		
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eSOLVE_CONTACT;
		pairFlags |= PxPairFlag::eDETECT_DISCRETE_CONTACT;
		pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
			//pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		//Calls constructors
		Plane* plane;
	
		RevoluteJoint* rightJoint,* leftJoint;

		PxMaterial* wallmat;

		Trampoline* trampoline;

		MySimulationEventCallback* my_callback;

		PinballBall* ball;

		Box* box, * box2;

		Box2* squareObj, * squareObj2;

		PinBallPA* playArea;

		Box2* scoreTrigger;

		
	
	public:
		unsigned int score = 50;
		unsigned int deaths = 0;
		const PxTransform& pose = PxTransform(PxVec3(6.6f, 8.0f, 6.));

		//The custom filter shader is used instead of the default shader.
		MyScene() : Scene(CustomFilterShader){};
	
		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		 
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			
			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);
	 
			//Creates a public rotation value that can be shared throughout the project. 
			//Enables objects to be rotated in a way that allows gravity to work, while not having
			//The playarea straight up. 

			PxTransform Rotation = PxTransform(PxQuat(-PxPi / 3, PxVec3(1.f, 0.f, 0.f)));

			//Creates a plane 
			plane = new Plane();
			wallmat = CreateMaterial(210.f / 255.f, 210.f / 255.f, 210.f / 255.f);
			plane->Material(wallmat);
			plane->SetTrigger(true);
			((PxRigidActor*)plane->Get())->setName("plane");
			Add(plane);	
	
			//Calls and creates the play area, including the rotation value.
			playArea = new PinBallPA(PxTransform(PxVec3(6.6f, 5.95f, 8.12f), Rotation.q));
			playArea->AddToScene(this);
			

			//Creates the plunger																					 //Stiffness //Damping
			trampoline = new Trampoline(PxTransform(PxVec3(6.6f, 5.947f, 8.12f), Rotation.q ), PxVec3(0.15f, 0.5f, 0.15f), 100.f, 15.f);			
			trampoline->AddToScene(this);

			//Creates 2 new boxes that are then used for the padels further below.
			box = new Box(PxTransform(PxVec3(-15.f, 2.5f, -10.f)));
			box->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));
			Add(box);
			//box->SetupFiltering(FilterGroup::ACTOR2, FilterGroup::ACTOR0);
			
			box2 = new Box(PxTransform(PxVec3(-15.f, 2.5f, -10.f)));
			box2->Color(PxVec3(47.f / 255.f, 79.f / 255.f, 79.f / 255.f));
			Add(box2);
			//box2->SetupFiltering(FilterGroup::ACTOR2, FilterGroup::ACTOR0);
			
			//Creates the first revolute joint for the padels, including the rotation, positioning and size.
			leftJoint = new RevoluteJoint(0, PxTransform(PxVec3(1.58f, 7.2f, 7.f), Rotation.q * PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), box, PxTransform(PxVec3(0.1f, 0.f,1.0f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			((PxRigidActor*)this->Get())->setName("rightBumper");
			rightJoint = new RevoluteJoint(0, PxTransform(PxVec3(-2.4f, 7.2f, 7.f), Rotation.q * PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), box2, PxTransform(PxVec3(.1f, 0.f, -1.0f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));		 
			((PxRigidActor*)this->Get())->setName("leftJoint");
			
			//Sets the limit of both padels
			leftJoint->SetLimits(-PxPi / 6, PxPi / 6);
			rightJoint->SetLimits(-PxPi / 6, PxPi / 6);

			//Creates two cube objects that the ball interacts with. Same use as the Cylinder
			//Allows player to gain points when cubes are hit. 
			squareObj = new Box2(PxTransform(PxVec3(3.5f, 12.5f, -4.0f),Rotation.q * PxQuat(PxPi / 3.8, PxVec3(0.f, 0.f, 1.f))), PxVec3(0.7f, .7f, .7f));
			Add(squareObj);
			squareObj->SetupFiltering(FilterGroup::ACTOR2, FilterGroup::ACTOR0);
			squareObj->Name("squareObj");
			squareObj->Color(PxVec3(0.f / 255.f, 0.f / 255.f, 255.f / 255.f));

			squareObj2 = new Box2(PxTransform(PxVec3(-3.5f, 12.5f, -4.0f), Rotation.q * PxQuat(PxPi / 3.8, PxVec3(0.f, 0.f, 1.f))), PxVec3(0.7f, .7f, .7f));
			Add(squareObj2);
			squareObj2->SetupFiltering(FilterGroup::ACTOR2, FilterGroup::ACTOR0);
			squareObj2->Name("squareObj2");
			squareObj2->Color(PxVec3(0.f / 255.f, 0.f / 255.f, 255.f / 255.f));

			//Declaration of the pinball method. 
			spawnBall(pose);

		
			
			
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			PxRigidBody* px_actor = (PxRigidBody*)ball->Get();

			//When the bool is triggered, updates the if and executes the ballDelete method. 
			if (BallDelete) 
			{
				ballDelete();
			}

			//When capsuleScore is true, the if is triggered which then adds a force to the pinball
			//and points to the score
			if (capsuleScore)
			{
				px_actor->addForce(PxVec3(0.f, 0.f, -.15f));
				score += 50;
				capsuleScore = false;
				cerr << "Score trigger hit" << endl;
				cerr << score << endl;
			}
		}

		void spawnBall(const PxTransform pose) {

			//Creates a new pinball.
			ball = new PinballBall(pose, 0.15f, 0.01f);
			((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			((PxRigidActor*)ball->Get())->setName("ball"); //Sets the name of the pinball to "ball".
			Add(ball); 
			ball->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2);
			//Filters the ball so that it can interact with the cylinders and cubes.
			ball->Color(PxVec3(255.f / 255.f, 0.f / 255.f, 0.f / 255.f));
			//Adds a colour for the ball to make it easy to see when playing. 
		}

		//Adds the force to the top object in the trampoline method
		void AddForce()
		{
			trampoline->AddForce(22.5f);
		}

		//A case and switch used for designating the drive valocity of the padels
		//This is then executed in VisualDebugger.cpp where the id number is used to
		//Add a force to the correct padel.
		void PadelForce(int id)
		{
			switch (id)
			{
			case 1:
				rightJoint->DriveVelocity(+15);
				break;

			case 2:
				rightJoint->DriveVelocity(-15);
				break;

			case 3:
				leftJoint->DriveVelocity(+15);
				break;

			case 4:
				leftJoint->DriveVelocity(-15);
				break;

			default:
				cerr << "Doesn't work" << endl;
				break;
			}	
		}

		//Deletes the pinball, executed when the deathTrigger or plane is triggered.
		//This then creates a new pinball at the original transform and takes away
		//20 score points from the total score. 
		void ballDelete() {
			if (BallDelete == true)
			{
				px_scene->removeActor(*(PxActor*)ball->Get());
				BallDelete = false;
				score -= 100;
				spawnBall(pose);
				cerr << "Ball Delete" << endl;
				deaths++;
			}		
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}
	};
}
