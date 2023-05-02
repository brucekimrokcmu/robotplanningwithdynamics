#include <iostream>
#include <btBulletDynamicsCommon.h>
#include "bullet/LinearMath/btVector3.h"


int main(){
    // Create a dynamics world with default configuration
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    // Create a rigid body with initial position and velocity
    btVector3 initialPosition(0, 0, 0);
    btVector3 initialVelocity(u.x(), u.y(), u.z());
    btTransform initialTransform;
    initialTransform.setIdentity();
    initialTransform.setOrigin(initialPosition);
    btDefaultMotionState* motionState = new btDefaultMotionState(initialTransform);
    btScalar mass = 1;
    btVector3 localInertia(0, 0, 0);
    btCollisionShape* collisionShape = new btBoxShape(btVector3(1, 1, 1));
    collisionShape->calculateLocalInertia(mass, localInertia);
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, collisionShape, localInertia);
    btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);
    rigidBody->setLinearVelocity(initialVelocity);

    // Add the rigid body to the dynamics world
    dynamicsWorld->addRigidBody(rigidBody);

    // Step the simulation for a time step dt
    btScalar dt = 1.0 / 60.0;  // 60 frames per second
    dynamicsWorld->stepSimulation(dt);

    // Get the new state of the rigid body
    btTransform newTransform;
    rigidBody->getMotionState()->getWorldTransform(newTransform);
    btVector3 newPosition = newTransform.getOrigin();
    btQuaternion newOrientation = newTransform.getRotation();
}