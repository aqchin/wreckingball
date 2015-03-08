#pragma once
#pragma warning(disable : 4098)

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btDefaultSoftBodySolver.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

class Physics {
public:
  static btBroadphaseInterface* bp;
  static btDefaultCollisionConfiguration* dcc;
  static btCollisionDispatcher* cd;
  static btSequentialImpulseConstraintSolver* sics;
  static btSoftRigidDynamicsWorld* ddw;
  static btDefaultSoftBodySolver* dsbs;

  static btSphereShape* ball;
  static btRigidBody** ballrb;
  static btCollisionShape* floor;
  static btBoxShape* brick;
  static btBoxShape* anchor;
  static btBoxShape* rope;
  static btRigidBody** roperb;
  static btRigidBody** brickrb;

  static void init();
  static void deinit();
};