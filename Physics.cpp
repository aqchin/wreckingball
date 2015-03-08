/*
* Credit for material from bullet engine physics tutorial to
*   THECPLUSPLUSGUY @ https://www.youtube.com/watch?v=d7_lJJ_j2NE
*/
#include "Physics.h"

btBroadphaseInterface* Physics::bp;
btDefaultCollisionConfiguration* Physics::dcc;
btCollisionDispatcher* Physics::cd;
btSequentialImpulseConstraintSolver* Physics::sics;
btSoftRigidDynamicsWorld* Physics::ddw;
btDefaultSoftBodySolver* Physics::dsbs;
btSphereShape* Physics::ball;
btRigidBody** Physics::ballrb = new btRigidBody*[1];
btRigidBody** Physics::brickrb = new btRigidBody*[120];
btCollisionShape* Physics::floor;
btBoxShape* Physics::brick;
btBoxShape* Physics::anchor;
btBoxShape* Physics::rope;
btRigidBody** Physics::roperb = new btRigidBody*[4];

void Physics::init() {
  bp = new btDbvtBroadphase();
  dcc = new btSoftBodyRigidBodyCollisionConfiguration();
  cd = new btCollisionDispatcher(dcc);
  sics = new btSequentialImpulseConstraintSolver();
  dsbs = new btDefaultSoftBodySolver();
  ddw = new btSoftRigidDynamicsWorld(cd, bp, sics, dcc, dsbs);
  ddw->setGravity(btVector3(0, -10, 0));

  floor = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
  ball = new btSphereShape(10);
  brick = new btBoxShape(btVector3(10, 8, 4));
  anchor = new btBoxShape(btVector3(1, 1, 1));
  rope = new btBoxShape(btVector3(0.5f, 5.0f, 0.5f));
  
  // Floor
  btDefaultMotionState* floorms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), 
    btVector3(0, -1, 0)));
  btRigidBody::btRigidBodyConstructionInfo
    floorrbci(0, floorms, floor, btVector3(0, 0, 0));
  floorrbci.m_restitution = 1.0f;
  floorrbci.m_friction = 0.5f;
  btRigidBody* floorrb = new btRigidBody(floorrbci);
  ddw->addRigidBody(floorrb);
  
  // Ball
  btDefaultMotionState* ballms =
    new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
  btScalar mass = 1;
  btVector3 balli(0, 0, 0);
  ball->calculateLocalInertia(mass, balli);
  btRigidBody::btRigidBodyConstructionInfo ballrbci(mass, ballms, ball, balli);
  ballrb[0] = new btRigidBody(ballrbci);
  ddw->addRigidBody(ballrb[0]);

  // Wall of bricks
  brick->setMargin(0.001f);
  float x = -40.0;
  float y = 8.0;
  float z = -50.0;
  btScalar brickm = 0.1;
  btVector3 bricki(0, 0, 0);
  brick->calculateLocalInertia(brickm, bricki);
  for (int r = 0; r < 8; r++) {
    y = (float)(8.0 + 16.0 * r);
    for (int c = 0; c < 15; c++) {
      x = (float)(-140.0 + 20.0 * c);
      btDefaultMotionState* brickms = 
        new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(x, y, z)));
      btRigidBody::btRigidBodyConstructionInfo brickrbci(brickm, brickms, brick, bricki);
      brickrb[c + 15*r] = new btRigidBody(brickrbci);
      ddw->addRigidBody(brickrb[c+15*r]);
    }
  }
  
  // String
  btSoftBody* sb = btSoftBodyHelpers::CreateRope(ddw->getWorldInfo(),
    btVector3(0, 150, 0), btVector3(0, 60, 0), 10, 1);
  sb->m_cfg.viterations = 20;
  sb->m_cfg.piterations = 20;
  sb->setTotalMass(.75);
  sb->appendAnchor(sb->m_nodes.size() - 1, ballrb[0]);
  ddw->addSoftBody(sb);

  /*
  // Anchor
  btScalar anchorm = 0;
  btVector3 anchori(0, 0, 0);
  btDefaultMotionState* anchorms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),
    btVector3(0, 100 + 1, 0)));
  btRigidBody::btRigidBodyConstructionInfo
    anchorrbci(0, anchorms, anchor, btVector3(0, 0, 0));
  btRigidBody* anchorrb = new btRigidBody(anchorrbci);
  ddw->addRigidBody(anchorrb);

  // Rope
  btScalar ropem = 1;
  btVector3 ropei(0, 0, 0);
  rope->calculateLocalInertia(ropem, ropei);
  for(int i = 0; i < 4; i++) {
    btDefaultMotionState* ropems =
      new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 95 - 10*i, 0)));
    btRigidBody::btRigidBodyConstructionInfo roperbci(ropem, ropems, rope, ropei);
    roperb[i] = new btRigidBody(roperbci);
    ddw->addRigidBody(roperb[i]);
  }

   btTransform fia, fib, fia1, fib1, fia2, fib2, fia3, fib3, fia4, fib4;
   fia = btTransform::getIdentity();
   fib = btTransform::getIdentity();
   fia.setOrigin(btVector3(0, 100, 0)); // anchor 0
   fib.setOrigin(btVector3(0, 100, 0));

   fia1 = btTransform::getIdentity();
   fib1 = btTransform::getIdentity();
   fia1.setOrigin(btVector3(0, 90, 0)); // joint 1
   fib1.setOrigin(btVector3(0, 90, 0));

   fia2 = btTransform::getIdentity();
   fib2 = btTransform::getIdentity();
   fia2.setOrigin(btVector3(0, 80, 0)); // joint 2
   fib2.setOrigin(btVector3(0, 80, 0));

   fia3 = btTransform::getIdentity();
   fib3 = btTransform::getIdentity();
   fia3.setOrigin(btVector3(0, 70, 0)); // joint 3
   fib3.setOrigin(btVector3(0, 70, 0));

   fia4 = btTransform::getIdentity();
   fib4 = btTransform::getIdentity();
   fia4.setOrigin(btVector3(0, 60, 0)); // joint 4
   fib4.setOrigin(btVector3(0, 60, 0));

   btGeneric6DofConstraint* gdc = new btGeneric6DofConstraint(*anchorrb, *roperb[0], fia, fib, false);
   btGeneric6DofConstraint* gdc1 = new btGeneric6DofConstraint(*roperb[0], *roperb[1], fia1, fib1, false);
   btGeneric6DofConstraint* gdc2 = new btGeneric6DofConstraint(*roperb[1], *roperb[2], fia2, fib2, false);
   btGeneric6DofConstraint* gdc3 = new btGeneric6DofConstraint(*roperb[2], *roperb[3], fia3, fib3, false);
   btGeneric6DofConstraint* gdc4 = new btGeneric6DofConstraint(*roperb[3], *ballrb[0], fia4, fib4, false);

   gdc->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc1->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc1->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc2->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc2->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc3->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc3->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc4->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc4->setAngularUpperLimit(btVector3(1.5, 0, 0));

   ddw->addConstraint(gdc);
   ddw->addConstraint(gdc1);
   ddw->addConstraint(gdc2);
   ddw->addConstraint(gdc3);
   ddw->addConstraint(gdc4);
   */
}

void Physics::deinit() {}