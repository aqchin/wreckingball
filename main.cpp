/*
 * Credit for bullet engine physics tutorial to 
 *   THECPLUSPLUSGUY @ https://www.youtube.com/watch?v=d7_lJJ_j2NE
 */
#include "main.h"

static int width = 800;
static int height = 600;

static bool m1_down = false;
static double rot_scale = 120.2;

Matrix4 m2w = *new Matrix4();
Vector3 v_prev, v_cur;
Physics physics;
btSoftBody* sb;
btRigidBody* rb;

btVector3 pos;
btScalar dis;
btGeneric6DofConstraint* con;

Camera* cam =
  new Camera(Vector3(0.0, 100.0, 150.0), Vector3(0.0, 50.0, 0.0), Vector3(0.0, 1.0, 0.0));

void reshape(int w, int h) {
  width = w;
  height = h;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (float)width / height, 1.0, 1000.0);
  glTranslatef(0.0, 0.0, -20.0);
  glMatrixMode(GL_MODELVIEW);
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);

  //Matrix4 glmatrix = m2w;
  glLoadIdentity();
  glLoadMatrixd(cam->getGLMatrix());

  // Plane
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_QUADS);
  //glNormal3f(0.0f, 1.0f, 0.0f);
  glVertex3f(-1000.0f, -0.01f, 1000.0f);
  glVertex3f(1000.0f, -0.01f, 1000.0f);
  glVertex3f(1000.0f, -0.01f, -1000.0f);
  glVertex3f(-1000.0f, -0.01f, -1000.0f);
  glEnd();

  // Ball
  float m[16];
  btTransform t;
  Physics::ballrb[0]->getMotionState()->getWorldTransform(t);
  t.getOpenGLMatrix(m);
  glPushMatrix();
  glMultMatrixf(m);
  glColor3f(0.0f, 0.0f, 1.0f);
  glutSolidSphere(Physics::ball->getRadius(), 30, 30);
  glPopMatrix();

  // Wall of bricks
  for(int i = 0; i < 120; i++) {
    Physics::brickrb[i]->getMotionState()->getWorldTransform(t);
    t.getOpenGLMatrix(m);
    glPushMatrix();
    glMultMatrixf(m);
    glColor3f(i % 20, i % 5, i % 10);
    glScalef(1.0,(float)16/20, (float)8/20);
    glutSolidCube(20);
    glPopMatrix();
  }

  // String
  for(int i = 0; i < Physics::ddw->getSoftBodyArray().size(); i++) {
    sb = Physics::ddw->getSoftBodyArray()[i];
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_TRIANGLES);
    for (int j = 0; j < sb->m_faces.size(); j++) {
      for(int k = 0; k < 3; k++) {
        glVertex3f(sb->m_faces[j].m_n[k]->m_x.x(), sb->m_faces[j].m_n[k]->m_x.y(),
          sb->m_faces[j].m_n[k]->m_x.z());
      }
    }
    glEnd();

    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    for(int j = 0; j < sb->m_links.size(); j++) {
      for(int k = 0; k < 2; k++) {
        glVertex3f(sb->m_links[j].m_n[k]->m_x.x(), sb->m_links[j].m_n[k]->m_x.y(),
          sb->m_links[j].m_n[k]->m_x.z());
      }
    }
    glEnd();
  }

  glFlush();
  glutSwapBuffers();
}

Vector3 raytrace(int x, int y) {
  int vp[4];
  double mm[16];
  double pm[16];
  float winx, winy, winz;
  double dx, dy, dz;

  glGetDoublev(GL_MODELVIEW_MATRIX, mm);
  glGetDoublev(GL_PROJECTION_MATRIX, pm);
  glGetIntegerv(GL_VIEWPORT, vp);

  winx = (float) x;
  winy = (float)vp[3] - (float)y;
  glReadPixels(x, int(winy), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);

  gluUnProject(winx, winy, winz, mm, pm, vp, &dx, &dy, &dz);
  //cout << dx << " " << dy << " " << dz << endl;

  return Vector3(dx, dy, dz);
}

void mouse(int k, int s, int x, int y) {
  m1_down = (s == GLUT_DOWN);
  if (k == GLUT_LEFT_BUTTON && s == GLUT_DOWN) {
    v_prev = raytrace(x, y);
    btVector3 from(cam->e.x, cam->e.y, cam->e.z);
    Vector3 v = (v_prev - cam->e);
    v.scale(10000);
    //cout << v.x << " " << v.y << " " << v.z << endl;
    //cout << cam->e.x << " " << cam->e.y << " " << cam->e.z << endl;

    btCollisionWorld::ClosestRayResultCallback rc(btVector3(cam->e.x, cam->e.y, cam->e.z),
      btVector3(v.x, v.y, v.z));
    Physics::ddw->rayTest(btVector3(cam->e.x, cam->e.y, cam->e.z),
      btVector3(v.x, v.y, v.z), rc);

    //cout << rc.hasHit() << endl;
    if(rc.hasHit()) {
      //const btRigidBody* rd = (btRigidBody::upcast)(rc.m_collisionObject);
      //btRigidBody re = *rd;
      //btRigidBody* rb = (btRigidBody*)(rc.m_collisionObject);//&re;
      rb = (btRigidBody*)(rc.m_collisionObject);

      //cout << !(rb->isStaticObject() || rb->isKinematicObject()) << endl;
      if (!(rb->isStaticObject() || rb->isKinematicObject()))
      {
        pos = rc.m_hitPointWorld;
        rb->setActivationState(DISABLE_DEACTIVATION);

        btVector3 lpiv = rb->getCenterOfMassTransform().inverse() * pos;

        btTransform t;
        t.setIdentity();
        t.setOrigin(lpiv);
        btGeneric6DofConstraint* n_con = new btGeneric6DofConstraint(*rb, t, false);
        n_con->setLinearLowerLimit(btVector3(0, 0, 0));
        n_con->setLinearUpperLimit(btVector3(0, 0, 0));
        n_con->setAngularLowerLimit(btVector3(0, 0, 0));
        n_con->setAngularUpperLimit(btVector3(0, 0, 0));

        Physics::ddw->addConstraint(n_con);
        con = n_con;

        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 0);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 1);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 2);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 3);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 4);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 5);

        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 0);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 1);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 2);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 3);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 4);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 5);

        dis = (pos - from).length();
      }
    }
  }
  else {
    if(con != NULL) {
      rb->forceActivationState(ACTIVE_TAG);
      rb->setDeactivationTime(0.0f);
      rb = NULL;

      Physics::ddw->removeConstraint(con);
      delete con;
      con = NULL;
    }
  }
}

void motion(int x, int y) {
  v_cur = raytrace(x,y);
  if (m1_down) {
    Vector3 dir;
    btGeneric6DofConstraint* n_con = static_cast<btGeneric6DofConstraint*>(con);
    if(n_con) {
      btVector3 to = btVector3(v_cur.x, v_cur.y, v_cur.z);
      btVector3 from = btVector3(cam->e.x, cam->e.y, cam->e.z);
      btVector3 piv = n_con->getFrameOffsetA().getOrigin();
      btVector3 n_piv;
      btVector3 v = to - from;
      v.normalize();
      v *= dis;
      n_piv = from + v;
      n_con->getFrameOffsetA().setOrigin(n_piv);
    }
  }
}

void keyboard(unsigned char k, int, int) {
  switch(k) {
  case 'w':
    cam->move('w');
    break;
  case 'a':
    cam->move('a');
    break;
  case 's':
    cam->move('s');
    break;
  case 'd':
    cam->move('d');
    break;
  case ' ':
    delete cam;
    cam = new Camera(Vector3(0.0, 100.0, 200.0), Vector3(0.0, 50.0, 0.0), Vector3(0.0, 1.0, 0.0));
    physics.init();
    break;
  }
}

void idle() {
  Physics::ddw->stepSimulation(1 / 60.f, 10);
  display();
}

int main(int argc, char* argv[]) {
  float specular[]  = {1.0, 1.0, 1.0, 1.0};
  float shininess[] = {100.0};
  float position[] = { 0.0, 10.0, 1.0, 0.0 };  // lightsource position

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(width, height);
  glutCreateWindow("Wrecking Ball");

  glEnable(GL_DEPTH_TEST);
  glClear(GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glCullFace(GL_BACK);
  glShadeModel(GL_SMOOTH);
  glMatrixMode(GL_PROJECTION);

  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_NORMALIZE);

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  physics.init();

  glutMainLoop();

  delete Physics::ball;
  delete Physics::ballrb;
  delete Physics::floor;
  delete Physics::brick;
  delete Physics::anchor;
  delete Physics::rope;
  delete Physics::roperb;
  delete Physics::brickrb;
  delete Physics::dsbs;
  delete Physics::sics;
  delete Physics::cd;
  delete Physics::dcc;
  delete Physics::bp;
  delete Physics::ddw;

  return 0;
}