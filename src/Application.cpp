#include "Application.h"
#define RADIANS_PER_DEGREE 0.01745329f
#define CAMERA_STEP_SIZE 5.0f

/*
  >-----------constructor --------------<
*/
Application::Application() :
  cameraPosition(10, 5, 0),
  cameraLookAt(0, 10, 0),
  cameraDistance(75),
  cameraPitch(0),
  cameraYaw(0),
  cameraUp(0,1,0),
  nearPlane(1),
  farPlane(1000),
  broadPhase(0),
  collisionConfiguration(0),
  dispatcher(0),
  solver(0),
  world(0)
  {}

/*
  >------ initialize lighting and objects in scene ------<
*/
void
Application::initializeScene(int argc, char ** argv)
{
  // light parameters
  GLfloat ambient[] = {0.2, 0.2, 0.2, 1};
  GLfloat diffuse[] = {1,1,1,1};
  GLfloat specular[] = {1,1,1,1};
  GLfloat position[] = {5,10,1,0};

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);

  glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMateriali(GL_FRONT, GL_SHININESS, 10);

  glShadeModel(GL_SMOOTH);

  glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

  glClearColor(0.6, 0.65, 0.85, 0);

  initializePhysics(argc, argv);
}

/*
  >---------- callbacks ---------------<
*/
void
Application::keyboard(unsigned char key, int x, int y)
{
  switch(key)
  {
    case 'q':
    case 27:
      exit(0);
  }
}

void
Application::special(int key, int x, int y)
{
  switch(key)
  {
    case GLUT_KEY_PAGE_UP:
      zoomCamera(CAMERA_STEP_SIZE);
      break;
    case GLUT_KEY_PAGE_DOWN:
      zoomCamera(-CAMERA_STEP_SIZE);
      break;
    case GLUT_KEY_LEFT:
      rotateCamera(cameraYaw, CAMERA_STEP_SIZE);
      break;
    case GLUT_KEY_RIGHT:
      rotateCamera(cameraYaw, -CAMERA_STEP_SIZE);
      break;
    case GLUT_KEY_UP:
      rotateCamera(cameraPitch, CAMERA_STEP_SIZE);
      break;
    case GLUT_KEY_DOWN:
      rotateCamera(cameraPitch, -CAMERA_STEP_SIZE);
      break;
  }
}

void
Application::idle()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  float t = clock.getTimeMilliseconds();
  //clock.reset();
  updateScene(t/1000.0);

  updateCamera();

  renderScene();

  glutSwapBuffers();
}

void
Application::reshape(int w, int h)
{
  windowWidth = w;
  windowHeight = h;

  glViewport(0,0,w,h);
  updateCamera();
}

/*
  >--------- camera controls ------------------<
*/
void
Application::updateCamera()
{
  //> set projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float aspectRatio = windowWidth / (float) windowHeight;
  glFrustum(-aspectRatio * nearPlane, aspectRatio * nearPlane,
            -nearPlane, nearPlane, nearPlane, farPlane);

  //> set view matrix
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  float pitch = cameraPitch * RADIANS_PER_DEGREE;
  float yaw = cameraYaw * RADIANS_PER_DEGREE;
  btQuaternion rotationYaw(cameraUp, yaw);
  btVector3 newCameraPosition(0,0,0);
  newCameraPosition[2] = -cameraDistance;
  btVector3 front(newCameraPosition[0], newCameraPosition[1], newCameraPosition[2]);
  if(front.length2() < SIMD_EPSILON) {
    front.setValue(1.0, 0.0, 0.0);
  }
  btVector3 right = cameraUp.cross(front);
  btQuaternion rotationPitch(right, -pitch);
  cameraPosition = btMatrix3x3(rotationYaw) * btMatrix3x3(rotationPitch) * newCameraPosition;
  cameraPosition += cameraLookAt;
  gluLookAt(cameraPosition[0], cameraPosition[1], cameraPosition[2],
            cameraLookAt[0],cameraLookAt[1], cameraLookAt[2],
            cameraUp[0], cameraUp[1], cameraUp[2]);
}

void
Application::rotateCamera(float & angle, float value)
{
  angle -= value;

  // bound check
  if(angle < 0)
    angle += 360;
  if(angle >= 360)
    angle -= 360;

  updateCamera();
}

void
Application::zoomCamera(float distance)
{
  cameraDistance -= distance;

  // bound check
  if(cameraDistance <0.1)
    cameraDistance = 0.1;

  updateCamera();
}

/*
  >------------ drawing functions ---------------<
*/
void
Application::drawBox(btVector3 & halfSize)
{
  // vertex positions
  btVector3 vertices[8] = {btVector3(halfSize[0], halfSize[1], halfSize[2]),
                           btVector3(-halfSize[0], halfSize[1], halfSize[2]),
                           btVector3(halfSize[0], -halfSize[1], halfSize[2]),
                           btVector3(-halfSize[0], -halfSize[1], halfSize[2]),
                           btVector3(halfSize[0], halfSize[1], -halfSize[2]),
                           btVector3(-halfSize[0], halfSize[1], -halfSize[2]),
                           btVector3(halfSize[0], -halfSize[1], -halfSize[2]),
                           btVector3(-halfSize[0], -halfSize[1], -halfSize[2])};
  // indices for using in GL_TRIANGLES
  static int indices[36] = {0, 1, 2,
                            3, 2, 1,
                            4, 0, 6,
                            6, 0, 2,
                            5, 1, 4,
                            4, 1, 0,
                            7, 3, 1,
                            7, 1, 5,
                            5, 4, 7,
                            7, 4, 6,
                            7, 2, 3,
                            7, 6, 2};
  glBegin(GL_TRIANGLES);
  for (int i = 0; i < 36; i+=3) {
    const btVector3 & vertex1 = vertices[indices[i]];
    const btVector3 & vertex2 = vertices[indices[i+1]];
    const btVector3 & vertex3 = vertices[indices[i+2]];

    btVector3 normal = (vertex3 - vertex1).cross(vertex2 - vertex1);
    normal.normalize();

    glNormal3f(normal.getX(), normal.getY(), normal.getZ());

    glVertex3f(vertex1.getX(), vertex1.getY(), vertex1.getZ());
    glVertex3f(vertex2.getX(), vertex2.getY(), vertex2.getZ());
    glVertex3f(vertex3.getX(), vertex3.getY(), vertex3.getZ());
  }
  glEnd();
}

void
Application::drawShape(btScalar * transform, btCollisionShape * shape, const btVector3 & color)
{
  glColor3f(color[0], color[1], color[2]);
  glPushMatrix();
  glMultMatrixf(transform);
  const btBoxShape * box = static_cast<const btBoxShape *>(shape);
  btVector3 halfSize = box->getHalfExtentsWithMargin();
  drawBox(halfSize);
  glPopMatrix();
}

/*
  >---------- render functions --------------<
*/
void
Application::renderScene()
{
  btScalar transform[16];

  for(auto x : objects) {
    x->getTransform(transform);
    drawShape(transform, x->getShape(), x->getColor());
  }
}

void
Application::updateScene(float t)
{
  if(world) {
    //updateConstraint(t);
    world->stepSimulation(t);
    checkCollisionEvents();
  }
}

/*
  >-------------- initialize Physics Engine ----------------<
*/
void
Application::initializePhysics(int argc, char ** argv)
{
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  broadPhase = new btDbvtBroadphase();
  solver = new btSequentialImpulseConstraintSolver();
  world = new btDiscreteDynamicsWorld(dispatcher, broadPhase, solver, collisionConfiguration);
  
  btVector3 obj1Pos(argv[1], argv[2], argv[3]);
  btVector3 obj2Pos(argv[4], argv[5], argv[6]);
	
  createObjects(obj1Pos, obj2Pos);
  enforceConstraint();
}

/*
  >------------- Create Objects for Simulation ----------------<
*/
void
Application::createObjects(const btVector3 & obj1Pos, const btVector3 & obj2Pos)
{
  // ground plane
  createObject(new btBoxShape(btVector3(1, 50, 50)), 0,
                   btVector3(0.2, 0.6, 0.6), btVector3(0,0,0));

  // object 1
  createObject(new btBoxShape(btVector3(1,1,1)), 1,
               btVector3(1, 0.2, 0.2), obj1Pos);

  // object 2
  createObject(new btBoxShape(btVector3(1,1,1)), 1,
               btVector3(0, 0.2, 0.8), obj2Pos);


}

/*
  >------------- Create Object in physics world ----------------<
*/
Object *
Application::createObject(btCollisionShape * shape, const float & mass,
                  const btVector3 & color,
                  const btVector3 & initialPosition,
                  const btQuaternion & initialRotation)
{
  Object * object = new Object(shape, mass, color, initialPosition, initialRotation);
  objects.push_back(object);

  if(world) {
    world->addRigidBody(object->getRigidBody());
  }
  return object;
}

/*
  >----------- Add constraint on a object at a position -------<
*/
void
Application::addConstraint(Object * object, btVector3 position)
{
  if(!world) return;
  btRigidBody * rigidBody = object->getRigidBody();
  rigidBody->setActivationState(DISABLE_DEACTIVATION);

  // get position of constraint in local coordinate frame
  btVector3 localPosition = rigidBody->getCenterOfMassTransform().inverse() * position;
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(localPosition);

  // setup constraint
  btGeneric6DofConstraint * constraint = new btGeneric6DofConstraint(*rigidBody, transform, true);
  world->addConstraint(constraint, true);

  // define constraint parameters
  float strength = 0;
  constraint->setParam(BT_CONSTRAINT_STOP_CFM, strength, 0);
  constraint->setParam(BT_CONSTRAINT_STOP_CFM, strength, 1);
  constraint->setParam(BT_CONSTRAINT_STOP_CFM, strength, 2);
  constraint->setParam(BT_CONSTRAINT_STOP_CFM, strength, 3);
  constraint->setParam(BT_CONSTRAINT_STOP_CFM, strength, 4);
  constraint->setParam(BT_CONSTRAINT_STOP_CFM, strength, 5);

  float erp = 0;
  constraint->setParam(BT_CONSTRAINT_STOP_ERP, erp, 0);
  constraint->setParam(BT_CONSTRAINT_STOP_ERP, erp, 1);
  constraint->setParam(BT_CONSTRAINT_STOP_ERP, erp, 2);
  constraint->setParam(BT_CONSTRAINT_STOP_ERP, erp, 3);
  constraint->setParam(BT_CONSTRAINT_STOP_ERP, erp, 4);
  constraint->setParam(BT_CONSTRAINT_STOP_ERP, erp, 5);

  constraints.push_back(constraint);
}

void
Application::updateConstraint(float t)
{
  // if(t > 2.5) {
    //  constraints[0]->getFrameOffsetA().setOrigin(objects[1]->getRigidBody()->getCenterOfMassPosition() + btVector3(0,10*t,0));
  // }
}

void
Application::enforceConstraint()
{
  //addConstraint(objects[1], objects[1]->getRigidBody()->getCenterOfMassPosition());
  objects[1]->getRigidBody()->setLinearVelocity(btVector3(20, 30, 0));
  objects[2]->getRigidBody()->setLinearVelocity(btVector3(-10, 30, 0));
  objects[1]->getRigidBody()->setAngularVelocity(btVector3(0, 0, -1));
  objects[2]->getRigidBody()->setAngularVelocity(btVector3(0, 0, 1));
}

/*
  >-------- Collision Events ---------<
*/
void
Application::checkCollisionEvents()
{
  CollisionPairs currentPairs;

  // check for all manifolds
  for(int i = 0; i < dispatcher->getNumManifolds(); i++) {
    btPersistentManifold * manifold = dispatcher->getManifoldByIndexInternal(i);

    // ignore objects with no contact points
    if(manifold->getNumContacts() <= 0){
      continue;
    }

    // bodies involved in collision
    const btRigidBody * body0 = static_cast<const btRigidBody *>(manifold->getBody0());
    const btRigidBody * body1 = static_cast<const btRigidBody *>(manifold->getBody1());

    // sort them
    bool const swappedFlag = body0 > body1;
    const btRigidBody * sortedBody0 = swappedFlag ? body1 : body0;
    const btRigidBody * sortedBody1 = swappedFlag ? body0 : body1;

    // insert in set
    CollisionPair currentPair = make_pair(sortedBody0, sortedBody1);
    currentPairs.insert(currentPair);

    if(collisionPairs.find(currentPair) == collisionPairs.end()) {
      collisionEvent((btRigidBody *) body0, (btRigidBody *) body1);
    }
  }

  // separation event
  CollisionPairs removedPairs;

  set_difference(collisionPairs.begin(), collisionPairs.end(),
                 currentPairs.begin(), currentPairs.end(),
                 inserter(removedPairs, removedPairs.begin()));

  for(CollisionPairs::const_iterator itr = removedPairs.begin();
      itr != removedPairs.end(); itr++) {
    separationEvent((btRigidBody *)itr->first, (btRigidBody *)itr->second);
  }

  collisionPairs = currentPairs;
}

void
Application::collisionEvent(btRigidBody * body0, btRigidBody * body1)
{
  //btVector3 linearVelocity0 = body0->getLinearVelocity();
  //btVector3 linearVelocity1 = body1->getLinearVelocity();

  //body0->applyCentralForce(btVector3(-30, 10, 0));
  //body1->applyCentralForce(btVector3(30, 10, 0));

  //cout << "LinearVelocity0: " << linearVelocity0[0] << " " << linearVelocity0[1] << " " << linearVelocity0[2] << endl;
  //cout << "LinearVelocity1: " << linearVelocity1[0] << " " << linearVelocity1[1] << " " << linearVelocity1[2] << endl;
}

void
Application::separationEvent(btRigidBody * body0, btRigidBody * body1)
{
}
