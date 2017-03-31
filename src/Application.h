#ifndef APPLICATION_H
#define APPLICATION_H

#include "GL/freeglut.h"
#include "bullet/btBulletDynamicsCommon.h"
#include "Object.h"
#include <iostream>
#include <vector>
#include <set>
#include <iterator>
#include <algorithm>
using namespace std;

typedef vector<Object *> Objects;
typedef vector<btGeneric6DofConstraint *> Constraints;
typedef pair<const btRigidBody *, const btRigidBody *> CollisionPair;
typedef set<CollisionPair> CollisionPairs;

class Application {

  /*
    >----------------- Rendering Section --------------------<
  */

  // window size
  int windowHeight = 768;
  int windowWidth = 1024;

  // camera parameters in scene
  btVector3 cameraPosition;
  btVector3 cameraLookAt;
  btVector3 cameraUp;
  float nearPlane;
  float farPlane;
  float cameraDistance;
  float cameraPitch;
  float cameraYaw;

  // camera controls
  void updateCamera();
  void rotateCamera(float & angle, float value);
  void zoomCamera(float distance);

  // draw objects
  void drawBox(btVector3 & halfSize);
  void drawShape(btScalar * transform, btCollisionShape * shape, const btVector3 & color);

  // render scene
  void renderScene();

  void updateScene(float t);

public:
  // constructor
  Application();

  // inititalize application - lighting and objects
  void initializeScene();

  // accessor functions
  int getWindowHeight() {return windowHeight;}
  int getWindowWidth() {return windowWidth;}

  // callbacks
  void keyboard(unsigned char key, int x, int y);
  void special(int key, int x, int y);
  void idle();
  void display() {}
  void reshape(int w, int h);

  /*
    >----------------- Dynamics Section --------------------<
  */
private:
  // Bullet Components
  btBroadphaseInterface * broadPhase;
  btCollisionConfiguration * collisionConfiguration;
  btCollisionDispatcher * dispatcher;
  btConstraintSolver * solver;
  btDynamicsWorld * world;

  // Physics Engine components
  btClock clock;
  Objects objects;
  Constraints constraints;
  CollisionPairs collisionPairs;

public:
  // initialize physics engine
  void initializePhysics();

  // create objects for simulation
  void createObjects();

  // create object in physics world
  Object * createObject(btCollisionShape * shape, const float & mass,
                    const btVector3 & color = btVector3(1,1,1),
                    const btVector3 & initialPosition = btVector3(0,0,0),
                    const btQuaternion & initialRotation = btQuaternion(0,0,1,1));

  // add constraint
  void addConstraint(Object * object, btVector3 position);

  // update constraint
  void updateConstraint(float t);

  // enforce constraint
  void enforceConstraint();

  // collision events
  void checkCollisionEvents();
  void collisionEvent(btRigidBody * body0, btRigidBody * body1);
  void separationEvent(btRigidBody * body0, btRigidBody * body1);

};


#endif
