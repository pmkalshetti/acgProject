#ifndef OBJECT_H
#define OBJECT_H

#include "bullet/btBulletDynamicsCommon.h"
#include "bullet/btBulletCollisionCommon.h"
#include "OpenGLMotionState.h"
#include <cmath>

class Object {
  btVector3 color;
  btCollisionShape * shape;
  btRigidBody * body;
  OpenGLMotionState * state;

public:
  Object(btCollisionShape * s, const float & mass, const btVector3 & c,
         const btVector3 & initialPosition = btVector3(0,0,0),
         const btQuaternion & initialRotation = btQuaternion(0,0,1,1))
  {
    shape = s;
    color = c;

    // set initial transform
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(initialPosition);
    transform.setRotation(initialRotation);
    state = new OpenGLMotionState(transform);

    btVector3 inertia(0,0,0);
    if (abs(mass) < 0.0001) {
      shape->calculateLocalInertia(mass, inertia);
    }

    btRigidBody::btRigidBodyConstructionInfo info(mass, state, shape, inertia);
    body = new btRigidBody(info);
  }

  // accessor functions
  btCollisionShape * getShape() { return shape; }
  btRigidBody * getRigidBody() { return body; }
  btMotionState * getMotionState() { return state; }
  btVector3 getColor() { return color; }

  // modifier function
  void setColor(const btVector3 & c) { color = c; }

  // get transformation matrix
  void getTransform(btScalar * tOutput)
  {
    if(state != nullptr) {
      state->getOpenGLWorldTransform(tOutput);
    }
  }

  ~Object()
  {
    delete body;
    delete state;
    delete shape;
  }

};

#endif
