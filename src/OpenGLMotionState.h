#ifndef OPENGL_MOTION_STATE_H
#define OPENGL_MOTION_STATE_H

class OpenGLMotionState : public btDefaultMotionState {
public:
  OpenGLMotionState(const btTransform & t) : btDefaultMotionState(t) {}

  void getOpenGLWorldTransform(btScalar * tOutput)
  {
    btTransform tInput;
    getWorldTransform(tInput);
    tInput.getOpenGLMatrix(tOutput);
  }
};

#endif
