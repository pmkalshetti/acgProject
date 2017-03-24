#ifndef CALLBACK_H
#define CALLBACK_H

#include "Application.h"

// global pointer to application object
static Application * globalApp;

// register callbacks
static void keyboardCallback(unsigned char key, int x, int y)
{
  globalApp->keyboard(key, x, y);
}

static void specialCallback(int key, int x, int y)
{
  globalApp->special(key, x, y);
}

static void idleCallback()
{
  globalApp->idle();
}

static void displayCallback()
{
  globalApp->display();
}

static void reshapeCallback(int w, int h)
{
  globalApp->reshape(w, h);
}


/*
  defines application parameters
*/
void initApp(int argc, char ** argv, Application & app)
{
  // store app object
  globalApp = &app;

  // initialize window
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
  glutInitWindowPosition(0,0);
  glutInitWindowSize(app.getWindowWidth(), app.getWindowHeight());
  glutCreateWindow("CS 775: Rigid Body Motion");
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

  globalApp->initializeScene();

  // set static callbacks
  glutKeyboardFunc(keyboardCallback);
  glutSpecialFunc(specialCallback);
  glutIdleFunc(idleCallback);
  glutDisplayFunc(displayCallback);
  glutReshapeFunc(reshapeCallback);

  globalApp->idle();

  glutMainLoop();
}

#endif
