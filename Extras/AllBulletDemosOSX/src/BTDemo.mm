/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#import "BTDemo.h"

///////////////////////////////////////////////////////////////////////
// Bullet includes

#include "LinearMath/btScalar.h"
#include "LinearMath/btMinMax.h"

#include "DemoApplication.h"
#include "DemoEntries.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "GLDebugDrawer.h"

#include "LinearMath/btQuickprof.h"


///////////////////////////////////////////////////////////////////////

#pragma mark -
#pragma mark Globals

static NSPoint MousePosition;
static NSSize  ViewSize;
static int     NumIterations = 10;
static BOOL	   DisableDeactivation = NO;
static BOOL	   DrawAABBs = NO;
static BOOL	   DebugDraw = NO;
static BOOL	   SplitImpulse = NO;
static BOOL	   DrawContacts = NO;

///////////////////////////////////////////////////////////////////////

#pragma mark -
#pragma mark BTDemoEntry

@interface BTDemoEntry : NSObject {
	btDemoEntry *ctor;
	DemoApplication *demo;
}

- (id) initWithDemoEntry: (btDemoEntry*) constructor;
- (DemoApplication*) demo;
- (void) reset;

@end

@implementation BTDemoEntry

- (id) initWithDemoEntry: (btDemoEntry*) constructor
{
	if ( self = [super init] )
	{
		ctor = constructor;
		[self reset];
	}
	
	return self;
}

- (void) dealloc
{
	if ( demo ) delete demo;
	[super dealloc];
}

- (DemoApplication*) demo
{
	return demo;
}

- (void) reset
{
	if ( demo ) 
	{
		if (demo->getDynamicsWorld()->getDebugDrawer())
			delete demo->getDynamicsWorld()->getDebugDrawer();
		
		delete demo;
		demo = NULL;
	}

	demo = ctor->createFcn();
	btAssert(demo);

	if (demo->getDynamicsWorld())
	{
		demo->getDynamicsWorld()->setDebugDrawer(new GLDebugDrawer());
	}

	#ifndef BT_NO_PROFILE
		CProfileManager::Reset();
	#endif //BT_NO_PROFILE
}

@end

///////////////////////////////////////////////////////////////////////

#pragma mark -
#pragma mark BTDemo


@implementation BTDemo

+ (NSArray*) demoNames
{
	static NSMutableArray *DEMOS = nil;
	if ( !DEMOS )
	{
		DEMOS = [[NSMutableArray alloc] init];
	
		btDemoEntry* e = g_demoEntries;
		while (e->createFcn)
		{
			[DEMOS addObject: [NSString stringWithUTF8String: e->name]];
			++e;
		}
	}
		
	return DEMOS;
}

+ (BOOL) isDemo: (NSString *) demoName
{
	return [[self demoNames] containsObject: demoName];
}

+ (BTDemo*) demoWithName: (NSString *) demoName
{	
	if ( [BTDemo isDemo: demoName] )
	{
		return [[[BTDemo alloc] initWithDemoName: demoName] autorelease];
	}
	
	return nil;
}

- (id) initWithDemoName: (NSString *) demoName
{
	if ( self = [super init] )
	{
		_demoName = [demoName copy];
		
		// now walk the constructor list and find this demo
		
		btDemoEntry* e = g_demoEntries;
		while (e->createFcn)
		{
			NSString *name = [NSString stringWithCString: e->name];
			if ( [name isEqualToString: demoName] )
			{
				_demo = [[BTDemoEntry alloc] initWithDemoEntry: e];
				break;
			}
			
			e++;
		}		
	}

	return self;
}

- (void) dealloc
{
	[_demoName release];
	[_demo release];
	[super dealloc];
}

- (void) reset
{
	[_demo reset];

	[self contextWillResize];
	[self contextResized: ViewSize];
	[self contextDidResize];
}

- (NSString*) demoName
{
	return _demoName;
}

#pragma mark -
#pragma mark BTOpenGLDisplayDelegate

- (void) contextCreated
{}

- (void) contextWillBeDestroyed
{}

- (void) contextWillResize
{}

- (void) contextResized: (NSSize) newSize
{
	ViewSize = newSize;
	glViewport( 0, 0, (int) newSize.width, (int) newSize.height );

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	[_demo demo]->reshape( (int) newSize.width, (int) newSize.height );
}

- (void) contextDidResize
{}

- (void) contextStateInvalidated
{}

- (void) display: (float) deltaT
{
	DemoApplication *demo = [_demo demo];

	if ( !demo )
	{
		glClearColor( 1, 0.5, 1, 1 );
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		return;
	}

	if (demo->getDynamicsWorld())
	{
		if (SplitImpulse)
		{
			demo->getDynamicsWorld()->getSolverInfo().m_splitImpulse=1;
		} else
		{
			demo->getDynamicsWorld()->getSolverInfo().m_splitImpulse=0;
		}
	}
	if (DrawAABBs)
	{ 
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawAabb);
	} 
	else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawAabb));
	}
	
	if (DebugDraw)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawWireframe);
	} 
	else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawWireframe));		
	}
	
	if (DrawContacts)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawContactPoints);
	} 
	else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawContactPoints));
	}
	
	if (DisableDeactivation)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_NoDeactivation);
	} 
	else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_NoDeactivation));
	}
	
	if (demo->getDynamicsWorld() && demo->getDynamicsWorld()->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD)
	{
		btDiscreteDynamicsWorld* discreteWorld = (btDiscreteDynamicsWorld*) demo->getDynamicsWorld();
		discreteWorld->getSolverInfo().m_numIterations = NumIterations;
	}
		
	if (!demo->isIdle())
	{
		demo->clientMoveAndDisplay();
	}
	else
	{
		demo->displayCallback();
	}
	
}

- (void) keyPressed: (unsigned char) key
{
	[_demo demo]->keyboardCallback(key, (int)MousePosition.x, (int)MousePosition.y );
}

- (void) keyReleased: (unsigned char) key
{}

- (void) specialKeyPressed: (unsigned) key
{
	[_demo demo]->specialKeyboard(key,(int)MousePosition.x, (int)MousePosition.y );
}

- (void) specialKeyReleased: (unsigned) key
{
	[_demo demo]->specialKeyboardUp(key,(int)MousePosition.x, (int)MousePosition.y );
}

- (void) mouseButtonPressed: (unsigned) mouseButton
{
	[_demo demo]->mouseFunc( mouseButton, GLUT_DOWN, (int)MousePosition.x, (int)MousePosition.y );
}

- (void) mouseButtonReleased: (unsigned) mouseButton
{
	[_demo demo]->mouseFunc( mouseButton, GLUT_UP, (int)MousePosition.x, (int)MousePosition.y );
}

- (void) mouseMoved: (NSPoint) delta
{
	[_demo demo]->mouseMotionFunc((int)MousePosition.x, (int)MousePosition.y );
}

- (void) newMousePosition: (NSPoint) newMousePosition
{
	// Need to invert Y, since DemoApplication assumes origin at top-left
	MousePosition = NSMakePoint( newMousePosition.x, ViewSize.height - newMousePosition.y );
}

- (void) scrollWheel: (NSPoint) delta
{}

#pragma mark -
#pragma mark Global Simulation Properties

+ (void) setIterations: (unsigned) iterations
{
	NumIterations = iterations;
}

+ (unsigned) iterations
{
	return NumIterations;
}

+ (unsigned) minIterations { return 1; }
+ (unsigned) maxIterations { return 1000; }


+ (void) setDisableDeactivation: (BOOL) disableDeactivation
{
	DisableDeactivation = disableDeactivation;
}

+ (BOOL) disableDeactivation
{
	return DisableDeactivation;
}

+ (void) setDrawAABBs: (BOOL) drawAABBs
{
	DrawAABBs = drawAABBs;
}

+ (BOOL) drawAABBs
{
	return DrawAABBs;
}

+ (void) setDebugDraw: (BOOL) debugDraw
{
	DebugDraw = debugDraw;
}

+ (BOOL) debugDraw
{
	return DebugDraw;
}

+ (void) setSplitImpulse: (BOOL) splitImpulse
{
	SplitImpulse = splitImpulse;
}

+ (BOOL) splitImpulse
{
	return SplitImpulse;
}

+ (void) setDrawContacts: (BOOL) drawContacts
{
	DrawContacts = drawContacts;
}

+ (BOOL) drawContacts
{
	return DrawContacts;
}


@end
