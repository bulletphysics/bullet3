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

#import <Cocoa/Cocoa.h>
#import "BTOpenGLDisplayDelegate.h"

@class BTDemoEntry;

@interface BTDemo : NSObject <BTOpenGLDisplayDelegate> {

	NSString *_demoName;
	BTDemoEntry *_demo;
}

/**
	@brief Array of all available bullet demos
*/
+ (NSArray*) demoNames;

/**
	@brief Verify a given string is a demo name
*/
+ (BOOL) isDemo: (NSString *) demoName;

/**
	@brief Create a demo with a given name
	@note Returns nil if the name is not a valid demo name
*/
+ (BTDemo*) demoWithName: (NSString *) demoName;

/**
	@brief Initialize a demo with a given name
*/
- (id) initWithDemoName: (NSString *) demoName;

/**
	@brief Reset current demo
*/
- (void) reset;

/**
	@brief Name of current demo
*/
- (NSString*) demoName;

///////////////////////////////////////////////////////////////////////
// BTOpenGLDisplayDelegate

- (void) contextCreated;
- (void) contextWillBeDestroyed;
- (void) contextWillResize;
- (void) contextResized: (NSSize) newSize;
- (void) contextDidResize;
- (void) contextStateInvalidated;

- (void) display: (float) deltaT;

- (void) keyPressed: (unsigned char) key;
- (void) keyReleased: (unsigned char) key;

- (void) specialKeyPressed: (unsigned) GLUTKey;
- (void) specialKeyReleased: (unsigned) GLUTKey;

- (void) mouseButtonPressed: (unsigned) mouseButton;
- (void) mouseButtonReleased: (unsigned) mouseButton;
- (void) mouseMoved: (NSPoint) delta;
- (void) newMousePosition: (NSPoint) newMousePosition;
- (void) scrollWheel: (NSPoint) delta;

///////////////////////////////////////////////////////////////////////
// Global simulation properties -- applies to all demos

+ (void) setIterations: (unsigned) iterations;
+ (unsigned) iterations;

+ (unsigned) minIterations;
+ (unsigned) maxIterations;

+ (void) setDisableDeactivation: (BOOL) disableDeactivation;
+ (BOOL) disableDeactivation;

+ (void) setDrawAABBs: (BOOL) drawAABBs;
+ (BOOL) drawAABBs;

+ (void) setDebugDraw: (BOOL) debugDraw;
+ (BOOL) debugDraw;

+ (void) setSplitImpulse: (BOOL) splitImpulse;
+ (BOOL) splitImpulse;

+ (void) setDrawContacts: (BOOL) drawContacts;
+ (BOOL) drawContacts;

@end
