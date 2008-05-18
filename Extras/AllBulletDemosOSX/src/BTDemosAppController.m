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

#import "BTDemosAppController.h"


@implementation BTDemosAppController

- (id) init
{
	if ( self = [super init] )
	{
		// change cwd to .app/Contents/Resources
		NSString *resourcePath = [[NSBundle mainBundle] resourcePath];
		[[NSFileManager defaultManager] changeCurrentDirectoryPath:resourcePath];
	}
	
	return self;
}

- (void) awakeFromNib
{}

#pragma mark -
#pragma mark Public API

- (void) setFullscreen: (BOOL) fullscreen
{
	[_glView setFullscreen: fullscreen];
}

- (BOOL) fullscreen
{
	return [_glView fullscreen];
}

- (void) setShowParameters: (BOOL) showParameters
{
	_showParameters = showParameters;
}

- (BOOL) showParameters
{
	return _showParameters;
}

- (NSArray*) demos
{
	return [BTDemo demoNames];
}

- (void) setDemo: (NSString*) demoName
{
	[_currentDemo release];
	[_glView setDelegate: nil];
	
	_currentDemo = [[BTDemo demoWithName: demoName] retain];

	if ( _currentDemo )
	{
		// the demo is the rendering & input delegate for the gl view
		[_glView setDelegate: _currentDemo];
		
		[_currentDemo contextWillResize];
		[_currentDemo contextResized: [_glView bounds].size];
		[_currentDemo contextDidResize];	
	}
}

- (NSString*) demo
{
	return _currentDemo ? [_currentDemo demoName] : nil;
}

#pragma mark - 
#pragma mark IBActions

- (IBAction) nextDemo: (id) sender
{	
	NSArray *demos = [self demos];
	unsigned index = [demos indexOfObject: [self demo]];
	if ( index != NSNotFound )
	{
		index = ( index + 1 ) % [demos count];
		[self setDemo: [demos objectAtIndex: index]];
	}
}

- (IBAction) previousDemo: (id) sender
{
	NSArray *demos = [self demos];
	unsigned index = [demos indexOfObject: [self demo]];
	if ( index != NSNotFound )
	{
		if ( index == 0 ) index = [demos count] - 1;
		else index--;

		[self setDemo: [demos objectAtIndex: index]];
	}
}

- (IBAction) toggleFullscreen: (id) sender
{
	[self setFullscreen: ![self fullscreen]];
}

- (IBAction) toggleParameters: (id) sender
{
	[self setShowParameters: ![self showParameters]];
}

- (IBAction) resetDemo: (id) sender
{
	[_currentDemo reset];
}

- (IBAction) nullMenuTarget: (id) sender
{
	// this handles a 10.4 bug. A menu bound to some property for toggling
	// will not invoke unless it has a target and selector set.
}

#pragma mark -
#pragma mark NSApplicationDelegate

- (void) applicationDidFinishLaunching: (NSNotification *)aNotification
{
	// note: We load the first demo here and not in -awakeFromNib as the
	// OpenGL view is instantiated, but not running yet at that point

	[self setDemo: [[self demos] objectAtIndex:0]];
}

- (BOOL) applicationShouldTerminateAfterLastWindowClosed: (NSApplication *)theApplication
{
	return YES;
}

@end
