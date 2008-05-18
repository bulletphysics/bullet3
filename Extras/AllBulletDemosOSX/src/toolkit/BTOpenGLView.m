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

#import "BTOpenGLView.h"

#include <CoreFoundation/CoreFoundation.h>
#import <OpenGL/OpenGL.h>
#import <Carbon/Carbon.h>

#import "BTFullScreenWindow.h"
#import "BTGLUTKeyAdapter.h"

#pragma mark -
#pragma mark Private Methods

@interface BTOpenGLView (Internal)

- (void) update;
- (void) boundsDidChange: (NSNotification *) notification;
- (void) setVBL: (BOOL*) vbl forContext: (NSOpenGLContext*) context;
- (void) setMultithreaded: (BOOL) mt;
- (NSOpenGLPixelFormat*) windowedPixelFormat: (BOOL*) antialias;

@end

@implementation BTOpenGLView

#pragma mark -
#pragma mark Bootstrap

- (id)initWithFrame:(NSRect)frameRect
{
	self = [super initWithFrame:frameRect];
	if (self == nil)
	{
		NSLog( @"BTOpenGLView::initWithFrame - Unable to init" );
		return nil;
	}

	_modifierFlags = 0;
	_multisample = YES;
	_vblSync = YES;
	_firstFrame = YES;
	_setupBoundsChangeNotification = NO;

	/*
		Set up the windowed context -- it will be assigned to the
		view later, not now.
	*/
	_windowedContext = [[NSOpenGLContext alloc] initWithFormat: [self windowedPixelFormat: &_multisample]
												  shareContext: nil];

	[self setVBL: &_vblSync forContext: _windowedContext];

	if (_windowedContext == nil)
	{
		NSLog(@"Got nil windowed context");
		[self dealloc];
		return nil;
	}
	
	/*
		Setup and start the update timer.
	*/
	_interval = 1.0 / 60.0;
	_timer = [[NSTimer scheduledTimerWithTimeInterval: _interval
											   target: self
											 selector: @selector(update)
											 userInfo: nil
											  repeats: YES ] retain];
	
	[[NSRunLoop currentRunLoop] addTimer: _timer forMode: NSEventTrackingRunLoopMode];
	
	return self;
}

- (void)dealloc
{
	[_timer invalidate];
	[_timer release];

	[_delegate contextWillBeDestroyed];
	
	[_windowedContext release];
	
	[[NSNotificationCenter defaultCenter] removeObserver: self];
	
	[super dealloc];
}

- (void) awakeFromNib
{
	NSWindow *window = [self window];
	[window setAcceptsMouseMovedEvents: YES];
	[window makeFirstResponder: self];
	[window setInitialFirstResponder: self];		
}

- (void)drawRect:(NSRect)rect
{
	[self update];
}

#pragma mark -
#pragma mark Public API

- (void) setDelegate: (id <BTOpenGLDisplayDelegate>) delegate
{
	// we don't retain delegates
	_delegate = delegate;
}

- (id <BTOpenGLDisplayDelegate>) delegate
{
	return _delegate;
}

- (void) setTargetFPS: (float) fps
{
	float newInterval = 1.0 / fps;
	if ( ABS( newInterval - _interval ) > 1.0e-3 )
	{
		_interval = newInterval;

		[_timer invalidate];
		[_timer release];

		_timer = [[NSTimer scheduledTimerWithTimeInterval: _interval
												   target: self
												 selector: @selector(update)
												 userInfo: nil
												  repeats: YES ] retain];
		
		[[NSRunLoop currentRunLoop] addTimer: _timer forMode: NSEventTrackingRunLoopMode];
	}
	
}

- (float) targetFPS
{
	return 1.0 / _interval;
}

- (float) currentFPS
{
	return _currentFPS;
}

- (void) setFullscreen: (BOOL) fullscreen
{
	if ( fullscreen == _isFullScreen ) return;

	_isFullScreen = fullscreen;
	_suppressResize = YES;
	
	if ( _isFullScreen )
	{
		_windowedWindow = [self window];
		
		/*
			Detach & retain the content view from the non-fullscreen window.
		*/

		NSView *contentView = [_windowedWindow contentView];
		[contentView retain];
		[contentView removeFromSuperviewWithoutNeedingDisplay];

		/*
			Create a fullscreen window, attach the content view,
			and release the content view since the fullscreen window retained it.
		*/
		
		_fullscreenWindow = [[BTFullScreenWindow alloc] initForScreen: [_windowedWindow screen]];
		[_fullscreenWindow setContentView: contentView ];
		[_fullscreenWindow makeKeyAndOrderFront:nil];		
		[contentView release];
		
		/*
			Hide the old window
		*/
		[_windowedWindow orderOut: nil];
		
		/*
			Now, use the SetSystemUIMode API to auto-hide the dock
		*/

		OSStatus error = SetSystemUIMode( kUIModeContentSuppressed, 0 );
		if ( error != noErr)
		{
			NSLog(@"Error couldn't set SystemUIMode: %ld", (long)error);
		}
		
	}
	else if ( _fullscreenWindow )
	{
		/*
			Detach and retain the content view from the fullscreen window
		*/
		NSView *contentView = [_fullscreenWindow contentView];
		[contentView retain];
		[contentView removeFromSuperviewWithoutNeedingDisplay];
		
		/*
			Reparent the content view to the non-fullscreen window,
			and release it since it's now owned by the non-fullscreen window
		*/
		[_windowedWindow setContentView: contentView];
		[contentView release];
		
		[_windowedWindow makeKeyAndOrderFront: nil];

		/*
			Release the fullscreen window
		*/
		[_fullscreenWindow orderOut: nil];
		[_fullscreenWindow release];
		_fullscreenWindow = nil;

		/*
			Restore dock's normal behaior
		*/
		
		OSStatus error = SetSystemUIMode( kUIModeNormal, 0 );
		if ( error != noErr)
		{
			NSLog(@"Error couldn't set SystemUIMode: %ld", (long)error);
		}
	}

	_suppressResize = NO;
	
	[self boundsDidChange: nil];
}

- (BOOL) fullscreen
{
	return _isFullScreen;
}

- (void) setMultisampleRendering: (BOOL) multisample
{
	if ( multisample == _multisample ) return;

	_multisample = multisample;
	_firstFrame = YES;

						
	NSOpenGLContext *oldWindowedContext = _windowedContext;
	_windowedContext = [[NSOpenGLContext alloc] initWithFormat: [self windowedPixelFormat: &_multisample]
												  shareContext:  oldWindowedContext ];

	[self setVBL: &_vblSync forContext: _windowedContext];

	[oldWindowedContext release];
	[_windowedContext setView: self];
	[_windowedContext makeCurrentContext];
	[_windowedContext update];
	[self update];		
}

- (BOOL) multisampleRendering
{
	return _multisample;
}

- (void) setVBLSync: (BOOL) sync
{
	if ( sync == _vblSync ) return;
	
	_vblSync = sync;
	[self setVBL: &_vblSync forContext: _windowedContext];
}

- (BOOL) vblSync
{
	return _vblSync;
}

- (NSOpenGLContext *) openGLContext
{
	return _windowedContext;
}

#pragma mark -
#pragma mark NSView Overrides

- (BOOL) isOpaque
{
	return YES;
}

- (BOOL) acceptsFirstResponder
{
	return YES;
}

- (void) keyDown:(NSEvent *)theEvent
{
	int key = [[theEvent charactersIgnoringModifiers] characterAtIndex:0];	
	
	if ( BTKeyIsAlpha( key ))
	{
		[ _delegate keyPressed: key ];
	}
	else
	{
		[_delegate specialKeyPressed: BTKeyTranslateKeyCodeToSpecial( key )];
	}	
}

- (void) keyUp:(NSEvent *)theEvent
{
	int key = [[theEvent charactersIgnoringModifiers] characterAtIndex:0];	

	if ( BTKeyIsAlpha( key ))
	{
		[ _delegate keyReleased: key ];
	}
	else
	{
		[_delegate specialKeyReleased: BTKeyTranslateKeyCodeToSpecial( key )];
	}	
}

- (void) mouseDown: (NSEvent *) event
{
	int button = GLUT_LEFT_BUTTON;
	switch( [event buttonNumber] )
	{
		case 0: button = GLUT_LEFT_BUTTON; break;
		case 1: button = GLUT_RIGHT_BUTTON; break;
		case 2: button = GLUT_MIDDLE_BUTTON; break;
		default: break;
	}

	if ( _modifierFlags & NSControlKeyMask ) button = GLUT_RIGHT_BUTTON;
	else if ( _modifierFlags & NSAlternateKeyMask ) button = GLUT_MIDDLE_BUTTON;
	
	[_delegate mouseButtonPressed: button];
}

- (void) mouseUp: (NSEvent *) event
{
	int button = GLUT_LEFT_BUTTON;
	switch( [event buttonNumber] )
	{
		case 0: button = GLUT_LEFT_BUTTON; break;
		case 1: button = GLUT_RIGHT_BUTTON; break;
		case 2: button = GLUT_MIDDLE_BUTTON; break;
		default: break;
	}

	if ( _modifierFlags & NSControlKeyMask ) button = GLUT_RIGHT_BUTTON;
	else if ( _modifierFlags & NSAlternateKeyMask ) button = GLUT_MIDDLE_BUTTON;
	
	[_delegate mouseButtonReleased: button];
}

-(void) mouseMoved: (NSEvent *) event
{
	float dx = [event deltaX],
	      dy = [event deltaY];

	NSPoint locationInView = [self convertPoint: [event locationInWindow] fromView: nil ];		  
		
	[_delegate mouseMoved: NSMakePoint( dx, dy )];
	[_delegate newMousePosition: locationInView];
}

-(void) mouseDragged: (NSEvent *) event
{
	[self mouseMoved: event];
}

- (void) scrollWheel: (NSEvent *) event
{
	float dx = [event deltaX],
	      dy = [event deltaY];

	[_delegate scrollWheel: NSMakePoint( dx, dy )];
}

- (void)flagsChanged:(NSEvent *) event
{
	_modifierFlags = [event modifierFlags];
}


#pragma mark -
#pragma mark Private

- (void) update
{
	if ( !_setupBoundsChangeNotification )
	{
		_setupBoundsChangeNotification = YES;
		
		/*
			 This is hacky, but basically, we can't handle bounds-changing
			 ops correctly until everything's set up correctly.
		 */
		[self setPostsBoundsChangedNotifications:YES];
		[[NSNotificationCenter defaultCenter] addObserver: self
												 selector: @selector( boundsDidChange: )
													 name: NSViewFrameDidChangeNotification
												   object: nil];
	}
	
	if (_firstFrame)
	{
		[_windowedContext setView:self];			
	}
	
	[_windowedContext makeCurrentContext];
	
	if (_firstFrame)
	{
		_firstFrame = NO;
		
		[_delegate contextCreated];
		
		if ( _multisample )
		{
			glEnable (GL_MULTISAMPLE_ARB);
			// this fucks up text rendering, on nVIDIA, at least
			//glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
		}
		else
		{
			glDisable( GL_MULTISAMPLE_ARB );
		}

		[self setMultithreaded: NO];
		
		NSSize contextSize;
		if ( _isFullScreen )
		{
			contextSize.width = CGDisplayPixelsWide(kCGDirectMainDisplay);
			contextSize.height = CGDisplayPixelsHigh(kCGDirectMainDisplay);
		}
		else
		{
			contextSize = [self bounds].size;
		}
		
		[_delegate contextWillResize];
		[_delegate contextResized: contextSize];
		[_delegate contextDidResize];
		[_delegate contextStateInvalidated];
	}
	
	double now = CFAbsoluteTimeGetCurrent();
	
	if ( _delegate) [_delegate display: now - _lastFrameTime];
	else
	{
		glClearColor( 0.5, 0.5, 0.5, 1 );
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	}
	
	_lastFrameTime = now;


	[[NSOpenGLContext currentContext] flushBuffer];
	
	/*
		Now, update our FPS
	*/

	{
		static unsigned int frameCounter = 1;
		static double lastCheckTime = 0;
		
		double elapsed = now - lastCheckTime;
		if ( elapsed > 1.0 )
		{
			_currentFPS = (float) ( ((double) frameCounter ) / elapsed );

			lastCheckTime = now;
			frameCounter = 0;
		}

		frameCounter++;
	}
}

- (void) boundsDidChange: (NSNotification *) notification
{
	if ( _suppressResize ) return;
	
	[_windowedContext setView:self];
	[_windowedContext makeCurrentContext];
	[_windowedContext update];
	
	NSSize contextSize = [self bounds].size;

	if ( _delegate )
	{
		[_delegate contextWillResize];
		[_delegate contextResized: contextSize ];
		[_delegate contextDidResize];
	}
	else
	{
		glViewport( 0, 0, (int) contextSize.width, (int) contextSize.height );
	}
}

- (void) setVBL: (BOOL*) vbl forContext: (NSOpenGLContext*) context
{
	GLint value = *vbl ? 1 : 0;
	[context setValues: &value forParameter: NSOpenGLCPSwapInterval];
	
	*vbl = value ? YES : NO;
}

- (void) setMultithreaded: (BOOL) mt
{
	CGLError err = kCGLNoError;
	CGLContextObj ctx = CGLGetCurrentContext();
	
	// Enable Apple's multi-threaded GL engine -- it's generally useful for 
	// high vertex throughput. Not high fragment situations

	if ( mt )
	{
		err =  CGLEnable( ctx, kCGLCEMPEngine );
	}
	else
	{
		err = CGLDisable( ctx, kCGLCEMPEngine );
	}
	
	if (err != kCGLNoError )
	{
		NSLog( @"BTOpenGLView -setMultithreaded: forContext: -- Unable to %s multithreaded GL",
			   mt ? "enable" : "disable" );
	} 	
}

- (NSOpenGLPixelFormat*) windowedPixelFormat: (BOOL*) antialias
{
	NSOpenGLPixelFormatAttribute aaAttrs[] =
	{
		NSOpenGLPFADoubleBuffer,
		NSOpenGLPFAAccelerated,
		NSOpenGLPFADepthSize, (NSOpenGLPixelFormatAttribute)32,
		NSOpenGLPFAStencilSize, (NSOpenGLPixelFormatAttribute)0,
		NSOpenGLPFASingleRenderer,
		NSOpenGLPFASampleBuffers, (NSOpenGLPixelFormatAttribute)( 1 ),
		NSOpenGLPFASamples, (NSOpenGLPixelFormatAttribute)( 4 ),
		NSOpenGLPFAScreenMask, (NSOpenGLPixelFormatAttribute) CGDisplayIDToOpenGLDisplayMask(kCGDirectMainDisplay),
		NSOpenGLPFANoRecovery,
		(NSOpenGLPixelFormatAttribute)0
	};

	NSOpenGLPixelFormatAttribute vanillaAttrs[] =
	{
		NSOpenGLPFADoubleBuffer,
		NSOpenGLPFAAccelerated,
		NSOpenGLPFADepthSize, (NSOpenGLPixelFormatAttribute)32,
		NSOpenGLPFAStencilSize, (NSOpenGLPixelFormatAttribute)0,
		NSOpenGLPFASingleRenderer,
		NSOpenGLPFASampleBuffers, (NSOpenGLPixelFormatAttribute)( 0 ),
		NSOpenGLPFASamples, (NSOpenGLPixelFormatAttribute)( 0 ),
		NSOpenGLPFAScreenMask, (NSOpenGLPixelFormatAttribute) CGDisplayIDToOpenGLDisplayMask(kCGDirectMainDisplay),
		NSOpenGLPFANoRecovery,
		(NSOpenGLPixelFormatAttribute)0
	};

	NSOpenGLPixelFormat* fmt = 0;
	
	if ( *antialias )
	{
		fmt = [[NSOpenGLPixelFormat alloc] initWithAttributes: (NSOpenGLPixelFormatAttribute*) aaAttrs]; 
		if ( nil == fmt )
		{
			*antialias = NO;
			fmt = [[NSOpenGLPixelFormat alloc] initWithAttributes: (NSOpenGLPixelFormatAttribute*) vanillaAttrs]; 
		}
	}
	else
	{
		fmt = [[NSOpenGLPixelFormat alloc] initWithAttributes: (NSOpenGLPixelFormatAttribute*) vanillaAttrs]; 
	}
	
	return fmt;
}


@end
