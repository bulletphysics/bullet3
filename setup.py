from setuptools import setup, Extension, find_packages
from distutils.util import get_platform
from sys import platform as _platform
import os, sys, glob

# see http://stackoverflow.com/a/8719066/295157

def rglob(path, exclude = set()):
	return [x for x in glob.glob(path, recursive = True) if os.path.basename(x) not in exclude]

platform = get_platform()
print(platform)

extra_compile_args = []

define_macros = [
	('GWEN_COMPILE_STATIC', None),
	('BT_USE_DOUBLE_PRECISION', None),
	('BT_ENABLE_ENET', None),
	('BT_ENABLE_CLSOCKET', None),
]

# libraries += [current_python]

libraries = []

sources = []

sources.extend(rglob('examples/pybullet/**/*.c'))

sources.extend(rglob('src/Bullet3Common/**/*.cpp'))
sources.extend(rglob('src/BulletCollision/**/*.cpp'))
sources.extend(rglob('src/BulletDynamics/**/*.cpp'))
sources.extend(rglob('src/BulletInverseDynamics/**/*.cpp'))
sources.extend(rglob('src/BulletSoftBody/**/*.cpp'))
sources.extend(rglob('src/LinearMath/**/*.cpp'))

sources.extend(rglob('Extras/InverseDynamics/**/*.cpp'))
sources.extend(rglob('Extras/Serialize/BulletFileLoader/**/*.cpp'))
sources.extend(rglob('Extras/Serialize/BulletWorldImporter/**/*.cpp'))

sources.extend(rglob('examples/ThirdPartyLibs/BussIK/**/*.cpp'))
sources.extend(rglob('examples/ThirdPartyLibs/clsocket/**/*.cpp'))
sources.extend(rglob('examples/ThirdPartyLibs/Gwen/**/*.cpp'))
sources.extend(rglob('examples/ThirdPartyLibs/stb_image/**/*.cpp'))
sources.extend(rglob('examples/ThirdPartyLibs/tinyxml/**/*.cpp'))

sources.extend(rglob('examples/ThirdPartyLibs/enet/**/*.c', exclude = {'unix.c', 'win32.c'}))
sources.extend(rglob('examples/ThirdPartyLibs/Wavefront/**/*.cpp', exclude = {'main.cpp'}))

sources.extend(rglob('examples/Utils/**/*.cpp'))

sources.extend(rglob('examples/TinyRenderer/**/*.cpp', exclude = {'main.cpp'}))
sources.extend(rglob('examples/MultiThreading/**/*.cpp', exclude = {'main.cpp', 'MultiThreadingExample.cpp'}))
sources.extend(rglob('examples/ExampleBrowser/**/*.cpp', exclude = {'main.cpp', 'ExampleEntries.cpp'}))

sources.extend(rglob('examples/Importers/**/*.cpp', exclude = {
	'BspConverter.cpp',
	'BspLoader.cpp',
	'ImportBspExample.cpp',
	'SerializeSetup.cpp',
	'ImportColladaSetup.cpp',
	'ImportMJCFSetup.cpp',
	'ImportObjExample.cpp',
	'ImportSDFSetup.cpp',
	'ImportSTLSetup.cpp',
	'ImportURDFSetup.cpp',
}))

sources.extend(rglob('examples/SharedMemory/**/*.cpp', exclude = {
	'main.cpp',
	'PhysicsClientExample.cpp',
	'PhysicsClientSharedMemory2.cpp',
	'PhysicsClientSharedMemory2_C_API.cpp',
	'PhysicsLoopBack.cpp',
	'PhysicsLoopBackC_API.cpp',
	'RobotControlExample.cpp',
	'SharedMemoryCommandProcessor.cpp',
}))

sources.extend(rglob('examples/OpenGLWindow/**/*.cpp', exclude = {
	'EGLOpenGLWindow.cpp',
	'MacOpenGLWindow.cpp',
	'Win32OpenGLWindow.cpp',
	'Win32Window.cpp',
	'X11OpenGLWindow.cpp',
}))

# normalize
sources = [os.path.normpath(x) for x in sources]

if _platform.startswith('linux'):
	libraries.extend([
		'dl',
		'pthread',
	])

	define_macros.extend([
		('_LINUX', None),
		('GLEW_STATIC', None),
		('GLEW_INIT_OPENGL11_FUNCTIONS', '1'),
		('GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS', '1'),
		('DYNAMIC_LOAD_X11_FUNCTIONS', None),
		('HAS_SOCKLEN_T', None),
	])

	extra_compile_args.extend(['-fno-inline-functions-called-once'])

	sources.extend([
		'examples/ThirdPartyLibs/enet/unix.c',
		'examples/OpenGLWindow/X11OpenGLWindow.cpp',
		'examples/ThirdPartyLibs/Glew/glew.c',
	])

elif _platform.startswith('win32'):
	print('win32!')

	libraries.extend([
		'Ws2_32',
		'Winmm',
		'User32',
		'Opengl32',
		'kernel32',
		'glu32',
		'Gdi32',
		'Comdlg32',
	])

	define_macros.extend([
		('WIN32', None),
		('GLEW_STATIC', None),
	])

	sources.extend([
		'examples/ThirdPartyLibs/enet/win32.c',
		'examples/OpenGLWindow/Win32Window.cpp',
		'examples/OpenGLWindow/Win32OpenGLWindow.cpp',
		'examples/ThirdPartyLibs/Glew/glew.c',
	])

elif _platform.startswith('darwin'):
	print('darwin!')

	os.environ['LDFLAGS'] = '-framework Cocoa -framework OpenGL'

	define_macros.extend([
		('HAS_SOCKLEN_T', None),
		('_DARWIN', None),
	])

	sources.extend([
		'examples/ThirdPartyLibs/enet/unix.c',
		'examples/OpenGLWindow/MacOpenGLWindow.cpp',
		'examples/OpenGLWindow/MacOpenGLWindowObjC.m',
	])

else:
	raise Exception('Platform %s is notrecognized' % _platform)

include_dirs = [
	'src',
	'examples/ThirdPartyLibs',
	'examples/ThirdPartyLibs/Glew',
	'examples/ThirdPartyLibs/enet/include',
	'examples/ThirdPartyLibs/clsocket/src',
]

pybullet = Extension(
	'pybullet', 
	sources = sources,
	libraries = libraries,
	define_macros = define_macros,
	extra_compile_args = extra_compile_args,
	include_dirs = include_dirs
)

classifiers = [
	'Development Status :: 4 - Beta',
	'License :: OSI Approved :: zlib/libpng License',
	'Operating System :: Microsoft :: Windows',
	'Operating System :: POSIX :: Linux',
	'Operating System :: MacOS',
	'Intended Audience :: Science/Research',
	'Programming Language :: Python',
	'Programming Language :: Python :: 2.7',
	'Programming Language :: Python :: 3.4',
	'Programming Language :: Python :: 3.5',
	'Programming Language :: Python :: 3.6',
	'Topic :: Games/Entertainment :: Simulation',
	'Framework :: Robot Framework',
]

keywords = [
	'game development',
	'virtual reality',
	'physics simulation',
	'robotics',
	'collision detection',
	'opengl',
]

long_description = '''
pybullet is an easy to use Python module for physics simulation, robotics and
machine learning based on the Bullet Physics SDK. With pybullet you can load
articulated bodies from URDF, SDF and other file formats. pybullet provides
forward dynamics simulation, inverse dynamics computation, forward and inverse
kinematics and collision detection and ray intersection queries. Aside from
physics simulation, pybullet supports to rendering, with a CPU renderer and
OpenGL visualization and support for virtual reality headsets.
'''

setup(
	name = 'pybullet',
	version = '0.1.6',
	description = 'Official Python Interface for the Bullet Physics SDK Robotics Simulator',
	long_description = long_description,
	url = 'https://github.com/bulletphysics/bullet3',
	author = 'Erwin Coumans, Yunfei Bai, Jasmine Hsu',
	author_email = 'erwincoumans@google.com',
	license = 'zlib',
	platforms = 'any',
	keywords = keywords,
	ext_modules = [pybullet],
	classifiers = classifiers,
	package_data = {
		'pybullet': ['data/*'],
	},
)
