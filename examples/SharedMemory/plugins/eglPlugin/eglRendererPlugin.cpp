
#ifdef EGL_ADD_PYTHON_INIT
#if defined(__APPLE__) && (!defined(B3_NO_PYTHON_FRAMEWORK))
#include <Python/Python.h>
#else
#include <Python.h>
#endif
#endif  //EGL_ADD_PYTHON_INIT

//eglRenderer plugin

//see Bullet/examples/pybullet/examples/eglRendererTest.py

#include "eglRendererPlugin.h"
#include "eglRendererVisualShapeConverter.h"

#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>

struct EGLRendererPluginClass
{
	EGLRendererVisualShapeConverter m_renderer;
	EGLRendererPluginClass()
	{
	}
	virtual ~EGLRendererPluginClass()
	{
	}
};

B3_SHARED_API int initPlugin_eglRendererPlugin(struct b3PluginContext* context)
{
	EGLRendererPluginClass* obj = new EGLRendererPluginClass();
	context->m_userPointer = obj;
	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API int executePluginCommand_eglRendererPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	return -1;
}

B3_SHARED_API void exitPlugin_eglRendererPlugin(struct b3PluginContext* context)
{
	EGLRendererPluginClass* obj = (EGLRendererPluginClass*)context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;
}

//all the APIs below are optional
B3_SHARED_API struct UrdfRenderingInterface* getRenderInterface_eglRendererPlugin(struct b3PluginContext* context)
{
	EGLRendererPluginClass* obj = (EGLRendererPluginClass*)context->m_userPointer;
	return &obj->m_renderer;
}

#ifdef EGL_ADD_PYTHON_INIT

static PyMethodDef eglMethods[] = {
	{NULL, NULL, 0, NULL} /* Sentinel */
};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef moduledef = {
	PyModuleDef_HEAD_INIT, "eglRenderer", /* m_name */
	"eglRenderer for PyBullet ",          /* m_doc */
	-1,                                   /* m_size */
	eglMethods,                           /* m_methods */
	NULL,                                 /* m_reload */
	NULL,                                 /* m_traverse */
	NULL,                                 /* m_clear */
	NULL,                                 /* m_free */
};
#endif

PyMODINIT_FUNC
#if PY_MAJOR_VERSION >= 3
PyInit_eglRenderer(void)
#else
initeglRenderer(void)
#endif
{
	PyObject* m;
#if PY_MAJOR_VERSION >= 3
	m = PyModule_Create(&moduledef);
#else
	m = Py_InitModule3("eglRenderer", eglMethods, "eglRenderer for PyBullet");
#endif

#if PY_MAJOR_VERSION >= 3
	if (m == NULL) return m;
#else
	if (m == NULL) return;
#endif
}
#endif  //EGL_ADD_PYTHON_INIT
