/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_TYPE_USER_INFO_H
#define BT_TYPE_USER_INFO_H

class btTypedUserInfo
{
protected:
	int m_type;
	char* m_name;
	void* m_userPointer;

	/* Only systems internal to Bullet are allowed
	 * to use this pointer
	 */
	void* m_privatePointer;
public:
	btTypedUserInfo ()
	{
		m_type = 0;
		m_name = NULL;
		m_userPointer = NULL;
		m_privatePointer = NULL;
	}
	int getType () { return m_type; }
	void setType (int type) { m_type = type; }

	char* getName () { return m_name; }
	void setName (char* name) { m_name = name; }

	void* getUserPointer () { return m_userPointer; }
	void setUserPointer (void* userPointer) { m_userPointer = userPointer; }

	void* getPrivatePointer () { returm m_privatePointer; }
	void setPrivatePointer (void* privatePointer) { m_privatePointer = privatePointer; }
};

#endif //BT_TYPE_USER_INFO_H
