#ifndef BODY_JOINT_INFO_UTILITY_H
#define BODY_JOINT_INFO_UTILITY_H

#include "Bullet3Common/b3Logging.h"

namespace Bullet
{
	class btMultiBodyDoubleData;
	class btMultiBodyFloatData;
};

inline char* strDup(const char* const str)
{
#ifdef _WIN32
	return _strdup(str);
#else
	return strdup(str);
#endif
}

template <typename T, typename U> void addJointInfoFromMultiBodyData(const T* mb, U* bodyJoints, bool verboseOutput)
{
	if (mb->m_baseName) 
	{
		if (verboseOutput) 
		{
			b3Printf("mb->m_baseName = %s\n", mb->m_baseName);
		}
	}
	int qOffset = 7;
	int uOffset = 6;

	for (int link = 0; link < mb->m_numLinks; link++) 
	{
		{
			b3JointInfo info;
			info.m_jointName = 0;
			info.m_linkName = 0;
			info.m_flags = 0;
			info.m_jointIndex = link;
			info.m_qIndex =
				(0 < mb->m_links[link].m_posVarCount) ? qOffset : -1;
			info.m_uIndex =
				(0 < mb->m_links[link].m_dofCount) ? uOffset : -1;

			if (mb->m_links[link].m_linkName) {
				if (verboseOutput) {
					b3Printf("mb->m_links[%d].m_linkName = %s\n", link,
								mb->m_links[link].m_linkName);
				}
				info.m_linkName = strDup(mb->m_links[link].m_linkName);
			}
			if (mb->m_links[link].m_jointName) {
				if (verboseOutput) {
					b3Printf("mb->m_links[%d].m_jointName = %s\n", link,
								mb->m_links[link].m_jointName);
				}
				info.m_jointName = strDup(mb->m_links[link].m_jointName);
			}

			info.m_jointType = mb->m_links[link].m_jointType;
			info.m_jointDamping = mb->m_links[link].m_jointDamping;
			info.m_jointFriction = mb->m_links[link].m_jointFriction;

			if ((mb->m_links[link].m_jointType == eRevoluteType) ||
				(mb->m_links[link].m_jointType == ePrismaticType)) {
				info.m_flags |= JOINT_HAS_MOTORIZED_POWER;
			}
			bodyJoints->m_jointInfo.push_back(info);
		}
		qOffset += mb->m_links[link].m_posVarCount;
		uOffset += mb->m_links[link].m_dofCount;
	}
	
}



#endif //BODY_JOINT_INFO_UTILITY_H