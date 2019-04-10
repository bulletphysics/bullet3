#ifndef BODY_JOINT_INFO_UTILITY_H
#define BODY_JOINT_INFO_UTILITY_H

#include "Bullet3Common/b3Logging.h"

namespace Bullet
{
class btMultiBodyDoubleData;
class btMultiBodyFloatData;
};  // namespace Bullet

inline char* strDup(const char* const str)
{
#ifdef _WIN32
	return _strdup(str);
#else
	return strdup(str);
#endif
}

template <typename T, typename U>
void addJointInfoFromMultiBodyData(const T* mb, U* bodyJoints, bool verboseOutput)
{
	int numDofs = 0;
	if (mb->m_baseMass>0)
	{
		numDofs = 6;
	}

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
			info.m_jointName[0] = 0;
			info.m_linkName[0] = 0;
			info.m_flags = 0;
			info.m_jointIndex = link;
			info.m_qIndex =
				(0 < mb->m_links[link].m_posVarCount) ? qOffset : -1;
			info.m_uIndex =
				(0 < mb->m_links[link].m_dofCount) ? uOffset : -1;

			if (mb->m_links[link].m_linkName)
			{
				if (verboseOutput)
				{
					b3Printf("mb->m_links[%d].m_linkName = %s\n", link,
							 mb->m_links[link].m_linkName);
				}
				strcpy(info.m_linkName, mb->m_links[link].m_linkName);
			}
			if (mb->m_links[link].m_jointName)
			{
				if (verboseOutput)
				{
					b3Printf("mb->m_links[%d].m_jointName = %s\n", link,
							 mb->m_links[link].m_jointName);
				}
				strcpy(info.m_jointName, mb->m_links[link].m_jointName);
				//info.m_jointName = strDup(mb->m_links[link].m_jointName);
			}

			info.m_jointType = mb->m_links[link].m_jointType;
			info.m_jointDamping = mb->m_links[link].m_jointDamping;
			info.m_jointFriction = mb->m_links[link].m_jointFriction;
			info.m_jointLowerLimit = mb->m_links[link].m_jointLowerLimit;
			info.m_jointUpperLimit = mb->m_links[link].m_jointUpperLimit;
			info.m_jointMaxForce = mb->m_links[link].m_jointMaxForce;
			info.m_jointMaxVelocity = mb->m_links[link].m_jointMaxVelocity;

			info.m_parentFrame[0] = mb->m_links[link].m_parentComToThisPivotOffset.m_floats[0];
			info.m_parentFrame[1] = mb->m_links[link].m_parentComToThisPivotOffset.m_floats[1];
			info.m_parentFrame[2] = mb->m_links[link].m_parentComToThisPivotOffset.m_floats[2];
			info.m_parentFrame[3] = mb->m_links[link].m_zeroRotParentToThis.m_floats[0];
			info.m_parentFrame[4] = mb->m_links[link].m_zeroRotParentToThis.m_floats[1];
			info.m_parentFrame[5] = mb->m_links[link].m_zeroRotParentToThis.m_floats[2];
			info.m_parentFrame[6] = mb->m_links[link].m_zeroRotParentToThis.m_floats[3];

			info.m_jointAxis[0] = 0;
			info.m_jointAxis[1] = 0;
			info.m_jointAxis[2] = 0;
			info.m_parentIndex = mb->m_links[link].m_parentIndex;

			if (info.m_jointType == eRevoluteType)
			{
				info.m_jointAxis[0] = mb->m_links[link].m_jointAxisTop[0].m_floats[0];
				info.m_jointAxis[1] = mb->m_links[link].m_jointAxisTop[0].m_floats[1];
				info.m_jointAxis[2] = mb->m_links[link].m_jointAxisTop[0].m_floats[2];
			}
			if (info.m_jointType == ePrismaticType)
			{
				info.m_jointAxis[0] = mb->m_links[link].m_jointAxisBottom[0].m_floats[0];
				info.m_jointAxis[1] = mb->m_links[link].m_jointAxisBottom[0].m_floats[1];
				info.m_jointAxis[2] = mb->m_links[link].m_jointAxisBottom[0].m_floats[2];
			}

			if ((mb->m_links[link].m_jointType == eRevoluteType) ||
				(mb->m_links[link].m_jointType == ePrismaticType))
			{
				info.m_flags |= JOINT_HAS_MOTORIZED_POWER;
			}
			bodyJoints->m_jointInfo.push_back(info);
		}
		qOffset += mb->m_links[link].m_posVarCount;
		uOffset += mb->m_links[link].m_dofCount;
		numDofs += mb->m_links[link].m_dofCount;
	}
	bodyJoints->m_numDofs = numDofs;
}

#endif  //BODY_JOINT_INFO_UTILITY_H
