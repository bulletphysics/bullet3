/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDAnimation.h"
#include "FCDocument/FCDAnimationChannel.h"
#include "FUtils/FUDaeParser.h"
using namespace FUDaeParser;

FCDAnimation::FCDAnimation(FCDocument* document) : FCDEntity(document, "Animation")
{
}

FCDAnimation::~FCDAnimation()
{
	CLEAR_POINTER_VECTOR(channels);
	CLEAR_POINTER_VECTOR(children);
	childNodes.clear();
}

// Creates a new animation entity sub-tree contained within this animation entity tree.
FCDAnimation* FCDAnimation::AddChild()
{
	FCDAnimation* animation = new FCDAnimation(GetDocument());
	children.push_back(animation);
	return animation;
}

// Releases an animation entity sub-tree contained by this animation entity tree.
void FCDAnimation::ReleaseChild(FCDAnimation* animation)
{
	FCDAnimationList::iterator itA = std::find(children.begin(), children.end(), animation);
	if (itA != children.end())
	{
		delete *itA;
		children.erase(itA);
	}
}

// Adds a new animation channel to this animation entity.
FCDAnimationChannel* FCDAnimation::AddChannel()
{
	FCDAnimationChannel* channel = new FCDAnimationChannel(GetDocument(), this);
	channels.push_back(channel);
	return channel;
}

// Releases an animation channel contained within this animation entity.
void FCDAnimation::ReleaseChannel(FCDAnimationChannel* channel)
{
	FCDAnimationChannelList::iterator itC = std::find(channels.begin(), channels.end(), channel);
	if (itC != channels.end())
	{
		delete *itC;
		channels.erase(itC);
	}
}

// Optimization: Look for the xml child node with the given id
xmlNode* FCDAnimation::FindChildById(const string& _id)
{
	FUCrc32::crc32 id = FUCrc32::CRC32(_id.c_str() + ((_id[0] == '#') ? 1 : 0));
	for (FUXmlNodeIdPairList::iterator it = childNodes.begin(); it != childNodes.end(); ++it)
	{
		if ((*it).id == id) return (*it).node;
	}
	return NULL;
}

// Look for an animation children with the given COLLADA Id.
FCDEntity* FCDAnimation::FindDaeId(const string& daeId)
{
	if (GetDaeId() == daeId) return this;
	
	for (FCDAnimationList::iterator it = children.begin(); it != children.end(); ++it)
	{
		FCDEntity* found = (*it)->FindDaeId(daeId);
		if (found != NULL) return found;
	}
	return NULL;
}

// Retrieve all the curves created under this animation element, in the animation tree
void FCDAnimation::GetCurves(FCDAnimationCurveList& curves)
{
	// Retrieve the curves for this animation tree element
	for (FCDAnimationChannelList::iterator it = channels.begin(); it != channels.end(); ++it)
	{
		const FCDAnimationCurveList& channelCurves = (*it)->GetCurves();
		for (FCDAnimationCurveList::const_iterator itC = channelCurves.begin(); itC != channelCurves.end(); ++itC)
		{
			curves.push_back(*itC);
		}
	}

	// Retrieve the curves for the animation nodes under this one in the animation tree
	for (FCDAnimationList::iterator it = children.begin(); it != children.end(); ++it)
	{
		(*it)->GetCurves(curves);
	}
}

FUStatus FCDAnimation::Link()
{
	FUStatus status;

	// Link the child nodes and check the curves for their drivers
	for (FCDAnimationChannelList::iterator it = channels.begin(); it != channels.end(); ++it)
	{
		status.AppendStatus((*it)->CheckDriver());
	}
	for (FCDAnimationList::iterator it = children.begin(); it != children.end(); ++it)
	{
		status.AppendStatus((*it)->Link());
	}

	return status;
}

// Check for animation curves that need this animated as a driver
bool FCDAnimation::LinkDriver(FCDAnimated* animated)
{
	bool driver = false;

	// Link the child curves and child nodes
	for (FCDAnimationChannelList::iterator it = channels.begin(); it != channels.end(); ++it)
	{
		driver |= (*it)->LinkDriver(animated);
	}
	for (FCDAnimationList::iterator it = children.begin(); it != children.end(); ++it)
	{
		driver |= (*it)->LinkDriver(animated);
	}

	return driver;
}

// Load a Collada animation node from the XML document
FUStatus FCDAnimation::LoadFromXML(xmlNode* node)
{
	FUStatus status = FCDEntity::LoadFromXML(node);
	if (!status) return status;
	if (!IsEquivalent(node->name, DAE_ANIMATION_ELEMENT))
	{
		return status.Warning(FS("Animation library contains unknown element."), node->line);
	}

	// Optimization: Grab all the IDs of the child nodes, in CRC format.
	ReadChildrenIds(node, childNodes);

	// Parse all the inner <channel> elements
	xmlNodeList channelNodes;
	FindChildrenByType(node, DAE_CHANNEL_ELEMENT, channelNodes);
	channels.reserve(channelNodes.size());
	for (xmlNodeList::iterator itC = channelNodes.begin(); itC != channelNodes.end(); ++itC)
	{
		// Parse each <channel> element individually
		// They each handle reading the <sampler> and <source> elements
		FCDAnimationChannel* channel = AddChannel();
		status.AppendStatus(channel->LoadFromXML(*itC));
		if (!status)
		{
			ReleaseChannel(channel);
		}
	}

	// Parse all the hierarchical <animation> elements
	xmlNodeList animationNodes;
	FindChildrenByType(node, DAE_ANIMATION_ELEMENT, animationNodes);
	for (xmlNodeList::iterator itA = animationNodes.begin(); itA != animationNodes.end(); ++itA)
	{
		FCDAnimation* animation = AddChild();
		animation->LoadFromXML(*itA);
	}
	return status;
}

// Search for an animation channel for the given XML pointer in this animation node
void FCDAnimation::FindAnimationChannels(const string& pointer, vector<FCDAnimationChannel*>& targetChannels)
{
	// Look for channels locally
	for (FCDAnimationChannelList::iterator itChannel = channels.begin(); itChannel != channels.end(); ++itChannel)
	{
		if ((*itChannel)->GetTargetPointer() == pointer)
		{
			targetChannels.push_back(*itChannel);
		}
	}

	// Look for channel(s) within the child animations
	for (FCDAnimationList::iterator it = children.begin(); it != children.end(); ++it)
	{
		(*it)->FindAnimationChannels(pointer, targetChannels);
	}
}

// Write out the COLLADA animations to the document
xmlNode* FCDAnimation::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* animationNode = WriteToEntityXML(parentNode, DAE_ANIMATION_ELEMENT);

	// Write out the local channels
	for (FCDAnimationChannelList::const_iterator itChannel = channels.begin(); itChannel != channels.end(); ++itChannel)
	{
		(*itChannel)->WriteToXML(animationNode);
	}

	// Write out the child animations
	for (FCDAnimationList::const_iterator it = children.begin(); it != children.end(); ++it)
	{
		(*it)->WriteToXML(animationNode);
	}

	FCDEntity::WriteToExtraXML(animationNode);
	return animationNode;
}
