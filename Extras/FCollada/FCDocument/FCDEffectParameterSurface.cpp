/*
	Copyright (C) 2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDEffectPass.h"
#include "FCDocument/FCDEffectProfile.h"
#include "FCDocument/FCDEffectTechnique.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterSurface.h"
#include "FCDocument/FCDImage.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

// surface type parameter
FCDEffectParameterSurface::FCDEffectParameterSurface(FCDocument* document) : FCDEffectParameter(document)
{
	initMethod = NULL;
}

FCDEffectParameterSurface::~FCDEffectParameterSurface()
{
	SAFE_DELETE(initMethod);
	names.clear();
	images.clear();
}

void FCDEffectParameterSurface::SetInitMethod(FCDEffectParameterSurfaceInit* method)
{
	SAFE_DELETE(initMethod);
	initMethod = method;
}

// Retrieves the index that matches the given image.
size_t FCDEffectParameterSurface::FindImage(const FCDImage* image) const
{
	FCDImageList::const_iterator it = std::find(images.begin(), images.end(), image);
	if (it != images.end())
	{
		return it - images.begin();
	}
	else return (size_t) -1;
}

// Adds an image to the list.
size_t FCDEffectParameterSurface::AddImage(FCDImage* image)
{
	size_t index = FindImage(image);
	if (index == (size_t) -1)
	{
		index = images.size();
		images.push_back(image);
	}
	return index;
}

// Removes an image from the list.
void FCDEffectParameterSurface::RemoveImage(FCDImage* image)
{
	size_t index = FindImage(image);
	if (index != (size_t) -1)
	{
		images.erase(images.begin() + index);

		if (initMethod != NULL && initMethod->GetInitType() == FCDEffectParameterSurfaceInitFactory::CUBE)
		{
			// Shift down all the indexes found within the cube map initialization.
			FCDEffectParameterSurfaceInitCube* cube = (FCDEffectParameterSurfaceInitCube*) initMethod;
			for (UInt16List::iterator itI = cube->order.begin(); itI != cube->order.end(); ++itI)
			{
				if ((*itI) == index) (*itI) = 0;
				else if ((*itI) > index) --(*itI);
			}
		}
	}
}

// Clone
FCDEffectParameter* FCDEffectParameterSurface::Clone()
{
	FCDEffectParameterSurface* clone = new FCDEffectParameterSurface(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->images.clear();
	for(uint32 i=0; i<images.size(); i++)
		clone->images.push_back(images[i]);
	clone->names.clear();
	for(uint32 i=0; i<names.size(); i++)
		clone->names.push_back(names[i]);

	if(initMethod)
		clone->initMethod = initMethod->Clone();

	clone->size = size;
	clone->viewportRatio = viewportRatio;
	clone->mipLevelCount = mipLevelCount;
	clone->generateMipmaps = generateMipmaps;

	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterSurface::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == SURFACE)
	{
		FCDEffectParameterSurface* s = (FCDEffectParameterSurface*) target;
		s->images.clear();
		for(uint32 i=0; i<images.size(); i++)
			s->images.push_back(images[i]);
		s->names.clear();
		for(uint32 i=0; i<names.size(); i++)
			s->names.push_back(names[i]);

//		s->initMethod->initType = initMethod->GetInitType();
		s->size = size;
		s->viewportRatio = viewportRatio;
		s->mipLevelCount = mipLevelCount;
		s->generateMipmaps = generateMipmaps;
	}
}

FUStatus FCDEffectParameterSurface::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* surfaceNode = FindChildByType(parameterNode, DAE_FXCMN_SURFACE_ELEMENT);

	bool initialized = false;
	xmlNode* valueNode = NULL;
	//The surface can now contain many init_from elements (1.4.1)
	xmlNodeList valueNodes;
	FindChildrenByType(surfaceNode, DAE_INITFROM_ELEMENT, valueNodes);
	for (xmlNodeList::iterator it = valueNodes.begin(); it != valueNodes.end(); ++it)
	{
		initialized = true;
		if(!initMethod)
			initMethod = new FCDEffectParameterSurfaceInitFrom();

		FCDEffectParameterSurfaceInitFrom* ptrInit = (FCDEffectParameterSurfaceInitFrom*)initMethod;
		StringList tempNames;
		FUStringConversion::ToStringList(ReadNodeContentDirect(*it), tempNames);
		
		if (tempNames.size() == 0 || tempNames[0].empty())
		{
			return status.Fail(FS("<init_from> element is empty in surface parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
		}

		if(tempNames.size() == 1) //might be 1.4.1, so allow the attributes mip, slice, face
		{
			if(HasNodeProperty(*it, DAE_MIP_ATTRIBUTE))
			{
				string mip = ReadNodeProperty(*it, DAE_MIP_ATTRIBUTE);
				ptrInit->mip.push_back(mip);
			}
			if(HasNodeProperty(*it, DAE_SLICE_ATTRIBUTE))
			{
				string slice = ReadNodeProperty(*it, DAE_SLICE_ATTRIBUTE);
				ptrInit->slice.push_back(slice);
			}
			if(HasNodeProperty(*it, DAE_FACE_ATTRIBUTE))
			{
				string face = ReadNodeProperty(*it, DAE_FACE_ATTRIBUTE);
				ptrInit->face.push_back(face);
			}
		}

		//go through the new names, get their images, and add update the image and name lists
		for(uint32 i=0; i<tempNames.size(); i++)
		{
			names.push_back(tempNames[i]);
			FCDImage* image = GetDocument()->FindImage(tempNames[i]);
			if (image != NULL)
			{
				images.push_back(image);
				
			}
			//else return status.Fail(FS("Unable to find image source for surface parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
		}
	}


	//Check if it's initialized AS NULL
	if(!initialized)
	{
		valueNode = FindChildByType(surfaceNode, DAE_INITASNULL_ELEMENT);
		if(valueNode)
		{
			initialized = true;
			initMethod = FCDEffectParameterSurfaceInitFactory::Create(FCDEffectParameterSurfaceInitFactory::AS_NULL);
		}
	}
	//Check if it's initialized AS TARGET
	if(!initialized)
	{
		valueNode = FindChildByType(surfaceNode, DAE_INITASTARGET_ELEMENT);
		if(valueNode)
		{
			initialized = true;
			initMethod = FCDEffectParameterSurfaceInitFactory::Create(FCDEffectParameterSurfaceInitFactory::AS_TARGET);
		}
	}
	//Check if it's initialized AS CUBE
	if(!initialized)
	{
		valueNode = FindChildByType(surfaceNode, DAE_INITCUBE_ELEMENT);
		if(valueNode)
		{
			initialized = true;
			initMethod = FCDEffectParameterSurfaceInitFactory::Create(FCDEffectParameterSurfaceInitFactory::CUBE);
			FCDEffectParameterSurfaceInitCube* ptrInit = (FCDEffectParameterSurfaceInitCube*) initMethod;

			//Check if it's an ALL reference
			xmlNode* refNode = FindChildByType(valueNode, DAE_ALL_ELEMENT);
			if(refNode)
			{
				ptrInit->cubeType = FCDEffectParameterSurfaceInitCube::ALL;
				string name = ReadNodeProperty(refNode, DAE_REF_ATTRIBUTE);
				if (name.empty())
				{
					return status.Fail(FS("<init_cube>'s all reference is empty in surface parameter: ") + TO_FSTRING(GetReference()), surfaceNode->line);
				}
				names.push_back(name);

				FCDImage* image = GetDocument()->FindImage(name);
				if (image == NULL)
				{
					return status.Fail(FS("Unable to find image source for surface parameter: ") + TO_FSTRING(GetReference()), surfaceNode->line);
				}
				images.push_back(image);
			}

			//Check if it's a PRIMARY reference
			if(!refNode)
			{
				refNode = FindChildByType(valueNode, DAE_PRIMARY_ELEMENT);
				if(refNode)
				{
					ptrInit->cubeType = FCDEffectParameterSurfaceInitCube::PRIMARY;
					string name = ReadNodeProperty(refNode, DAE_REF_ATTRIBUTE);
					if (name.empty())
					{
						return status.Fail(FS("<init_cube>'s primary reference is empty in surface parameter: ") + TO_FSTRING(GetReference()), valueNode->line);
					}
					names.push_back(name);

					FCDImage* image = GetDocument()->FindImage(name);
					if (image == NULL)
					{
						return status.Fail(FS("Unable to find image source for surface parameter: ") + TO_FSTRING(GetReference()), valueNode->line);
					}
					images.push_back(image);
	
					xmlNode* orderNode = FindChildByType(refNode, DAE_ORDER_ELEMENT);
					if(orderNode)
					{
						//FIXME: complete when the spec has more info
					}
				}
			}

			//Check if it's a FACE reference
			if(!refNode)
			{
				xmlNodeList faceNodes;
				FindChildrenByType(valueNode, DAE_FACE_ELEMENT, faceNodes);
				if(faceNodes.size()==6)
				{
					ptrInit->cubeType = FCDEffectParameterSurfaceInitCube::FACE;
					for(uint8 ii=0; ii<faceNodes.size(); ii++)
					{
						string valueName = ReadNodeProperty(faceNodes[ii], DAE_REF_ATTRIBUTE);
						if (valueName.empty())
						{
							return status.Fail(FS("<init_cube>'s face reference is empty in surface parameter: ") + TO_FSTRING(GetReference()), refNode->line);
						}
						names.push_back(valueName);
						FCDImage* image = GetDocument()->FindImage(valueName);
						
						if (image == NULL)
						{
							return status.Fail(FS("Unable to find image source for surface parameter: ") + TO_FSTRING(GetReference()), refNode->line);
						}
						images.push_back(image);
					}
				}
			}
		}
	}

	//Check if it's initialized AS VOLUME
	if(!initialized)
	{
		valueNode = FindChildByType(surfaceNode, DAE_INITVOLUME_ELEMENT);
		if(valueNode)
		{
			initialized = true;
			initMethod = FCDEffectParameterSurfaceInitFactory::Create(FCDEffectParameterSurfaceInitFactory::VOLUME);
			FCDEffectParameterSurfaceInitVolume* ptrInit = (FCDEffectParameterSurfaceInitVolume*) initMethod;

			//Check if it's an ALL reference
			xmlNode* refNode = FindChildByType(valueNode, DAE_ALL_ELEMENT);
			if(refNode)
			{
				ptrInit->volumeType = FCDEffectParameterSurfaceInitVolume::ALL;
				string name = ReadNodeProperty(refNode, DAE_REF_ATTRIBUTE);
				if (name.empty())
				{
					return status.Fail(FS("<init_cube>'s all reference is empty in surface parameter: ") + TO_FSTRING(GetReference()), refNode->line);
				}
				names.push_back(name);

				FCDImage* image = GetDocument()->FindImage(name);
				if (image == NULL)
				{
					return status.Fail(FS("Unable to find image source for surface parameter: ") + TO_FSTRING(GetReference()), refNode->line);
				}
				images.push_back(image);
			}

			//Check if it's a PRIMARY reference
			if(!refNode)
			{
				refNode = FindChildByType(valueNode, DAE_PRIMARY_ELEMENT);
				if(refNode)
				{
					ptrInit->volumeType = FCDEffectParameterSurfaceInitVolume::PRIMARY;
					string name = ReadNodeProperty(refNode, DAE_REF_ATTRIBUTE);
					if (name.empty())
					{
						return status.Fail(FS("<init_cube>'s primary reference is empty in surface parameter: ") + TO_FSTRING(GetReference()), valueNode->line);
					}
					names.push_back(name);

					FCDImage* image = GetDocument()->FindImage(name);
					if (image == NULL)
					{
						return status.Fail(FS("Unable to find image source for surface parameter: ") + TO_FSTRING(GetReference()), valueNode->line);
					}
					images.push_back(image);
				}
			}
		}
	}

	//Check if it's initialized as PLANAR
	if(!initialized)
	{
		valueNode = FindChildByType(surfaceNode, DAE_INITPLANAR_ELEMENT);
		if(valueNode)
		{
			initialized = true;
			initMethod = FCDEffectParameterSurfaceInitFactory::Create(FCDEffectParameterSurfaceInitFactory::PLANAR);

			//Check if it's an ALL reference
			xmlNode* refNode = FindChildByType(valueNode, DAE_ALL_ELEMENT);
			if(refNode)
			{
				string name = ReadNodeProperty(refNode, DAE_REF_ATTRIBUTE);
				if (name.empty())
				{
					return status.Fail(FS("<init_planar>'s all reference is empty in surface parameter: ") + TO_FSTRING(GetReference()), refNode->line);
				}
				names.push_back(name);

				FCDImage* image = GetDocument()->FindImage(name);
				if (image == NULL)
				{
					return status.Fail(FS("Unable to find image source for surface parameter: ") + TO_FSTRING(GetReference()), refNode->line);
				}
				images.push_back(image);
			}
		}
	}
	
	// It is acceptable for a surface not to have an initialization option
	//but we should flag a warning
	if(!initialized)
	{
		DebugOut("Warning: surface %s not initialized", GetReference().c_str());
	}

	xmlNode* sizeNode = FindChildByType(parameterNode, DAE_SIZE_ELEMENT);
	size = FUStringConversion::ToPoint(ReadNodeContentDirect(sizeNode));
	xmlNode* viewportRatioNode = FindChildByType(parameterNode, DAE_VIEWPORT_RATIO);
	viewportRatio = FUStringConversion::ToFloat(ReadNodeContentDirect(viewportRatioNode));
	xmlNode* mipLevelsNode = FindChildByType(parameterNode, DAE_MIP_LEVELS);
	mipLevelCount = (uint16) FUStringConversion::ToInt32(ReadNodeContentDirect(mipLevelsNode));
	xmlNode* mipmapGenerateNode = FindChildByType(parameterNode, DAE_MIPMAP_GENERATE);
	generateMipmaps = FUStringConversion::ToBoolean(ReadNodeContentDirect(mipmapGenerateNode));

	return status;
}

xmlNode* FCDEffectParameterSurface::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	xmlNode* surfaceNode = AddChild(parameterNode, DAE_FXCMN_SURFACE_ELEMENT);
	if (!images.empty() && initMethod != NULL)
	{
		switch (initMethod->GetInitType())
		{
			case FCDEffectParameterSurfaceInitFactory::FROM:
			{
				//Since 1.4.1, there are two possibilities here.
				//Possibility 1
				//<init_from> image1 image2...imageN </init_from>

				//Possibility 2
				//<init_from mip=... face=... slice=...> image1 </init_from>
				//<init_from mip=... face=... slice=...> image2 </init_from>

				FCDEffectParameterSurfaceInitFrom* in = (FCDEffectParameterSurfaceInitFrom*)initMethod;
				size_t size = images.size(); //images.size() should always be equal to names.size()

				if( size == in->face.size() || size == in->mip.size() || size == in->slice.size())
				{
					//This is possibility 2
					for(uint32 i=0; i<size; i++)
					{
						xmlNode* childNode = AddChild(surfaceNode, DAE_INITFROM_ELEMENT, images[i]->GetDaeId());
						AddAttribute(childNode, DAE_MIP_ATTRIBUTE, in->mip[i]);
						AddAttribute(childNode, DAE_SLICE_ATTRIBUTE, in->slice[i]);
						AddAttribute(childNode, DAE_FACE_ATTRIBUTE, in->face[i]);
					}
				}
				else
				{
					//This is possibility 1
					globalSBuilder.reserve(size * 18); // Pulled out of a hat
					StringList::const_iterator itV = names.begin();
					globalSBuilder.set(*itV);
					for (++itV; itV != names.end(); ++itV) 
					{ 
						globalSBuilder.append(' ');
						globalSBuilder.append(*itV);
					}
					
					xmlNode* childNode = AddChild(surfaceNode, DAE_INITFROM_ELEMENT);
					AddContent(childNode, globalSBuilder.ToString());
				}
				break;
			}
			case FCDEffectParameterSurfaceInitFactory::AS_NULL:
			{
				AddChild(surfaceNode, DAE_INITASNULL_ELEMENT);
				break;
			}
			case FCDEffectParameterSurfaceInitFactory::AS_TARGET:
			{
				AddChild(surfaceNode, DAE_INITASTARGET_ELEMENT);
				break;
			}
			case FCDEffectParameterSurfaceInitFactory::VOLUME:
			{
				FCDEffectParameterSurfaceInitVolume* in = (FCDEffectParameterSurfaceInitVolume*)initMethod;
				xmlNode* childNode = AddChild(surfaceNode, DAE_INITVOLUME_ELEMENT);
				if(in->volumeType == FCDEffectParameterSurfaceInitVolume::ALL)
				{
					xmlNode* typeNode = AddChild(childNode, DAE_ALL_ELEMENT);
					AddAttribute(typeNode, DAE_REF_ATTRIBUTE, names[0]);
				}
				else if(in->volumeType == FCDEffectParameterSurfaceInitVolume::PRIMARY)
				{
					xmlNode* typeNode = AddChild(childNode, DAE_PRIMARY_ELEMENT);
					AddAttribute(typeNode, DAE_REF_ATTRIBUTE, names[0]);
				}
				break;
			}
			case FCDEffectParameterSurfaceInitFactory::CUBE:
			{
				FCDEffectParameterSurfaceInitCube* in = (FCDEffectParameterSurfaceInitCube*)initMethod;
				xmlNode* childNode = AddChild(surfaceNode, DAE_INITCUBE_ELEMENT);
				if(in->cubeType == FCDEffectParameterSurfaceInitCube::ALL)
				{
					xmlNode* typeNode = AddChild(childNode, DAE_ALL_ELEMENT);
					AddAttribute(typeNode, DAE_REF_ATTRIBUTE, names[0]);
				}
				else if(in->cubeType == FCDEffectParameterSurfaceInitCube::PRIMARY)
				{
					xmlNode* typeNode = AddChild(childNode, DAE_PRIMARY_ELEMENT);
					AddChild(typeNode, DAE_ORDER_ELEMENT); //FIXME: complete when the spec gets more info.
					AddAttribute(typeNode, DAE_REF_ATTRIBUTE, names[0]);
				}
				if(in->cubeType == FCDEffectParameterSurfaceInitCube::FACE)
				{
					xmlNode* childNode = AddChild(surfaceNode, DAE_FACE_ELEMENT);
					AddAttribute(childNode, DAE_REF_ATTRIBUTE, names[0]);
				}
				break;
			}
			case FCDEffectParameterSurfaceInitFactory::PLANAR:
			{
				xmlNode* childNode = AddChild(surfaceNode, DAE_INITPLANAR_ELEMENT);
				xmlNode* typeNode = AddChild(childNode, DAE_ALL_ELEMENT);
				AddAttribute(typeNode, DAE_REF_ATTRIBUTE, names[0]);
				break;
			}
			default:
				break;
		}

	}
	return parameterNode;
}


void FCDEffectParameterSurfaceInit::Clone(FCDEffectParameterSurfaceInit* UNUSED(clone))
{
	//no member variables to copy in this class, but leave this for future use.
}

FCDEffectParameterSurfaceInitCube::FCDEffectParameterSurfaceInitCube()
{
	cubeType = ALL;
}

FCDEffectParameterSurfaceInit* FCDEffectParameterSurfaceInitCube::Clone()
{
	FCDEffectParameterSurfaceInitCube* clone = new FCDEffectParameterSurfaceInitCube();
	FCDEffectParameterSurfaceInit::Clone(clone);
	clone->cubeType = cubeType;
	return clone;
}

FCDEffectParameterSurfaceInit* FCDEffectParameterSurfaceInitFrom::Clone()
{
	FCDEffectParameterSurfaceInitFrom* clone = new FCDEffectParameterSurfaceInitFrom();
	FCDEffectParameterSurfaceInit::Clone(clone);
	clone->face = face;
	clone->mip = mip;
	clone->slice = slice;
	return clone;
}

FCDEffectParameterSurfaceInitVolume::FCDEffectParameterSurfaceInitVolume()
{
	volumeType = ALL;
}

FCDEffectParameterSurfaceInit* FCDEffectParameterSurfaceInitVolume::Clone()
{
	FCDEffectParameterSurfaceInitVolume* clone = new FCDEffectParameterSurfaceInitVolume();
	FCDEffectParameterSurfaceInit::Clone(clone);
	clone->volumeType = volumeType;
	return clone;
}

FCDEffectParameterSurfaceInit* FCDEffectParameterSurfaceInitAsNull::Clone()
{
	FCDEffectParameterSurfaceInitAsNull* clone = new FCDEffectParameterSurfaceInitAsNull();
	FCDEffectParameterSurfaceInit::Clone(clone);
	return clone;
}

FCDEffectParameterSurfaceInit* FCDEffectParameterSurfaceInitAsTarget::Clone()
{
	FCDEffectParameterSurfaceInitAsTarget* clone = new FCDEffectParameterSurfaceInitAsTarget();
	FCDEffectParameterSurfaceInit::Clone(clone);
	return clone;
}

FCDEffectParameterSurfaceInit* FCDEffectParameterSurfaceInitPlanar::Clone()
{
	FCDEffectParameterSurfaceInitPlanar* clone = new FCDEffectParameterSurfaceInitPlanar();
	FCDEffectParameterSurfaceInit::Clone(clone);
	return clone;
}

FCDEffectParameterSurfaceInit* FCDEffectParameterSurfaceInitFactory::Create(InitType type)
{
	FCDEffectParameterSurfaceInit* parameter = NULL;

	switch (type)
	{
	case FCDEffectParameterSurfaceInitFactory::AS_NULL:		parameter = new FCDEffectParameterSurfaceInitAsNull(); break;
	case FCDEffectParameterSurfaceInitFactory::AS_TARGET:	parameter = new FCDEffectParameterSurfaceInitAsTarget(); break;
	case FCDEffectParameterSurfaceInitFactory::CUBE:		parameter = new FCDEffectParameterSurfaceInitCube(); break;
	case FCDEffectParameterSurfaceInitFactory::FROM:		parameter = new FCDEffectParameterSurfaceInitFrom(); break;
	case FCDEffectParameterSurfaceInitFactory::PLANAR:		parameter = new FCDEffectParameterSurfaceInitPlanar(); break;
	case FCDEffectParameterSurfaceInitFactory::VOLUME:		parameter = new FCDEffectParameterSurfaceInitVolume(); break;
	default: break;
	}

	return parameter;
}