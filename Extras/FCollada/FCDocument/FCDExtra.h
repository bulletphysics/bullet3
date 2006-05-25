/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDExtra.h
	This file contains the FCDExtra class and its sub-classes:
	FCDENode, FCDETechnique and FCDEAttribute.
*/

#ifndef _FCD_EXTRA_H_
#define _FCD_EXTRA_H_

#include "FCDocument/FCDObject.h"

class FCDAnimated;
class FCDAnimatedCustom;
class FCDETechnique;
class FCDENode;

/**
	An extra tree attribute.
	Contains a name and a value string.
*/
struct FCDEAttribute
{
	string name; /**< The attribute name. Must be provided. */
	fstring value; /**< The attribute value. Is optional. */
};

typedef vector<FCDETechnique*> FCDETechniqueList; /**< A dynamically-sized array of extra tree techniques. */
typedef vector<FCDENode*> FCDENodeList; /**< A dynamically-sized array of extra tree nodes. */
typedef vector<FCDEAttribute*> FCDEAttributeList; /**< A dynamically-sized array of extra tree attributes. */

/**
	A COLLADA extra tree.

	An extra tree contains the user-defined COLLADA information
	contained within \<extra\> elements. For this, the extra tree
	root simply contains a list of techniques. Each technique
	belongs to a different application-specific profile.
*/
class FCOLLADA_EXPORT FCDExtra : public FCDObject
{
private:
	FCDETechniqueList techniques;

public:
	/** Constructor: do not use directly.
		The structures that contain extra trees will create them.
		@param document The COLLADA document that owns the extra tree. */
	FCDExtra(FCDocument* document);

	/** Destructor: do not use directly.
		The structures that contain extra trees will release them. */
	virtual ~FCDExtra();

	/** Retrieves the list of techniques contained by this extra tree.
		@return The list of techniques. */
	FCDETechniqueList& GetTechniques() { return techniques; }
	const FCDETechniqueList& GetTechniques() const { return techniques; } /**< See above. */

	/** Retrieves the number of techniques contained by this extra tree.
		@return The number of techniques. */
	size_t GetTechniqueCount() const { return techniques.size(); }

	/** Retrieves a specific technique contained by this extra tree.
		@param index The index of the technique.
		@return The technique. This pointer will be NULL if the
			index is out-of-bounds. */
	FCDETechnique* GetTechnique(size_t index) { FUAssert(index < techniques.size(), return NULL); return techniques.at(index); }
	const FCDETechnique* GetTechnique(size_t index) const { FUAssert(index < techniques.size(), return NULL); return techniques.at(index); } /**< See above. */

	/** Adds a new application-specific profile technique to the extra tree.
		If the given application-specific profile already exists
		within the extra tree, the old technique will be returned.
		@param profile The application-specific profile name.
		@return A technique for this application-specific profile. */
	FCDETechnique* AddTechnique(const char* profile);
	inline FCDETechnique* AddTechnique(const string& profile) { return AddTechnique(profile.c_str()); } /**< See above. */

	/** Releases a technique contained within the extra tree.
		@param technique The technique to release. */
	void ReleaseTechnique(FCDETechnique* technique);

	/** Retrieves a specific technique contained by this extra tree.
		@param profile The application-specific profile name of the technique.
		@return The technique that matches the profile name. This pointer may
			be NULL if no technique matches the profile name. */
	FCDETechnique* FindTechnique(const char* profile);
	const FCDETechnique* FindTechnique(const char* profile) const; /**< See above. */
	inline FCDETechnique* FindTechnique(const string& profile) { return FindTechnique(profile.c_str()); } /**< See above. */
	inline const FCDETechnique* FindTechnique(const string& profile) const { return FindTechnique(profile.c_str()); } /**< See above. */

	/** Retrieves the extra tree node that has a given element name.
		This function searches for the extra tree node within all the
		techniques.
		@param name An element name.
		@return The extra tree node that matches the element name. This pointer
			will be NULL if no extra tree node matches the element name. */
	FCDENode* FindRootNode(const char* name);
	const FCDENode* FindRootNode(const char* name) const; /**< See above. */
	inline FCDENode* FindRootNode(const string& name) { return FindRootNode(name.c_str()); } /**< See above. */
	inline const FCDENode* FindRootNode(const string& name) const { return FindRootNode(name.c_str()); } /**< See above. */

	/** [INTERNAL] Reads in the extra tree from a given COLLADA XML tree node.
		@param extraNode The COLLADA \<extra\> element XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the entity.*/
	FUStatus LoadFromXML(xmlNode* extraNode);

	/** [INTERNAL] Writes out the extra tree to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the \<extra\> element.
		@return The created element XML tree node. */
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA extra tree node.

	The extra tree node is a hierarchical structure that contains child
	extra tree nodes as well as attributes. If the extra tree node is a leaf
	of the tree, it may contain textual content.

	The extra tree node leaf may be animated, if it has the 'sid' attribute.
*/
class FCOLLADA_EXPORT FCDENode : public FCDObject
{
private:
	string name;
	fstring content;

	FCDENode* parent;
	FCDENodeList children;
	FCDEAttributeList attributes;

	FCDAnimatedCustom* animated;

public:
	/** Constructor: do not use directly.
		Instead, call the FCDENode::AddChild function of the parent within the hierarchy.
		@param document The COLLADA document that owns the extra tree node.
		@param parent The extra tree node that contains this extra tree node. */
	FCDENode(FCDocument* document, FCDENode* parent);

	/** Destructor: do not use directly.
		Instead, for hierarchical nodes: call the Release function
		or the parent's ReleaseChildNode function and for techniques:
		call the FCDExtra::ReleaseTechnique function. */
	virtual ~FCDENode();

	/** Releases this extra tree node.
		This function is a shortcut to the parent's FCDENode::ReleaseChildNode function. */
	void Release();

	/** Retrieves the name of the extra tree node.
		The name of the extra tree node is the name of the equivalent XML tree node.
		@return The name of the extra tree node. */
	inline const char* GetName() const { return name.c_str(); }

	/** Sets the name of the extra tree node.
		The name of the extra tree node is the name of the equivalent XML tree node.
		@param _name The name of the extra tree node. */
	inline void SetName(const char* _name) { name = _name; }
	inline void SetName(const string& _name) { name = _name; } /**< See above. */

	/** Retrieves the textual content of the extra tree node.
		This value is only valid for extra tree node that have no children,
		as COLLADA doesn't allow for mixed-content.
		@return The textual content of the extra tree node. */
	const fchar* GetContent() const { return content.c_str(); }

	/** Sets the textual content of the extra tree node.
		This function will release all the child node of this extra tree node,
		as COLLADA doesn't allow for mixed-content.
		@param _content The textual content. */
	void SetContent(const fchar* _content);
	inline void SetContent(const fstring& _content) { return SetContent(_content.c_str()); } /**< See above. */

	/** Retrieves the animated values associated with this extra tree node.
		Extra tree node leaves may be animated. If this extra tree node leaf
		is animated, this animated value will contain the animation curves.
		@return The animated value. */
	FCDAnimatedCustom* GetAnimated() { return animated; }
	const FCDAnimatedCustom* GetAnimated() const { return animated; } /**< See above. */

	/** Retrieves the parent of an extra tree node.
		The hierarchy cannot be changed dynamically. If you to move an extra tree node,
		you will need to clone it manually and release the old extra tree node.
		@return The parent extra tree node within the hierarchy. This pointer
			will be NULL if the extra tree node is a extra tree technique. */
	FCDENode* GetParent() { return parent; }
	const FCDENode* GetParent() const { return parent; } /**< See above. */

	/** Retrieves the children of an extra tree node.
		@return The list of child extra tree nodes. */
	FCDENodeList& GetChildNodes() { return children; }
	const FCDENodeList& GetChildNodes() const { return children; } /**< See above. */

	/** Retrieves the number of children of an extra tree node.
		@return The number of children. */
	size_t GetChildNodeCount() const { return children.size(); }

	/** Retrieves a specific child extra tree node.
		@param index The index of the child extra tree node.
		@return The child extra tree node. This pointer will be NULL if the index
			is out-of-bounds. */
	FCDENode* GetChildNode(size_t index) { FUAssert(index < children.size(), return NULL); return children.at(index); }
	const FCDENode* GetChildNode(size_t index) const { FUAssert(index < children.size(), return NULL); return children.at(index); } /**< See above. */

	/** Adds a new child extra tree to this extra tree node.
		@return The new child extra tree node. */
	FCDENode* AddChildNode();

	/** Releases a child extra tree node of this extra tree node.
		@param childNode The child to release. */
	void ReleaseChildNode(FCDENode* childNode);

	/** Retrieves the child extra tree node with the given name.
		@param name A name.
		@return The child extra tree node that matches the given name.
			This pointer will be NULL if no child extra tree node matches
			the given name. */
	FCDENode* FindChildNode(const char* name);
	const FCDENode* FindChildNode(const char* name) const; /**< See above. */
	inline FCDENode* FindChildNode(const string& name) { return FindChildNode(name.c_str()); } /**< See above. */
	inline const FCDENode* FindChildNode(const string& name) const { return FindChildNode(name.c_str()); } /**< See above. */

	/** Retrieves the child extra tree node with the given name.
		This function is used for COLLADA 1.3 backward compatibility,
		where all parameters were described as \<param name='X'>value\</param\>.
		So, if the child extra tree node with the name 'X' as searched for:
		both the above COLLADA 1.3 parameter and the COLLADA 1.4+
		\<X\>value\</X\> parameters will be returned.
		@param name The parameter name.
		@return The first child extra tree node holding the wanted parameter within the hierarchy.
			This pointer will be NULL to indicate that no parameter matches the given name. */
	FCDENode* FindParameter(const char* name);

	/** Retrieves a list of all the parameters contained within the hierarchy.
		This function is used for COLLADA 1.3 backward compatibility,
		where all parameters were described as \<param name='X'>value\</param\>.
		In COLLADA 1.4+, the same parameter should be described as: \<X\>value\</X\>.
		Using this function, both parameters would be returned with the name 'X'.
		@param nodes The list of parameters to fill in. This list is not emptied by the function.
		@param names The list of names of the parameters. This list is not emptied by the function. */
	void FindParameters(FCDENodeList& nodes, StringList& names);

	/** Retrieves the list of attributes for this extra tree node.
		@return The list of attributes. */
	FCDEAttributeList& GetAttributes() { return attributes; }
	const FCDEAttributeList& GetAttributes() const { return attributes; } /**< See above. */

	/** Retrieves the number of attributes for this extra tree node.
		@return The number of attributes. */
	size_t GetAttributeCount() const { return attributes.size(); }

	/** Retrieves a specific attribute of this extra tree node.
		@param index The index.
		@return The attribute at this index. This pointer will be NULL
			if the index is out-of-bounds. */
	FCDEAttribute* GetAttribute(size_t index) { FUAssert(index < attributes.size(), return NULL); return attributes.at(index); }
	const FCDEAttribute* GetAttribute(size_t index) const { FUAssert(index < attributes.size(), return NULL); return attributes.at(index); } /**< See above. */

	/** Adds a new attribute to this extra tree node.
		If an attribute with the same name already exists, this function simply
		assigns the new value to the existing attribute and returns the existing attribute.
		@param _name The name of the attribute.
		@param _value The value of the attribute.
		@return The new attribute. */
	FCDEAttribute* AddAttribute(const char* _name, const fchar* _value);
	inline FCDEAttribute* AddAttribute(const string& _name, const fchar* _value) { return AddAttribute(_name.c_str(), _value); } /**< See above. */
	inline FCDEAttribute* AddAttribute(const char* _name, const fstring& _value) { return AddAttribute(_name, _value.c_str()); } /**< See above. */
	inline FCDEAttribute* AddAttribute(const string& _name, const fstring& _value) { return AddAttribute(_name.c_str(), _value.c_str()); } /**< See above. */
	template <typename T> inline FCDEAttribute* AddAttribute(const char* _name, const T& _value) { return AddAttribute(_name, TO_FSTRING(_value)); } /**< See above. */
	template <typename T> inline FCDEAttribute* AddAttribute(const string& _name, const T& _value) { return AddAttribute(_name.c_str(), TO_FSTRING(_value)); } /**< See above. */

	/** Releases an attribute of this extra tree node.
		@param attribute The attribute to release. */
	void ReleaseAttribute(FCDEAttribute* attribute);

	/** Retrieve the attribute of this extra tree node with the given name.
		Attribute names are unique within an extra tree node.
		@param name The attribute name.
		@return The attribute that matches the name. This pointer will be NULL if
			there is no attribute with the given name. */
	FCDEAttribute* FindAttribute(const char* name);
	const FCDEAttribute* FindAttribute(const char* name) const; /**< See above. */

	/** [INTERNAL] Reads in the extra tree node from a given COLLADA XML tree node.
		@param customNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the extra tree node.*/
	virtual FUStatus LoadFromXML(xmlNode* customNode);

	/** [INTERNAL] Writes out the extra tree node to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the extra tree node.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

protected:
	/** [INTERNAL] Reads in the children nodes of the extra tree node.
		Used by the FCDETechnique class exclusively.
		@param customNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the extra tree node.*/
	FUStatus ReadChildrenFromXML(xmlNode* customNode);

	/** [INTERNAL] Writes out the children nodes of extra tree node.
		Used by the FCDETechnique class exclusively.
		@param customNode The COLLADA XML node for the extra tree node. */
	void WriteChildrenToXML(xmlNode* customNode) const;
};

/**
	A COLLADA extra tree technique.

	For convenience, this extra tree technique is based on top of the FCDENode class.
	An extra tree technique is the root of the extra tree specific to 
	the profile of an application.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDETechnique : public FCDENode
{
private:
	string profile;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDExtra::AddTechnique function.
		@param document The COLLADA document that owns the technique.
		@param profile The application-specific profile name. */
	FCDETechnique(FCDocument* document, const char* profile);

	/** Destructor: do not use directly.
		Instead, release a technique using the FCDExtra::ReleaseTechnique function. */
	virtual ~FCDETechnique();

	/** Retrieves the name of the application-specific profile of the technique.
		@return The name of the application-specific profile. */
	const char* GetProfile() const { return profile.c_str(); }

	/** [INTERNAL] Reads in the extra tree technique from a given COLLADA XML tree node.
		@param techniqueNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the extra tree technique.*/
	virtual FUStatus LoadFromXML(xmlNode* techniqueNode);

	/** [INTERNAL] Writes out the extra tree technique to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the extra tree technique.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_EXTRA_H_
