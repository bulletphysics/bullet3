/* Copyright (C) 2006 Charlie C
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#include <sstream>
#include "bDNA.h"
#include "bBlenderFile.h"
#include "btBulletFile.h"
#include "bCommon.h"
#include <map>
#include <vector>
#include <string.h>

bool isBulletFile = false;

using namespace bParse;
typedef std::string bString;

///////////////////////////////////////////////////////////////////////////////
typedef std::map<bString, bString> bStringMap;
typedef std::vector<class bVariable> bVariableList;
typedef std::vector<bString> bStringList;


///////////////////////////////////////////////////////////////////////////////
static FILE *dump = 0;
static bDNA *mDNA =0;
static bStringMap mStructs;


///////////////////////////////////////////////////////////////////////////////
class bVariable
{
public:
	bVariable();
	~bVariable();


	bString dataType;
	bString variableName;


	bString functionName;
	bString classCtor;

	bString memberVariable;
	bString memberDataType;
	bString functionArgs;


	void initialize(bString dataType, bString variable, bStringMap refDataTable);

	bool isPtr;
	bool isFunctionPtr;
	bool isPtrToPtr;
	bool isArray;
	bool isCharArray;
	bool isListBase;
	bool isPadding;
	bool isCommentedOut;
	bool isGeneratedType;
	bool isbString;
};

///////////////////////////////////////////////////////////////////////////////
bool dataTypeStandard(bString dataType)
{
	if (dataType == "char")
		return true;
	if (dataType == "short")
		return true;
	if (dataType == "int")
		return true;
	if (dataType == "long")
		return true;
	if (dataType == "float")
		return true;
	if (dataType == "double")
		return true;
	if (dataType == "void")
		return true;
	if (dataType == "btScalar")
		return true;
	return false;
}

///////////////////////////////////////////////////////////////////////////////
void writeTemplate(short *structData)
{
	bString type = mDNA->getType(structData[0]);
	bString className=type;
	bString prefix = isBulletFile? "bullet_" : "blender_";
	
	int thisLen = structData[1];
	structData+=2;

	bString fileName = prefix+type;

	bVariableList dataTypes;
	bStringMap includeFiles;


	for (int dataVal =0; dataVal<thisLen; dataVal++, structData+=2)
	{
		bString dataType = mDNA->getType(structData[0]);
		bString dataName = mDNA->getName(structData[1]);
		{
			bString newDataType = "";
			bString newDataName = "";
			
			bStringMap::iterator addB = mStructs.find(dataType);
			if (addB != mStructs.end())
			{
				newDataType = addB->second;
				newDataName = dataName;
			}

			else 
			{
				if (dataTypeStandard(dataType))
				{
					newDataType = dataType;
					newDataName = dataName;
				}
				else
				{
					// Unresolved
					// set it to an empty struct
					// if it's not a ptr generate an error
					newDataType = "bInvalidHandle";
					newDataName = dataName;

					if (dataName[0] != '*')
					{
					}

				} 
			}

			if (!newDataType.empty() && !newDataName.empty())
			{
				bVariable var = bVariable();
				var.initialize(newDataType, newDataName, mStructs);
				dataTypes.push_back(var);
			}
		}


		bStringMap::iterator include = mStructs.find(dataType);
		if (include != mStructs.end())
		{
			if (dataName[0] != '*') 
			{
				if (includeFiles.find(dataType)== includeFiles.end())
				{
					includeFiles[dataType]=prefix+dataType;
				}
			}
		}
	}


	fprintf(dump, "###############################################################\n");
	fprintf(dump, "%s = bStructClass()\n", fileName.c_str());
	fprintf(dump, "%s.name = '%s'\n", fileName.c_str(), className.c_str());
	fprintf(dump, "%s.filename = '%s'\n", fileName.c_str(), fileName.c_str());

	bVariableList::iterator vars = dataTypes.begin();
	while (vars!= dataTypes.end())
	{
		fprintf(dump, "%s.dataTypes.append('%s %s')\n", fileName.c_str(), vars->dataType.c_str(), vars->variableName.c_str());
		vars++;
	}

	bStringMap::iterator inc = includeFiles.begin();
	while (inc != includeFiles.end())
	{
		fprintf(dump, "%s.includes.append('%s.h')\n", fileName.c_str(), inc->second.c_str());
		inc++;
	}
	fprintf(dump, "DataTypeList.append(%s)\n", fileName.c_str());
}


///////////////////////////////////////////////////////////////////////////////
char data[]={
"\n"
"class bStructClass:\n"
"    def __init__(self):\n"
"       self.name = \"\";\n"
"       self.filename = \"\";\n"
"       self.includes = []\n"
"       self.dataTypes = []\n"
"\n\n"
"DataTypeList = []\n"
};


///////////////////////////////////////////////////////////////////////////////
int main(int argc,char** argv)
{
	using namespace bParse;
	dump = fopen("dump.py", "w");

	if (!dump) return 0;
	fprintf(dump, "%s\n", data);

	
	char* filename = "../../../Demos/SerializeDemo/testFile.bullet";
	
	if (argc==2)
		filename = argv[1];

	bString fileStr(filename);
	bString extension(".bullet");

	int index2 = fileStr.find(extension);
	if (index2>=0)
		isBulletFile=true;

	
	FILE* fp = fopen (filename,"rb");

	if (!fp)
	{
		printf("error: file not found %s\n",filename);
		exit(0);
	}

	char* memBuf = 0;
	int len = 0;

	long currentpos = ftell(fp); /* save current cursor position */
	long newpos;
	int bytesRead;

	fseek(fp, 0, SEEK_END); /* seek to end */
	newpos = ftell(fp); /* find position of end -- this is the length */
	fseek(fp, currentpos, SEEK_SET); /* restore previous cursor position */
	
	len = newpos;
	
	memBuf = (char*)malloc(len);
	bytesRead = fread(memBuf,len,1,fp);

	bool swap = false;

	
	if (isBulletFile)
	{
		btBulletFile f(memBuf,len);
		swap = (f.getFlags() & FD_ENDIAN_SWAP)!=0;
	} else
	{
		bBlenderFile	f(memBuf,len);
		swap = (f.getFlags() & FD_ENDIAN_SWAP)!=0;
	}

	
	


	char *blenderData = memBuf;
	int sdnaPos=0;
	int mDataStart = 12;

	char *tempBuffer = blenderData;
	for (int i=0; i<len; i++)
	{
		// looking for the data's starting position
		// and the start of SDNA decls

		if (!mDataStart && strncmp(tempBuffer, "REND", 4)==0)
			mDataStart = i;
		if (!sdnaPos && strncmp(tempBuffer, "SDNA", 4)==0)
			sdnaPos = i;

		if (mDataStart && sdnaPos) break;
		tempBuffer++;
	}

	

	FILE* fpdna = fopen("dnaString.txt","w");
	char buf[1024];

	for (int i=0;i<len-sdnaPos;i++)
	{
		int dnaval = (memBuf+sdnaPos)[i];

		if ((i%32)==0)
		{
			sprintf(buf,"%d,\n",dnaval);
			
		} else
		{
			sprintf(buf,"%d,",dnaval);
		}
		
		
		fwrite(buf,strlen(buf),1,fpdna);
	}

	fclose(fpdna);



	mDNA = new bDNA();
	//mDNA->initMemory();
	
	mDNA->init(memBuf+sdnaPos, len-sdnaPos, swap);
	

	for (int i=0; i<mDNA->getNumStructs(); i++)
	{
		short *structData = mDNA->getStruct(i);
		bString type = mDNA->getType(structData[0]);

		bString className = type;
		mStructs[type]=className;
	}


	for (int i=0; i<mDNA->getNumStructs(); i++)
	{
		short *structData = mDNA->getStruct(i);
		writeTemplate(structData);
	}

	delete mDNA;
	fclose(dump);
	return 0;
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
int _getArraySize(char* str)
{
	int a, mul=1;
	char stri[100], *cp=0;
	int len = (int)strlen(str);

	memcpy(stri, str, len+1);
	for (a=0; a<len; a++)
	{
		if (str[a]== '[')
			cp= &(stri[a+1]);
		else if ( str[a]==']' && cp)
		{
			stri[a]= 0;
			mul*= atoi(cp);
		}
	}
	return mul;
}

///////////////////////////////////////////////////////////////////////////////
bVariable::bVariable()
	:	dataType("invalid"),
		variableName("invalid"),
		functionName(""),
		classCtor(""),
		memberVariable(""),
		memberDataType(""),
		functionArgs(""),
		isPtr(false),
		isFunctionPtr(false),
		isPtrToPtr(false),
		isArray(false),
		isCharArray(false),
		isListBase(false),
		isPadding(false),
		isCommentedOut(false),
		isGeneratedType(false),
		isbString(false)
{
}


///////////////////////////////////////////////////////////////////////////////
bVariable::~bVariable()
{
	dataType.clear();
	variableName.clear();
}


///////////////////////////////////////////////////////////////////////////////
void bVariable::initialize(bString type, bString variable, bStringMap refDataTable)
{
	dataType = type;
	variableName = variable;

	if (variableName[0] == '*')
	{
		isPtr = true;
		if (variableName[1] == '*')
			isPtrToPtr = true;
	}
	if (variableName[0] == '(')
		if (variableName[1] == '*')
			isFunctionPtr = true;

	if (variableName[variableName.size()-1] == ']')
	{
		isArray = true;
		if (type == "char")
			isCharArray = true;
	}

	if (type == "ListBase")
		isListBase = true;

	if (variableName[0] == 'p')
	{
		bString sub = variableName.substr(0,3);
		if (sub == "pad")
			isPadding = true;
	}
	if (dataType[0] == '/' && dataType[1] == '/')
		isCommentedOut = true;


	if (refDataTable.find(dataType) != refDataTable.end())
		isGeneratedType = true;

	if (!isBulletFile)
	{
		// replace valid float arrays
		if (dataType == "float" && isArray)
		{
			int size = _getArraySize((char*)variableName.c_str());
			if (size==3)
			{
				dataType = "vec3f";
				variableName = variableName.substr(0, variableName.find_first_of("["));
			}
			if (size==4)
			{
				dataType = "vec4f";
				variableName = variableName.substr(0, variableName.find_first_of("["));
			}
		}
	}
	memberDataType = dataType;
	functionArgs = variableName;
}

// eof
