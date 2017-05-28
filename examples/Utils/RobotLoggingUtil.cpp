#include "RobotLoggingUtil.h"
#include <stdio.h>
#include "LinearMath/btAlignedObjectArray.h"

#include "../Importers/ImportURDFDemo/urdfStringSplit.h"


static bool readLine(FILE* file, btAlignedObjectArray<char>& line)
{
	int c = 0;
	for (c=fgetc(file);(c != EOF && c != '\n');c=fgetc(file))
	{
		line.push_back(c);
	}
	line.push_back(0);
	return (c == EOF);
}


int readMinitaurLogFile(const char* fileName, btAlignedObjectArray<std::string>& structNames, std::string& structTypes, btAlignedObjectArray<MinitaurLogRecord>& logRecords, bool verbose)
{

	int retVal = 0;

	FILE* f = fopen(fileName,"rb");
	if (f)
	{
		if (verbose)
		{
			printf("Opened file %s\n", fileName);
		}
		btAlignedObjectArray<char> line0Buf;
		bool eof = readLine(f,line0Buf);
		btAlignedObjectArray<char> line1Buf;
		eof |= readLine(f,line1Buf);
		std::string line0 = &line0Buf[0];
		structTypes = &line1Buf[0];
		
		btAlignedObjectArray<std::string> separators;
		separators.push_back(",");
		
		urdfStringSplit(structNames,line0,separators);
		if (verbose)
		{
			printf("Num Fields = %d\n",structNames.size());
		}
		btAssert(structTypes.size() == structNames.size());
		if (structTypes.size() != structNames.size())
		{
			retVal = eCorruptHeader;
		}
		int numStructsRead  = 0;

		if (structTypes.size() == structNames.size())
		{
			while (!eof)
			{
				unsigned char blaat[1024];
				size_t s = fread(blaat,2,1,f);
				if (s!=1)
				{
					eof=true;
					retVal = eInvalidAABBAlignCheck;
					break;
				}
				if ((blaat[0] != 0xaa) || (blaat[1] != 0xbb))
				{
					if (verbose)
					{
						printf("Expected 0xaa0xbb, terminating\n");
					}
				}

				if (verbose)
				{
					printf("Reading structure %d\n",numStructsRead);
				}
				MinitaurLogRecord record;

				for (int i=0;i<structNames.size();i++)
				{
					

					switch (structTypes[i])
					{
					case 'I':
					{
						size_t s = fread(blaat,sizeof(int),1,f);
						if (s != 1)
						{
							eof = true;
							retVal = eCorruptValue;
							break;

						}
						int v = (int) *(unsigned int*)blaat;
						if (s==1)
						{
							if (verbose)
							{
								printf("%s = %d\n",structNames[i].c_str(),v);
							}
							record.m_values.push_back(v);
						}
						break;
					}
					case 'i':
					{
						size_t s = fread(blaat,sizeof(int),1,f);
						if (s != 1)
						{
							eof = true;
							retVal = eCorruptValue;
							break;

						}
						int v = *(int*)blaat;
						if (s==1)
						{
							if (verbose)
							{
								printf("%s = %d\n",structNames[i].c_str(),v);
							}
							record.m_values.push_back(v);
						}
						break;
					}
					case 'f':
					{
						float v;
						size_t s = fread(&v,sizeof(float),1,f);
						if (s != 1)
						{
							eof = true;
							break;
						}

						if (s==1)
						{
							if (verbose)
							{
								printf("%s = %f\n",structNames[i].c_str(),v);
							}
							record.m_values.push_back(v);
						}
						break;
					}
					case 'B':
					{
						char v;
						size_t s = fread(&v,sizeof(char),1,f);
						if (s != 1)
						{
							eof = true;
							break;
						}
						if (s==1)
						{
							if (verbose)
							{
								printf("%s = %d\n",structNames[i].c_str(),v);
							}
							record.m_values.push_back(v);
						}
						break;
					}
					default:
					{
						if (verbose)
						{
							printf("Unknown type\n");
						}
						retVal = eUnknownType;
						btAssert(0);
					}
					}
					

				}
				logRecords.push_back(record);
				numStructsRead++;
			}
			if (verbose)
			{
				printf("numStructsRead = %d\n",numStructsRead);
			}
			if (retVal==0)
			{
				retVal = numStructsRead;
			}
		}
		
		//read header and 
	} else
	{
		if (verbose)
		{
			printf("Could not open file %s", fileName);
		}
		retVal = eMinitaurFileNotFound;
	}
	return retVal;
}


FILE* createMinitaurLogFile(const char* fileName, btAlignedObjectArray<std::string>& structNames, std::string& structTypes)
{
	FILE* f = fopen(fileName,"wb");
	if (f)
	{
		for (int i=0;i<structNames.size();i++)
		{
			int len = strlen(structNames[i].c_str());
			fwrite(structNames[i].c_str(),len,1,f);
			if (i<structNames.size()-1)
			{
				fwrite(",",1,1,f);
			}
		}
		int sz = sizeof("\n");
		fwrite("\n",sz-1,1,f);
		fwrite(structTypes.c_str(),strlen(structTypes.c_str()),1,f);
		fwrite("\n",sz-1,1,f);

	}
	

	return f;
}

void appendMinitaurLogData(FILE* f, std::string& structTypes, const MinitaurLogRecord& logData)
{
	if (f)
	{
		unsigned char buf[2] = {0xaa,0xbb};
		fwrite(buf,2,1,f);
		if (structTypes.length() == logData.m_values.size())
		{
			for (int i=0;i<logData.m_values.size();i++)
			{
				switch(structTypes[i])
				{
				case 'i':
				case 'I':
				{
					fwrite(&logData.m_values[i].m_intVal,sizeof(int),1,f);
					break;
				}
				case 'f':
				{
					fwrite(&logData.m_values[i].m_floatVal,sizeof(float),1,f);
					break;
				}
				case 'B':
				{
					fwrite(&logData.m_values[i].m_charVal,sizeof(char),1,f);
					break;
				}
				default:
				{

				}
				}
			}
		}
	}
}

void closeMinitaurLogFile(FILE* f)
{
	if (f)
	{
		fclose(f);
	}
}
