#ifndef BT_INT2_H
#define BT_INT2_H

struct btUnsignedInt2
{
	union
	{
		struct
		{
			unsigned int x,y;
		};
		struct
		{
			unsigned int s[2];
		};
	};
};

struct btInt2
{
	union
	{
		struct
		{
			int x,y;
		};
		struct
		{
			int s[2];
		};
	};
};


#endif