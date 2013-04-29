#ifndef B3_INT2_H
#define B3_INT2_H

struct b3UnsignedInt2
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

struct b3Int2
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