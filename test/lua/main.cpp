
// Some quick experiments to use Lua and C (C++) together
// so Lua can be used to setup the Bullet demos
// See http://csl.name/lua/ 
// and http://stackoverflow.com/questions/7298642/passing-c-struct-pointer-to-lua-script


#include <iostream>
#include <vector>

extern "C" {
	#include "lua.h"
	#include "lualib.h"
#include "lauxlib.h"
}



struct MyTest
{
	int m_bla;
	MyTest():m_bla(10)
	{
		
	}
	
	void methodPrint()
	{
		printf("t->m_bla=%d\n",m_bla);
	}

};

std::vector<MyTest*> gValidPointers;

bool isValidPointer(MyTest* ptr)
{
	bool result = false;

	for (int i=0;i<gValidPointers.size();i++)
	{
		if (gValidPointers[i]==ptr)
			result = true;
	}
	return result;
}

bool removeValidPointer(MyTest* ptr)
{
	bool result = false;
	for (int i=0;i<gValidPointers.size();i++)
	{
		if (gValidPointers[i]==ptr)
		{
			gValidPointers[i]=0;
			gValidPointers[i] = gValidPointers[gValidPointers.size()-1];
			gValidPointers.pop_back();
			result = true;
		}
	}
	return result;
}


int my_printTest(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc==1)
	{
		MyTest* tst = (MyTest*) lua_touserdata(L,1);
		
		if (isValidPointer(tst))
		{
			tst->methodPrint();
		} else
		{
			 std::cerr << "error my_printTest called with invalid argument ";
		}
	}
	return 0;
}

int my_createFunction(lua_State *L)
{
  int argc = lua_gettop(L);

  std::cerr << "-- my_function() called with " << argc
    << " arguments:" << std::endl;

  for ( int n=1; n<=argc; ++n ) {
    std::cerr << "-- argument " << n << ": "
      << lua_tostring(L, n) << std::endl;
  }

  

  MyTest* newTest = new MyTest();
  gValidPointers.push_back(newTest);
  lua_pushlightuserdata (L, newTest);



  return 1; // number of return values
}

int my_deleteFunction(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc==1)
	{
		MyTest* tst = (MyTest*) lua_touserdata(L,1);
		if (tst)
		{
			bool result = removeValidPointer(tst);
			if (result)
			{
				delete tst;
			} else
			{
				 std::cerr << "error my_deleteFunction called with invalid pointer argument ";
			}
		} else
		{
			 std::cerr << "error my_deleteFunction called with non-pointer argument ";
		}
	}

  return 0; // number of return values
}



void report_errors(lua_State *L, int status)
{
  if ( status!=0 ) {
    std::cerr << "-- " << lua_tostring(L, -1) << std::endl;
    lua_pop(L, 1); // remove error message
  }
}

int main(int argc, char** argv)
{
  for ( int n=1; n<argc; ++n ) {
    const char* file = argv[n];

    lua_State *L = luaL_newstate();

    luaopen_io(L); // provides io.*
    luaopen_base(L);
    luaopen_table(L);
    luaopen_string(L);
    luaopen_math(L);
    //luaopen_package(L);
	luaL_openlibs(L);

	 // make my_function() available to Lua programs
    lua_register(L, "my_createFunction", my_createFunction);
	lua_register(L, "my_printTest", my_printTest);
	lua_register(L, "my_deleteFunction", my_deleteFunction);
	


    std::cerr << "-- Loading file: " << file << std::endl;

    int s = luaL_loadfile(L, file);

    if ( s==0 ) {
      // execute Lua program
      s = lua_pcall(L, 0, LUA_MULTRET, 0);
    }

    report_errors(L, s);
    lua_close(L);
    std::cerr << std::endl;
  }

  return 0;

}