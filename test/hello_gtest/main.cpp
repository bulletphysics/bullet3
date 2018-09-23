/////////////////////////////
// In the header file

#include <sstream>
using namespace std;

class Salutation
{
public:
	static string greet(const string& name);
};

///////////////////////////////////////
// In the class implementation file

string Salutation::greet(const string& name)
{
	ostringstream s;
	s << "Hello " << name << "!";
	return s.str();
}

///////////////////////////////////////////
// In the test file
#include <gtest/gtest.h>

TEST(SalutationTest, Static)
{
	EXPECT_EQ(string("Hello World!"), Salutation::greet("World"));
}

int main(int argc, char** argv)
{
#if _MSC_VER
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//void *testWhetherMemoryLeakDetectionWorks = malloc(1);
#endif
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
