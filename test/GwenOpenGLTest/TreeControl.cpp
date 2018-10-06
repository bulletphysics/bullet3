#include "UnitTest.h"
#include "Gwen/Controls/TreeControl.h"

using namespace Gwen;

using namespace Gwen::Controls;

class TreeControl2 : public GUnit
{
public:
	GWEN_CONTROL_INLINE(TreeControl2, GUnit)
	{
		{
			Gwen::Controls::TreeControl* ctrl = new Gwen::Controls::TreeControl(this);

			ctrl->SetKeyboardInputEnabled(true);
			ctrl->AddNode(L"Node One");
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
			pNode->SetSelected(true);

			pNode->AddNode(L"Brown")->AddNode(L"Node Two Inside")->AddNode(L"Eyes")->AddNode(L"Brown");
			ctrl->AddNode(L"Node Three");
			ctrl->Focus();
			ctrl->SetKeyboardInputEnabled(true);

			ctrl->SetBounds(30, 30, 200, 200);
			ctrl->ExpandAll();
		}

		{
			Gwen::Controls::TreeControl* ctrl = new Gwen::Controls::TreeControl(this);

			ctrl->AllowMultiSelect(true);

			ctrl->AddNode(L"Node One");
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
			Gwen::Controls::TreeNode* pNodeTwo = pNode->AddNode(L"Brown")->AddNode(L"Node Two Inside")->AddNode(L"Eyes");
			pNodeTwo->AddNode(L"Brown");
			pNodeTwo->AddNode(L"Green");
			pNodeTwo->AddNode(L"Slime");
			pNodeTwo->AddNode(L"Grass");
			pNodeTwo->AddNode(L"Pipe");

			ctrl->AddNode(L"Node Three");

			ctrl->SetBounds(240, 30, 200, 200);
			ctrl->ExpandAll();
		}
	}
};

DEFINE_UNIT_TEST(TreeControl2, L"TreeControl");